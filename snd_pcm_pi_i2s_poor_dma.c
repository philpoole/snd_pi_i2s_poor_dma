/* Raspberry Pi PCM I2S ALSA Driver
 *  Copyright (c) by Phil Poole 2013
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
   This is an ALSA PCM I2S driver for the Raspberry Pi.
   By using this driver, it is possible to output I2S on GPIO pins.
   Note, this driver is currently hardwired to only support stereo,
   16 bits per sample, I2S at a 44100Hz sampling frequency - for instance
   to drive a TDA1541A DAC.
   If you need to output a different format or frequency, then 
   modification may well be required.

   This file is strongly based on the dummy ALSA driver example, dummy.c
   written by Jaroslav Kysela. Thanks for the inspiration.
*/

#include <asm/system.h>

#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <sound/core.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/info.h>
#include <sound/initval.h>


#include <linux/dma-mapping.h>

#include <mach/dma.h>
#include <mach/irqs.h>

MODULE_AUTHOR("Phil Poole");
MODULE_DESCRIPTION("Raspberry Pi PCM I2S ALSA driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("{{ALSA,PCM I2S}}");

/*IO stuff*/
#include <linux/ioport.h>
#include <asm/io.h>
#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define I2S_BASE                (BCM2708_PERI_BASE + 0x203000) /* GPIO controller */
#define CLOCK_BASE               (BCM2708_PERI_BASE + 0x101000) /* Clocks */
#define I2S_FIFO_PHYSICAL_ADDR    0x7E203004

#define CS_A     0
#define FIFO_A   1
#define MODE_A   2
#define RXC_A    3
#define TXC_A    4
#define DREQ_A   5
#define INTEN_A  6
#define INTSTC_A 7
#define GRAY     8
const char *i2s_register_name[] = {"CS_A", "FIFO_A", "MODE_A", "RXC_A", "TXC_A", "DREQ_A", "INTEN_A", "INTSTC_A", "GRAY"};


static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;	/* Index 0-MAX */
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;	/* ID for this card */
struct pi_i2s_model {
	const char *name;
	int (*playback_constraints)(struct snd_pcm_runtime *runtime);
	int (*capture_constraints)(struct snd_pcm_runtime *runtime);
	u64 formats;
        size_t buffer_bytes_max;
	size_t period_bytes_min;
	size_t period_bytes_max;
	unsigned int periods_min;
	unsigned int periods_max;
	unsigned int rates;
	unsigned int rate_min;
	unsigned int rate_max;
	unsigned int channels_min;
	unsigned int channels_max;
};

struct snd_pi_i2s {
	struct snd_card *card;
	struct pi_i2s_model *model;
	struct snd_pcm *pcm;
	struct snd_pcm_hardware pcm_hw;
	spinlock_t mixer_lock;
	const struct pi_i2s_timer_ops *timer_ops;
};

/*
 * card models
 */
struct pi_i2s_model model_tda1541 = {
	.name = "tda1541",
	.buffer_bytes_max = (64*1024),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.channels_min = 2,
	.channels_max = 2,
	.periods_min = 1,
	.periods_max = 1,  
};


static unsigned int isr_real_pointer = 0;


/*
 * DMA stuff
 */
#define BCM2708_DMA_DREQ_PCM 2
static void __iomem	       *dma_chan_base;
static int dma_chan;
static int dma_irq_number;
static struct snd_pcm_substream * ref_substream;

static unsigned int pi_i2s_opened = 0;

static int current_dma_block=0;
static int queued_dma_block=1;
static int next_dma_block=2;

static struct bcm2708_dma_cb *control_block[3];
static dma_addr_t control_block_phys[3];
static struct bcm2708_dma_cb *sentinel_block[3];
static dma_addr_t sentinel_block_phys[3];
static int *status_block[3];
static dma_addr_t status_block_phys[3];
static int *status_block_dest;
static dma_addr_t status_block_dest_phys;
static int *src_addr[3];
static dma_addr_t src_addr_phys[3];
static int *silent_data;
static dma_addr_t silent_data_phys;


static unsigned long dma_irq_flags=0;

static unsigned int *i2s_registers;
static unsigned int dma_int_aborted = 0;

static unsigned int started=0;


static irqreturn_t dma_interrupt_handler(int irq, void *dev_id)
{
    static int valid =0;
    local_irq_save(dma_irq_flags);

    /*acknowledge interrupt*/
    writel(BCM2708_DMA_INT,
		       dma_chan_base + BCM2708_DMA_CS);



    if(*(i2s_registers+CS_A) & ((1<<15)|(1<<13)))
    {

        *(i2s_registers+CS_A) &= ~(1<<2);  /*Disable transmission*/
        *(i2s_registers+CS_A) |= (1<<3);   /*Clear TX FIFO*/
#if 0
        /*use sync flag to wait 2 PCM clocks*/
        isr_temp = *(i2s_registers+CS_A) & (1<<24);
        *(i2s_registers+CS_A) |= ~isr_temp;
        while(~isr_temp != *(i2s_registers+CS_A) & (1<<24) )
        {
            /*do nothing*/
        }
#endif
        *(i2s_registers+CS_A) |= (1<<2);    /*Enable transmission*/
        /*shouldn't we clear the error?*/
        *(i2s_registers+CS_A)|= (1<<13);
    }

    valid=status_block_dest[2];

    status_block[current_dma_block][2] = 0; /*invalid - set to valid in copy()*/
    control_block[current_dma_block]->src = silent_data_phys;

    if(++current_dma_block>=3)
        current_dma_block=0;
    if(++queued_dma_block>=3)
        queued_dma_block=0;
    if(++next_dma_block>=3)
        next_dma_block=0;
    if(!dma_int_aborted)
        bcm_dma_start(dma_chan_base,control_block_phys[current_dma_block]);

    if(valid == 1 && pi_i2s_opened == 1) /*valid, so notify*/
    {
    /*    real_isr_pointer+=status_block_dest[0];*/
        snd_pcm_period_elapsed(ref_substream);
    } 

    local_irq_restore(dma_irq_flags);
    return IRQ_HANDLED;
}

static struct irqaction i2s_dma_irq = {
   .name = "I2S DMA",
   .flags = 0,
   .handler = dma_interrupt_handler,
};

/*
 * PCM interface
 */

unsigned int *gpio;
unsigned int *clock_registers;

#define NUMBER_OF_BUFFERS 128
static void *pi_i2s_page[NUMBER_OF_BUFFERS];


static void setup_control_blocks()
{
    int i = 0;
    for(i=0; i<3; i++)
    {

        control_block[i]->info = BCM2708_DMA_PER_MAP(BCM2708_DMA_DREQ_PCM) |
                     BCM2708_DMA_S_INC|
		     BCM2708_DMA_D_DREQ;		  
        control_block[i]->src=src_addr_phys[i];
        control_block[i]->dst=I2S_FIFO_PHYSICAL_ADDR;
        control_block[i]->length=4096;
        control_block[i]->stride=0;
        control_block[i]->next=sentinel_block_phys[i];
        control_block[i]->pad[0]=0;
        control_block[i]->pad[1]=0;
   
        sentinel_block[i]->info=   
                     BCM2708_DMA_S_INC|
		     BCM2708_DMA_D_INC|BCM2708_DMA_INT_EN ;
        sentinel_block[i]->src=status_block_phys[i];
        sentinel_block[i]->dst=status_block_dest_phys;
        sentinel_block[i]->length=16;
        sentinel_block[i]->stride=0;
        sentinel_block[i]->next=0;
        sentinel_block[i]->pad[0]=0;
        sentinel_block[i]->pad[1]=0;

    }
}

static int pcm_copy_call_count=0;


static int pi_i2s_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
        int err = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
             /*   pcm_copy_call_count=0;
                started=1;*/
                return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
               /* bcm_dma_abort(dma_chan_base);
                don't set up i2s interrupts at the moment
                started=0;
                pcm_copy_call_count=0;
                real_isr_pointer = 0; */
	        return 0;
        }
	return -EINVAL;
}

static int pi_i2s_pcm_prepare(struct snd_pcm_substream *substream)
{
	/*printk(KERN_INFO "PCM PREPARE\n");*/
        setup_control_blocks();
        ref_substream = substream; /*could use status block[n] here*/
        return 0;
}

static snd_pcm_uframes_t pi_i2s_pcm_pointer(struct snd_pcm_substream *substream)
{
    /*printk(KERN_EMERG "POINTER %d\n", isr_real_pointer);*/
    return (snd_pcm_uframes_t) isr_real_pointer;
}

static struct snd_pcm_hardware pi_i2s_pcm_hardware = {
	.info =			(SNDRV_PCM_INFO_MMAP |
				 SNDRV_PCM_INFO_INTERLEAVED |
				 SNDRV_PCM_INFO_RESUME |
				 SNDRV_PCM_INFO_MMAP_VALID),
	.formats =		SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_44100,
	.rate_min =		44100,
	.rate_max =		44100,
	.channels_min =		2,
	.channels_max =		2,
	.buffer_bytes_max =	(64*1024), 
	.period_bytes_min =	(4*1024),
	.period_bytes_max =     (4*1024),
	.periods_min =		1,
	.periods_max =		1,       
	.fifo_size =		0,
};

static int pi_i2s_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{

    /* runtime->dma_bytes has to be set manually to allow mmap */
    substream->runtime->dma_bytes = params_buffer_bytes(hw_params);
    return 0;
    /*return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));*/
}

static int pi_i2s_pcm_hw_free(struct snd_pcm_substream *substream)
{
    return 0;
    /*return snd_pcm_lib_free_pages(substream);*/
}

static int pi_i2s_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pi_i2s *dummy = snd_pcm_substream_chip(substream);
    struct pi_i2s_model *model = dummy->model;
    struct snd_pcm_runtime *runtime = substream->runtime;
    int err; 
    printk(KERN_INFO "PCM OPEN\n");
    runtime->hw = dummy->pcm_hw;
    if (substream->pcm->device & 1) {
        printk(KERN_INFO "noninterleaved\n");
        runtime->hw.info &= ~SNDRV_PCM_INFO_INTERLEAVED;
        runtime->hw.info |= SNDRV_PCM_INFO_NONINTERLEAVED;
    }
    if (substream->pcm->device & 2){
        printk(KERN_INFO "NOTMMAP\n");
        runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
				      SNDRV_PCM_INFO_MMAP_VALID);
    }
    if (model == NULL)
        return 0;

    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
        if (model->playback_constraints)
            err = model->playback_constraints(substream->runtime);
    } else {
        if (model->capture_constraints)
			err = model->capture_constraints(substream->runtime);
    }
    pi_i2s_opened = 1;
    return 0;
}

static int pi_i2s_pcm_close(struct snd_pcm_substream *substream)
{
    pi_i2s_opened = 0;
    printk(KERN_INFO "PCM CLOSE\n");
    return 0;
}
static int pi_i2s_pcm_copy(struct snd_pcm_substream *substream,
			  int channel, snd_pcm_uframes_t pos,
			  void __user *dst, snd_pcm_uframes_t count)
{
   /* printk(KERN_EMERG "COPY %d\n", count);*/
    if(pcm_copy_call_count==0)
    {
        printk(KERN_EMERG "COPY INIT\n");
        current_dma_block=0;
        queued_dma_block=1;
        next_dma_block=2;
        /*control_block[current_dma_block]->src = silent_data_phys;*/
        control_block[current_dma_block]->length = 4096;
       /* control_block[queued_dma_block]->src = silent_data_phys;*/
        control_block[queued_dma_block]->length = 4096;
    }
    

    copy_from_user(src_addr[next_dma_block], dst, count*4);
    control_block[next_dma_block]->src = src_addr_phys[next_dma_block];
    control_block[next_dma_block]->length = count*4;
    isr_real_pointer+=count; /*a bit of a cludge, but what isn't?*/
    status_block[queued_dma_block][0] = count;
    status_block[next_dma_block][0] = count; /*just in case*/
    status_block[queued_dma_block][2] = 1;  /*valid, so notify*/
    status_block[next_dma_block][2] = 1;  /*valid, so notify*/
    if(pcm_copy_call_count==0)
    {
        bcm_dma_start(dma_chan_base,control_block_phys[0]);
 
    }
    pcm_copy_call_count++;
    return 0; 
}



static int pi_i2s_pcm_silence(struct snd_pcm_substream *substream,
			     int channel, snd_pcm_uframes_t pos,
			     snd_pcm_uframes_t count)
{
	return 0; /* do nothing */
}

static struct page *pi_i2s_pcm_page(struct snd_pcm_substream *substream,
				   unsigned long offset)
{
        /*does this need to change?*/
        printk(KERN_INFO "PAGE\n");
	return virt_to_page(pi_i2s_page[substream->stream]); /* the same page */
}

static struct snd_pcm_ops pi_i2s_pcm_ops = {
	.open =		pi_i2s_pcm_open,
	.close =	pi_i2s_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	pi_i2s_pcm_hw_params,
	.hw_free =	pi_i2s_pcm_hw_free,
	.prepare =	pi_i2s_pcm_prepare,
	.trigger =	pi_i2s_pcm_trigger,
	.pointer =	pi_i2s_pcm_pointer,
	.copy =		pi_i2s_pcm_copy,
	.silence =	pi_i2s_pcm_silence,
	.page =		pi_i2s_pcm_page,
};

static int __devinit snd_card_pi_i2s_pcm(struct snd_pi_i2s *dummy, int device,
					int substreams)
{
	struct snd_pcm *pcm;
	struct snd_pcm_ops *ops;
	int err;

	err = snd_pcm_new(dummy->card, "Dummy PCM", device,
			       substreams, substreams, &pcm);
	if (err < 0)
		return err;
	dummy->pcm = pcm;
		ops = &pi_i2s_pcm_ops;
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, ops);
	pcm->private_data = dummy;
	pcm->info_flags = 0;
	strcpy(pcm->name, "Raspberry Pi PCM I2S");

	return 0;
}



void setup_io(void);
void setup_i2s(void);
void setup_gpio(void);
void release_the_hounds(void);


static int __devinit snd_pi_i2s_probe(struct platform_device *pdev)
{
    struct snd_card *card;
    struct snd_pi_i2s *dummy;
    int err=0;
    int i=0;
    int dev = pdev->id;

    /*setup DMA*/

    err = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST,
				 &dma_chan_base,
				 &dma_irq_number);
    if (err < 0) {
        printk(KERN_EMERG "couldn't allocate a DMA channel\n");
    }
    else{
        printk(KERN_EMERG "Allocated DMA channel. IRQ %d\n", dma_irq_number);
        dma_chan = err;
    }

    err = setup_irq(dma_irq_number, &i2s_dma_irq);
    if(err)
           printk(KERN_EMERG "Error - setup_irq()\n");

    for(i = 0; i < 3; i++)
    {
        control_block[i] =  dma_alloc_coherent(&pdev->dev, SZ_64K, &control_block_phys[i], GFP_KERNEL);
        sentinel_block[i]  = dma_alloc_coherent(&pdev->dev, SZ_64K, &sentinel_block_phys[i], GFP_KERNEL);
        status_block[i] =  dma_alloc_coherent(&pdev->dev, SZ_64K, &status_block_phys[i], GFP_KERNEL);
        src_addr[i] =  dma_alloc_coherent(&pdev->dev, SZ_64K, &src_addr_phys[i], GFP_KERNEL);
        
        memset(control_block[i], 0, SZ_64K); 
        memset(status_block[i], 0, SZ_64K);
        memset(sentinel_block[i], 0, SZ_64K); 
    
    }
    status_block_dest  = dma_alloc_coherent(&pdev->dev, SZ_64K, &status_block_dest_phys, GFP_KERNEL);
    silent_data =  dma_alloc_coherent(&pdev->dev, SZ_64K, &silent_data_phys, GFP_KERNEL);

    memset(silent_data, 0, SZ_64K); 
    

    setup_io();


    err = snd_card_create(index[dev], id[dev], THIS_MODULE,
			      sizeof(struct snd_pi_i2s), &card);
    if (err < 0)
        return err;
    dummy = card->private_data;
    dummy->card = card;
    dummy->model = &model_tda1541;
    err = snd_card_pi_i2s_pcm(dummy, 0, 1);
    if (err < 0)
        goto __nodev;
    dummy->pcm_hw = pi_i2s_pcm_hardware;
	
    strcpy(card->driver, "Raspberry Pi I2S PCM");
    strcpy(card->shortname, "Pi I2s");
    sprintf(card->longname, "Raspberry Pi I2S PCM Driver %i", dev + 1);

    snd_card_set_dev(card, &pdev->dev);
    setup_control_blocks();
    dma_int_aborted=0;
    err = snd_card_register(card);
    if (err == 0) {
        platform_set_drvdata(pdev, card);
        return 0;
    }
    __nodev:
    snd_card_free(card);
    return err;
}

static int __devexit snd_pi_i2s_remove(struct platform_device *pdev)
{
    int i=0;
    printk(KERN_EMERG "REMOVING\n");
   
 
    snd_card_free(platform_get_drvdata(pdev));
    platform_set_drvdata(pdev, NULL);

    dma_int_aborted=1;
	
    /*free DMA (wait for completion)?*/
    bcm_dma_chan_free(dma_chan);

#if 0
    /*might need to do something here like this...*/

    if(status_block_dest==1)
        cb_base->next=0;
    else
        cb_base2->next=0;
#endif
    msleep(1000);
    remove_irq(dma_irq_number, &i2s_dma_irq);
    
    for(i=0; i< 3; i++)
    {
        dma_free_coherent(&pdev->dev, SZ_64K, control_block[i], control_block_phys[i]);
        dma_free_coherent(&pdev->dev, SZ_64K, sentinel_block[i], sentinel_block_phys[i]);
        dma_free_coherent(&pdev->dev, SZ_64K, status_block[i], status_block_phys[i]);
        dma_free_coherent(&pdev->dev, SZ_64K, src_addr[i], src_addr_phys[i]);
    }
    dma_free_coherent(&pdev->dev, SZ_64K, status_block_dest, status_block_dest_phys);
    dma_free_coherent(&pdev->dev, SZ_64K, silent_data, silent_data_phys);

    release_the_hounds();   
    return 0;
}

#ifdef CONFIG_PM
/* I don't expect these to work */
static int snd_pi_i2s_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_card *card = platform_get_drvdata(pdev);
	struct snd_pi_i2s *dummy = card->private_data;

	snd_power_change_state(card, SNDRV_CTL_POWER_D3hot);
	snd_pcm_suspend_all(dummy->pcm);
	return 0;
}
	
static int snd_pi_i2s_resume(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	snd_power_change_state(card, SNDRV_CTL_POWER_D0);
	return 0;
}
#endif

#define SND_PI_I2S_DRIVER	"snd_pi_i2s_pcm"

static struct platform_driver snd_pi_i2s_driver = {
    .probe = snd_pi_i2s_probe,
    .remove = __devexit_p(snd_pi_i2s_remove),
#ifdef CONFIG_PM
    .suspend = snd_pi_i2s_suspend,
    .resume = snd_pi_i2s_resume,
#endif
    .driver = {
        .name = SND_PI_I2S_DRIVER
    },
};

#define DMA_MASK_BITS_COMMON 32
static u64 systemtimer_dmamask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON);
struct platform_device snd_pi_i2s_device = {
        .name = SND_PI_I2S_DRIVER,
	.dev = {
		.dma_mask = &systemtimer_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(DMA_MASK_BITS_COMMON),
		},
};


static void snd_pi_i2s_unregister_all(void)
{
    platform_device_unregister(&snd_pi_i2s_device);
    platform_driver_unregister(&snd_pi_i2s_driver);

}



static int __init alsa_card_pi_i2s_init(void)
{
	int  cards, err = 1;
	err = platform_driver_register(&snd_pi_i2s_driver);
	if (err < 0)
		return err;
	cards = 0;

	platform_device_register(&snd_pi_i2s_device);						 
        printk(KERN_INFO "READY FOR ACTION\n");
	return 0;
}

static void __exit alsa_card_pi_i2s_exit(void)
{        
    snd_pi_i2s_unregister_all();
    return;    
}

void setup_io(void)
{
    setup_gpio();
    setup_i2s();
}

void setup_gpio(void)
{

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

	int pin;
	gpio = ioremap(GPIO_BASE, SZ_16K);

	/* SPI is on GPIO 7..11 */
	for (pin = 28; pin <= 31; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 2);	/* set mode to ALT 0 */
	}

#undef INP_GPIO
#undef SET_GPIO_ALT
    printk(KERN_INFO "GPIO SETUP\n");

}

void setup_i2s(void)
{
    int i=0;
    i2s_registers = ioremap(I2S_BASE, 32);
    clock_registers = ioremap(CLOCK_BASE, 2);
    
    printk(KERN_INFO "Disabling I2S clock\n");
    *(clock_registers+0x26) = 0x5A000000;
    *(clock_registers+0x27) = 0x5A000000;
     
    udelay(10);

    printk(KERN_INFO "Configure I2S clock\n");
    *(clock_registers+0x26) = 0x5A000001;

    *(clock_registers+0x27) = 0x5A006CD7; /*a calculated guess, hopefully about 6.8, but seems out somewhat (I want 64*44100 = 2822400 Hz)*/
    udelay(10);
    printk(KERN_INFO "Enabling I2S clock\n");
    *(clock_registers+0x26) = 0x5A000611;

    /* disable I2S so we can modify the regs*/
    printk(KERN_INFO "Disable I2S\n");

    *(i2s_registers+CS_A) &= ~(1<<24); /*just for completeness*/
    *(i2s_registers+CS_A) = 0;
    *(i2s_registers+MODE_A) = 0;
    *(i2s_registers+TXC_A) = 0;
    *(i2s_registers+RXC_A) = 0;
    *(i2s_registers+GRAY) = 0;
    udelay(100);

  
  /* set register settings
     --> enable Channel1 with 32bit width
     For 16 bit I2S, Channel width should be 16, possible positions should be 1 and 17?
    (e.g. look for 1<<29 and 17<<4 */
    printk(KERN_INFO "Setting TX channel settings\n");
    *(i2s_registers+TXC_A) = 0<<31 | 1<<30 | 1 << 20  | 8<<16   |   0<<15 | 1<<14 | 33<<4 | 8 ;

    /*Set frame length and frame sync length (32 and 16), and set FTXP=1 so I can inject 2 channels into a single 32 bit word*/
    *(i2s_registers+MODE_A) = 1 << 24 | 63<<10 | 32;

    /* --> disable STBY */
    printk(KERN_INFO "disabling standby\n");
    *(i2s_registers+CS_A) |= 1<<25;
    udelay(50);

    *(i2s_registers+CS_A) |= 1<<3  /*|1<<4*/; /* clear TX FIFO*/

    *(i2s_registers+CS_A) |=( 1 <<5); /*2 seems to avoid errors (until interrupts are stalled)*/
    for(i = 0; i < 32; i++)
       (*(i2s_registers+FIFO_A)) = 0;
    /* --> ENABLE SYNC bit*/
    printk(KERN_INFO "setting sync bit high\n");
    *(i2s_registers+CS_A) |= 0<<24;

    if (*(i2s_registers+CS_A) & 1<<24) {
        printk(KERN_INFO "SYNC bit high, strange.\n");
    } else {
        printk(KERN_INFO "SYNC bit low, as expected.\n");
    }

    udelay(1);

    if (*(i2s_registers+CS_A) & 1<<24) {
        printk(KERN_INFO "SYNC bit high, as expected.\n");
    } else {
        printk(KERN_INFO "SYNC bit low, strange.\n");
    }



    /*SETUP DREQ*/
    *(i2s_registers+CS_A) = (1<<9);
    *(i2s_registers+DREQ_A) = 0x10303020;/*0x18303020*/
    printk(KERN_EMERG "DREQ levels %x\n", *(i2s_registers+DREQ_A));
    printk(KERN_INFO "ENABLE I2S\n");
    /* enable I2S*/
    *(i2s_registers+CS_A) |= 0x01;

    printk(KERN_INFO "ENABLE TRANSMISSION\n");   
    /* enable transmission*/
    *(i2s_registers+CS_A) |= 0x04;
    printk(KERN_INFO "I2S SETUP COMPLETE\n");
    return;
}

void release_the_hounds(void)
{
    /* Clear everything - Monty Burns style :) */
    /*disable interrupts first*/
    *(i2s_registers+INTSTC_A) = 0x000f;
    *(i2s_registers+INTEN_A) = 0x00; 
    *(i2s_registers+CS_A) &= ~(1<<24);
    udelay(100);
    *(i2s_registers+CS_A) |= 0x00;

    iounmap(i2s_registers);
    iounmap(gpio);
    iounmap(clock_registers);
    return;
}
module_init(alsa_card_pi_i2s_init)
module_exit(alsa_card_pi_i2s_exit)
