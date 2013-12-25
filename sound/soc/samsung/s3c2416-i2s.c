/*
 * s3c24xx-i2s.c  --  ALSA Soc Audio Layer
 *
 * (c) 2006 Wolfson Microelectronics PLC.
 * Graeme Gregory graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 * Copyright 2004-2005 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
*  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <mach/regs-gpio.h>
#include <mach/dma.h>
#include <plat/regs-iis.h>
#include <mach/regs-s3c2443-clock.h>
#include "dma.h"
#include "s3c24xx-i2s.h"

#define S3C24XX_I2S_DEBUG 1
#if S3C24XX_I2S_DEBUG
#define DBG(x...) printk(KERN_DEBUG "s3c24xx-i2s: " x)
#else
#define DBG(x...)
#endif

static struct s3c2410_dma_client s3c24xx_dma_client_out = {
	.name = "I2S PCM Stereo out"
};

static struct s3c2410_dma_client s3c24xx_dma_client_in = {
	.name = "I2S PCM Stereo in"
};

static struct s3c_dma_params s3c24xx_i2s_pcm_stereo_out = {
	.client		= &s3c24xx_dma_client_out,
	.channel	= DMACH_I2S_OUT,
	.dma_addr	= S3C2410_PA_IIS + S3C2416_IISFIFOTX,
	.dma_size	= 4,            /*2416 必须以 2words 写FIFO*/
};

static struct s3c_dma_params s3c24xx_i2s_pcm_stereo_in = {
	.client		= &s3c24xx_dma_client_in,
	.channel	= DMACH_I2S_IN,
	.dma_addr	= S3C2410_PA_IIS + S3C2416_IISFIFORX,
	.dma_size	= 4,
};

struct s3c24xx_i2s_info {
	void __iomem	*regs;
	struct clk	*iis_clk;
	int master;
	u32		iiscon;
	u32		iismod;
	u32		iisfcon;
	u32		iispsr;
};
static struct s3c24xx_i2s_info s3c24xx_i2s;

static void s3c24xx_snd_txctrl(int on)
{

	u32 iiscon;
    u32 iismod;
    u32 iisfic, iispsr;
    u32 reg;
	DBG("Entered %s, on=%d\n", __func__, on);

    iisfic = readl(s3c24xx_i2s.regs + S3C2416_IISFIC);
    iispsr = readl(s3c24xx_i2s.regs + S3C2416_IISPSR);
	iiscon  = readl(s3c24xx_i2s.regs + S3C2410_IISCON);
	iismod  = readl(s3c24xx_i2s.regs + S3C2410_IISMOD);
	DBG("r: IISCON: %x IISMOD: %x IISFCON: %x IISPSR: %x\n", iiscon, iismod, iisfic, iispsr);
    
	if (on) {

		iiscon |= S3C2416_IISCON_TXDMACTIVE;
		iiscon &= ~S3C2416_IISCON_FTXURINTEN;//disable under-run interrupt
		iiscon |= S3C2416_IISCON_I2SACTIVE;
        
		writel(iiscon,  s3c24xx_i2s.regs + S3C2410_IISCON);
        writel(iismod,  s3c24xx_i2s.regs + S3C2410_IISMOD);
        
	} else {
		/* note, we have to disable the FIFOs otherwise bad things
		 * seem to happen when the DMA stops. According to the
		 * Samsung supplied kernel, this should allow the DMA
		 * engine and FIFOs to reset. If this isn't allowed, the
		 * DMA engine will simply freeze randomly.
		 */
//		iiscon  &= ~S3C2416_IISCON_I2SACTIVE;
		iiscon &= ~S3C2416_IISCON_TXDMACTIVE;
		writel(iiscon,  s3c24xx_i2s.regs + S3C2410_IISCON);
	}
    DBG("w: IISCON: %x IISMOD: %x IISFCON: %x IISPSR: %x\n", iiscon, iismod, iisfic, iispsr);
}

static void s3c24xx_snd_rxctrl(int on)
{

	u32 iiscon, iisfic, iismod, iispsr;
	DBG("Entered %s\n", __func__);
    iisfic = readl(s3c24xx_i2s.regs + S3C2416_IISFIC);
    iispsr = readl(s3c24xx_i2s.regs + S3C2416_IISPSR);
	iiscon  = readl(s3c24xx_i2s.regs + S3C2410_IISCON);
	iismod  = readl(s3c24xx_i2s.regs + S3C2410_IISMOD);
	DBG("r: IISCON: %x IISMOD: %x IISFCON: %x IISPSR: %x\n", iiscon, iismod, iisfic, iispsr);
    
	if (on) {

		iiscon	|= S3C2416_IISCON_RXDMACTIVE;
		iiscon  |= S3C2416_IISCON_I2SACTIVE;
		writel(iiscon,  s3c24xx_i2s.regs + S3C2410_IISCON);
	} else {
		/* note, we have to disable the FIFOs otherwise bad things
		 * seem to happen when the DMA stops. According to the
		 * Samsung supplied kernel, this should allow the DMA
		 * engine and FIFOs to reset. If this isn't allowed, the
		 * DMA engine will simply freeze randomly.
		 */
        /* 不关闭iis防止，双工工作时，停止录音对播放造成影响 */
//		iiscon  &= ~S3C2416_IISCON_I2SACTIVE;
		iiscon	&= ~S3C2416_IISCON_RXDMACTIVE;
		writel(iiscon,  s3c24xx_i2s.regs + S3C2410_IISCON);
	}
    DBG("w: IISCON: %x IISMOD: %x IISFCON: %x IISPSR: %x\n", iiscon, iismod, iisfic, iispsr);

}

/*
 * Wait for the LR signal to allow synchronisation to the L/R clock
 * from the codec. May only be needed for slave mode.
 */
static int s3c24xx_snd_lrsync(void)
{
	u32 iiscon;

	int timeout=jiffies + msecs_to_jiffies(5);

	DBG("Entered %s\n", __func__);

	while (1) {
		iiscon = readl(s3c24xx_i2s.regs + S3C2410_IISCON);
		if (iiscon & S3C2410_IISCON_LRINDEX)
			break;
		if (timeout < jiffies)
			return -ETIMEDOUT;
		udelay(100);
	}

	return 0;
}

/*
 * Check whether CPU is the master or slave
 */
static inline int s3c24xx_snd_is_clkmaster(void)
{
	DBG("Entered %s\n", __func__);
	return (readl(s3c24xx_i2s.regs + S3C2410_IISMOD) & S3C2416_IISMOD_SLAVE_PCLK)? 0:1;
}

/*
 * Set S3C24xx I2S DAI format
 */
static int s3c24xx_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	u32 iismod;

	iismod = readl(s3c24xx_i2s.regs + S3C2410_IISMOD);
	DBG("hw_params r: IISMOD: %x \n", iismod);
	iismod &= ~S3C2416_IISMOD_CLK_MASK;
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		//master mode use PCLK
		iismod |= S3C2416_IISMOD_MASTER_PCLK | S3C2416_IISMOD_INTERNAL_CLK;
        DBG("Master mode use PCLK\n");
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		//slave mode use PCLK
		iismod |= S3C2416_IISMOD_SLAVE_PCLK;
        DBG("Slave mode use PCLK\n");
		break;
	default:
		return -EINVAL;
	}
	iismod &= ~S3C2416_IISMOD_FM_MASK;//SDF
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_LEFT_J://MSB
		iismod |= S3C2416_IISMOD_MSB;

		break;
	case SND_SOC_DAIFMT_RIGHT_J://MSB
		iismod |= S3C2416_IISMOD_LSB;
		break;
	default: //I2S mode
		iismod |=S3C2416_IISMOD_IIS;
		break;
	}

	writel(iismod, s3c24xx_i2s.regs + S3C2410_IISMOD);

	return 0;
}

static int s3c24xx_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct s3c_dma_params *dma_data;
	u32 iismod;
	u32 iiscon;
	u32 iisfcon;
	u32 gpeup=0;

	DBG("Entered %s\n", __func__);

	writel((readl(S3C2410_MISCCR) & ~(7<<8))|(1<<8), S3C2410_MISCCR);
#if 0
	/* Configure the I2S pins in correct mode */
	writel(0x0, S3C2450_GPESEL);
	s3c2410_gpio_cfgpin(S3C2410_GPE0, S3C2410_GPE0_I2SLRCK);
	s3c2410_gpio_cfgpin(S3C2410_GPE1, S3C2410_GPE1_I2SSCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE2, S3C2410_GPE2_CDCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE3, S3C2410_GPE3_I2SSDI);
	s3c2410_gpio_cfgpin(S3C2410_GPE4, S3C2410_GPE4_I2SSDO);

	gpeup = readl(S3C2410_GPEUP)& ~(0x3ff);
	gpeup |= 0x1f;
	writel(gpeup, S3C2410_GPEUP);
#endif

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_data = &s3c24xx_i2s_pcm_stereo_out;
	else
		dma_data = &s3c24xx_i2s_pcm_stereo_in;

	snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, dma_data);

	/* Working copies of registers */
	iiscon = readl(s3c24xx_i2s.regs + S3C2410_IISCON);
	iismod = readl(s3c24xx_i2s.regs + S3C2410_IISMOD);
	iisfcon = readl(s3c24xx_i2s.regs + S3C2416_IISFIC);

	//TXR
	iismod &= ~S3C2416_IISMOD_MODE_MASK;
	iismod |= S3C2416_IISMOD_TXRXMODE; //Transmit and receive simultaneous mode


	iisfcon |= S3C2416_IIS_TX_FLUSH; //TX FIFO flush command
	iisfcon |= S3C2416_IIS_RX_FLUSH; //RX FIFO flush command
#if 0    
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
        iismod &=~S3C2416_IISMOD_BLC_MASK;
//        iismod |= S3C2416_IISMOD_BLC_8BITS;//UDA1431 supports 8bit 16bit 20bit
//        dma_data->dma_size = 1;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
         iismod &=~S3C2416_IISMOD_BLC_MASK;
         //       iismod |= S3C2416_IISMOD_BLC_16BITS;//UDA1431 supports 8bit 16bit 20bit
        dma_data->dma_size = 2;
		break;
	default:
		return -EINVAL;
	}
#endif
    iismod &=~S3C2416_IISMOD_BLC_MASK;
    iismod |= S3C2416_IISMOD_BLC_16BITS;//UDA1431 supports 8bit 16bit 20bit
    
	writel(iiscon, s3c24xx_i2s.regs + S3C2410_IISCON);
	writel(iismod, s3c24xx_i2s.regs + S3C2410_IISMOD);
	writel(iisfcon, s3c24xx_i2s.regs + S3C2416_IISFIC);

	/* Tx, Rx fifo flush bit clear */
	iisfcon  &= ~(S3C2416_IIS_TX_FLUSH | S3C2416_IIS_RX_FLUSH);
	writel(iisfcon, s3c24xx_i2s.regs + S3C2416_IISFIC);

	return 0;
}

static int s3c24xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;
	struct s3c_dma_params *dma_data =
		snd_soc_dai_get_dma_data(dai, substream);

	DBG("Entered %s\n", __func__);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (!s3c24xx_snd_is_clkmaster()) {
			ret = s3c24xx_snd_lrsync();
			if (ret)
				goto exit_err;
		}

		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			s3c24xx_snd_rxctrl(1);
		else
			s3c24xx_snd_txctrl(1);
		s3c2410_dma_ctrl(dma_data->channel, S3C2410_DMAOP_STARTED);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			s3c24xx_snd_rxctrl(0);
		else
			s3c24xx_snd_txctrl(0);
		break;
	default:
		ret = -EINVAL;
		break;
	}

exit_err:
	return ret;
}

/*
 * Set S3C24xx Clock source
 */
static int s3c24xx_i2s_set_sysclk(struct snd_soc_dai *cpu_dai,
	int clk_id, unsigned int freq, int dir)
{
	u32 iismod = readl(s3c24xx_i2s.regs + S3C2410_IISMOD);

	DBG("Entered %s\n", __func__);

	//because we setting IIS works in master mode  
	iismod &= ~S3C2416_IISMOD_CLK_MASK;

	iismod |= S3C2416_IISMOD_MASTER_PCLK | S3C2416_IISMOD_INTERNAL_CLK;
	writel((readl(S3C2410_MISCCR) & ~(7<<8))|(1<<8), S3C2410_MISCCR);

	writel(iismod, s3c24xx_i2s.regs + S3C2410_IISMOD);
	return 0;
}

/*
 * Set S3C24xx Clock dividers
 */
static int s3c24xx_i2s_set_clkdiv(struct snd_soc_dai *cpu_dai,
	int div_id, int div)
{
	u32 reg;


	switch (div_id) {
	case S3C24XX_DIV_BCLK: //1: setting BFS
//        div = 0<<1;        /*32fs*/
		reg = readl(s3c24xx_i2s.regs + S3C2410_IISMOD) & ~S3C2416_IISMOD_BFS_MASK;
		writel(reg | div, s3c24xx_i2s.regs + S3C2410_IISMOD);
		break;
	case S3C24XX_DIV_MCLK://0: setting RFS|FS  //BFS*BLC(16)==> RFS
//        div = 0<<3;       /* root fs 256fs*/
 		reg = readl(s3c24xx_i2s.regs + S3C2410_IISMOD) & ~S3C2416_IISMOD_FS_MASK;
		writel(reg | div, s3c24xx_i2s.regs + S3C2410_IISMOD);
		break;
	case S3C24XX_DIV_PRESCALER: //2: setting prescaler
//        div = 5<<8;             /*codec clock 256fs*/
		reg = readl(s3c24xx_i2s.regs + S3C2416_IISPSR);
		reg &= ~(S3C2416_IISPSR_PS_MASK|S3C2416_IISPSR_PSRAEN); //clear bits
		reg |= ( div |S3C2416_IISPSR_PSRAEN);//setting value
		writel(reg, s3c24xx_i2s.regs + S3C2416_IISPSR);
		break;
	default:
		return -EINVAL;
	}
	DBG("Entered %s, div_id=%d, div=%d\n", __func__,div_id, div);
	return 0;
}

/*
 * To avoid duplicating clock code, allow machine driver to
 * get the clockrate from here.
 */
u32 s3c24xx_i2s_get_clockrate(void)
{
	return clk_get_rate(s3c24xx_i2s.iis_clk);
}
EXPORT_SYMBOL_GPL(s3c24xx_i2s_get_clockrate);

static int s3c24xx_i2s_probe(struct snd_soc_dai *dai)
{
	DBG("Entered %s\n", __func__);

	s3c24xx_i2s.regs = ioremap(S3C2410_PA_IIS, 0x100);
	if (s3c24xx_i2s.regs == NULL)
		return -ENXIO;

	s3c24xx_i2s.iis_clk = clk_get(dai->dev, "iis");
	if (s3c24xx_i2s.iis_clk == NULL) {
		pr_err("failed to get iis_clock\n");
		iounmap(s3c24xx_i2s.regs);
		return -ENODEV;
	}

	clk_enable(s3c24xx_i2s.iis_clk);

	/* Configure the I2S pins in correct mode */
	s3c2410_gpio_cfgpin(S3C2410_GPE0, S3C2410_GPE0_I2SLRCK);
	s3c2410_gpio_cfgpin(S3C2410_GPE1, S3C2410_GPE1_I2SSCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE2, S3C2410_GPE2_CDCLK);
	s3c2410_gpio_cfgpin(S3C2410_GPE3, S3C2410_GPE3_I2SSDI);
	s3c2410_gpio_cfgpin(S3C2410_GPE4, S3C2410_GPE4_I2SSDO);
	s3c2410_gpio_pullup(S3C2410_GPE0, 0);
	s3c2410_gpio_pullup(S3C2410_GPE1, 0);
	s3c2410_gpio_pullup(S3C2410_GPE2, 0);
	s3c2410_gpio_pullup(S3C2410_GPE3, 0);
	s3c2410_gpio_pullup(S3C2410_GPE4, 0);
    
	writel(S3C2416_IISCON_I2SACTIVE, s3c24xx_i2s.regs + S3C2410_IISCON);

	s3c24xx_snd_txctrl(0);
	s3c24xx_snd_rxctrl(0);
	return 0;
}

#ifdef CONFIG_PM
static int s3c24xx_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	DBG("Entered %s\n", __func__);

	clk_disable(s3c24xx_i2s.iis_clk);

	return 0;
}

static int s3c24xx_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	DBG("Entered %s\n", __func__);
	clk_enable(s3c24xx_i2s.iis_clk);
	return 0;
}
#else
#define s3c24xx_i2s_suspend NULL
#define s3c24xx_i2s_resume NULL
#endif


#define S3C24XX_I2S_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

static struct snd_soc_dai_ops s3c24xx_i2s_dai_ops = {
	.trigger	= s3c24xx_i2s_trigger,
	.hw_params	= s3c24xx_i2s_hw_params,
	.set_fmt	= s3c24xx_i2s_set_fmt,
	.set_clkdiv	= s3c24xx_i2s_set_clkdiv,
	.set_sysclk	= s3c24xx_i2s_set_sysclk,
};

static struct snd_soc_dai_driver s3c24xx_i2s_dai = {
	.probe = s3c24xx_i2s_probe,
	.suspend = s3c24xx_i2s_suspend,
	.resume = s3c24xx_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = S3C24XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &s3c24xx_i2s_dai_ops,
};

static __devinit int s3c24xx_iis_dev_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &s3c24xx_i2s_dai);
}

static __devexit int s3c24xx_iis_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver s3c24xx_iis_driver = {
	.probe  = s3c24xx_iis_dev_probe,
	.remove = s3c24xx_iis_dev_remove,
	.driver = {
		.name = "s3c24xx-iis",
		.owner = THIS_MODULE,
	},
};

static int __init s3c24xx_i2s_init(void)
{

	return platform_driver_register(&s3c24xx_iis_driver);

}
module_init(s3c24xx_i2s_init);

static void __exit s3c24xx_i2s_exit(void)
{
	platform_driver_unregister(&s3c24xx_iis_driver);
}
module_exit(s3c24xx_i2s_exit);

/* Module information */
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_DESCRIPTION("s3c24xx I2S SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:s3c24xx-iis");
