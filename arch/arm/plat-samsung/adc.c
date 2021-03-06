/* arch/arm/plat-samsung/adc.c
 *
 * Copyright (c) 2008 Simtec Electronics
 *	http://armlinux.simtec.co.uk/
 *	Ben Dooks <ben@simtec.co.uk>, <ben-linux@fluff.org>
 *
 * Samsung ADC device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>

#include <plat/regs-adc.h>
#include <plat/adc.h>

/* This driver is designed to control the usage of the ADC block between
 * the touchscreen and any other drivers that may need to use it, such as
 * the hwmon driver.
 *
 * Priority will be given to the touchscreen driver, but as this itself is
 * rate limited it should not starve other requests which are processed in
 * order that they are received.
 *
 * Each user registers to get a client block which uniquely identifies it
 * and stores information such as the necessary functions to callback when
 * action is required.
 */

enum s3c_cpu_type {
	TYPE_S3C24XX,
	TYPE_S3C64XX
};

struct adc_device {
	struct platform_device	*pdev;
	struct platform_device	*owner;
	struct clk		*clk;
	struct s3c_adc_client	*cur;
	struct s3c_adc_client	*ts_pend;
	void __iomem		*regs;
	spinlock_t		 lock;

	unsigned int		 prescale;

	int			 irq;
	struct regulator	*vdd;
};

static struct adc_device *adc_dev;

static LIST_HEAD(adc_pending);	/* protected by adc_device.lock */

#define adc_dbg(_adc, msg...) dev_dbg(&(_adc)->pdev->dev, msg)

static inline void s3c_adc_convert(struct adc_device *adc, struct s3c_adc_client *client)
{
	unsigned con = readl(adc->regs + S3C2410_ADCCON);

#ifdef CONFIG_MACH_TQ6410
	if(client->data_bit==10){
		con &= ~S3C64XX_ADCCON_RESSEL;
	}
#elif defined (CONFIG_CPU_S3C2416)
	if(client->data_bit == 12){
		con |= (1<<3);//lhh modify
	}
	else
	{
		con &= ~(1<<3);		
	}

#endif
	con |= S3C2410_ADCCON_ENABLE_START;
	writel(con, adc->regs + S3C2410_ADCCON);
}

static inline void s3c_adc_select(struct adc_device *adc,
				  struct s3c_adc_client *client)
{
	unsigned con = readl(adc->regs + S3C2410_ADCCON);
//	enum s3c_cpu_type cpu = platform_get_device_id(adc->pdev)->driver_data;

	client->select_cb(client, 1);

	con &= ~S3C2410_ADCCON_MUXMASK;
	con &= ~S3C2410_ADCCON_STDBM;
	con &= ~S3C2410_ADCCON_STARTMASK;

	if (!client->is_ts){
	//	con |= S3C2410_ADCCON_SELMUX(client->channel);//lhh modify
			writel(client->channel, adc->regs + S3C2416_ADCMUX);
	}

	writel(con, adc->regs + S3C2410_ADCCON);
}

static void s3c_adc_dbgshow(struct adc_device *adc)
{
	adc_dbg(adc, "CON=%08x, TSC=%08x, DLY=%08x\n",
		readl(adc->regs + S3C2410_ADCCON),
		readl(adc->regs + S3C2410_ADCTSC),
		readl(adc->regs + S3C2410_ADCDLY));
}

static void s3c_adc_try(struct adc_device *adc,struct s3c_adc_client *client)
{
	struct s3c_adc_client *next = adc->ts_pend;

	if (!next && !list_empty(&adc_pending)) {
		next = list_first_entry(&adc_pending,
					struct s3c_adc_client, pend);
		list_del(&next->pend);
	} else
		adc->ts_pend = NULL;

	if (next) {
		adc_dbg(adc, "new client is %p\n", next);
		adc->cur = next;
		s3c_adc_select(adc, next);
		s3c_adc_convert(adc,client);
		s3c_adc_dbgshow(adc);
	}
}

int s3c_adc_start(struct s3c_adc_client *client,
		  unsigned int channel, unsigned int nr_samples)
{
	struct adc_device *adc = adc_dev;
	unsigned long flags;

	if (!adc) {
		printk(KERN_ERR "%s: failed to find adc\n", __func__);
		return -EINVAL;
	}

	if (client->is_ts && adc->ts_pend)
		return -EAGAIN;

	spin_lock_irqsave(&adc->lock, flags);

	client->channel = channel;
	client->nr_samples = nr_samples;

	if (client->is_ts)
		adc->ts_pend = client;
	else
		list_add_tail(&client->pend, &adc_pending);

	if (!adc->cur)
		s3c_adc_try(adc,client);

	spin_unlock_irqrestore(&adc->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(s3c_adc_start);

static void s3c_convert_done(struct s3c_adc_client *client,
			     unsigned v, unsigned u, unsigned *left)
{
	client->result = v;
	wake_up(client->wait);
}

int s3c_adc_read(struct s3c_adc_client *client, unsigned int ch)
{
	DECLARE_WAIT_QUEUE_HEAD_ONSTACK(wake);
	int ret;

	client->convert_cb = s3c_convert_done;
	client->wait = &wake;
	client->result = -1;

	ret = s3c_adc_start(client, ch, 1);
	if (ret < 0)
		goto err;

	ret = wait_event_timeout(wake, client->result >= 0, HZ / 2);
	if (client->result < 0) {
		ret = -ETIMEDOUT;
		goto err;
	}

	client->convert_cb = NULL;
	return client->result;

err:
	return ret;
}
EXPORT_SYMBOL_GPL(s3c_adc_read);

static void s3c_adc_default_select(struct s3c_adc_client *client,
				   unsigned select)
{
}

struct s3c_adc_client *s3c_adc_register(struct platform_device *pdev,
					void (*select)(struct s3c_adc_client *client,
						       unsigned int selected),
					void (*conv)(struct s3c_adc_client *client,
						     unsigned d0, unsigned d1,
						     unsigned *samples_left),
					unsigned int is_ts)
{
	struct s3c_adc_client *client;

	WARN_ON(!pdev);

	if (!select)
		select = s3c_adc_default_select;

	if (!pdev)
		return ERR_PTR(-EINVAL);

	client = kzalloc(sizeof(struct s3c_adc_client), GFP_KERNEL);
	if (!client) {
		dev_err(&pdev->dev, "no memory for adc client\n");
		return ERR_PTR(-ENOMEM);
	}

	client->pdev = pdev;
	client->is_ts = is_ts;
	client->select_cb = select;
	client->convert_cb = conv;

	return client;
}
EXPORT_SYMBOL_GPL(s3c_adc_register);

void s3c_adc_release(struct s3c_adc_client *client)
{
	unsigned long flags;

	spin_lock_irqsave(&adc_dev->lock, flags);

	/* We should really check that nothing is in progress. */
	if (adc_dev->cur == client)
		adc_dev->cur = NULL;
	if (adc_dev->ts_pend == client)
		adc_dev->ts_pend = NULL;
	else {
		struct list_head *p, *n;
		struct s3c_adc_client *tmp;

		list_for_each_safe(p, n, &adc_pending) {
			tmp = list_entry(p, struct s3c_adc_client, pend);
			if (tmp == client)
				list_del(&tmp->pend);
		}
	}

	if (adc_dev->cur == NULL)
		s3c_adc_try(adc_dev,client);

	spin_unlock_irqrestore(&adc_dev->lock, flags);
	kfree(client);
}
EXPORT_SYMBOL_GPL(s3c_adc_release);

static irqreturn_t s3c_adc_irq(int irq, void *pw)
{
	struct adc_device *adc = pw;
	struct s3c_adc_client *client = adc->cur;
	enum s3c_cpu_type cpu = platform_get_device_id(adc->pdev)->driver_data;
	unsigned data0, data1;

	if (!client) {
		dev_warn(&adc->pdev->dev, "%s: no adc pending\n", __func__);
		goto exit;
	}
#ifdef CONFIG_MACH_TQ6410
	if(client->data_bit==10){
		writel(readl(adc->regs + S3C2410_ADCCON)|S3C64XX_ADCCON_RESSEL,
			adc->regs + S3C2410_ADCCON);
	}
#endif
#ifndef CONFIG_MACH_TQ2416 //这里设置基本没用了，已经设置过了，已经转换了，在这里设置多此一举--lhh 
	if(client->data_bit==12){
		writel(readl(adc->regs + S3C2410_ADCCON)|(1<<3),
			adc->regs + S3C2410_ADCCON);
	}
	else
	{
		writel(readl(adc->regs + S3C2410_ADCCON)&(~(1<<3)),
			adc->regs + S3C2410_ADCCON);
	}
#endif
	data0 = readl(adc->regs + S3C2410_ADCDAT0);
	data1 = readl(adc->regs + S3C2410_ADCDAT1);
	adc_dbg(adc, "read %d: 0x%04x, 0x%04x\n", client->nr_samples, data0, data1);
#ifdef CONFIG_MACH_TQ2416 //lhh add
	if(client->data_bit==12){//加这断代码，就是为了勉强纠正触屏和ADC之间的分辨率冲突所产生的后果，
		if(((readl(adc->regs + S3C2410_ADCCON)) & (1<<3)) == 0)
		{//10bitmod
			//printk( "10_read %d: 0x%04x, 0x%04x\n", client->nr_samples, data0, data1);
			data0 = (data0 & (~0xfff)) | (data0&0xfff)*4;//*4095/1023;
		}
	}
	else
	{
		if(((readl(adc->regs + S3C2410_ADCCON)) & (1<<3)) != 0)
		{//12bitmod
			//printk( "12_read %d: 0x%04x, 0x%04x\n", client->nr_samples, data0, data1);
			
			data0 = (data0 & (~0xfff)) | (data0&0xfff)/4;//*1023/4095;
		}
	}
#endif

	client->nr_samples--;
	if (cpu == TYPE_S3C64XX) {
#ifdef CONFIG_MACH_TQ6410
		if(client->data_bit==10)
		{
		/* S3C64XX ADC resolution is 10-bit */
			data0 &= 0x3ff;
			data1 &= 0x3ff;

		}else{
		/* S3C64XX ADC resolution is 12-bit */
			data0 &= 0xfff;
			data1 &= 0xfff;
		}
#endif

	}else {
		if(client->data_bit==12)
		{
		/* S3C24XX ADC resolution is 12-bit */
			data0 &= 0xfff;
			data1 &= 0xfff;

		}else{
		/* S3C24XX ADC resolution is 10-bit */
			data0 &= 0x3ff;
			data1 &= 0x3ff;
		}

#ifdef CONFIG_SAMSUNG_ADC_DEBUG__
	printk("%s S3C2410_ADCCON=>0x%08x \n",__func__,readl(adc->regs+S3C2410_ADCCON));
#endif
	}

	if (client->convert_cb)
		(client->convert_cb)(client, data0, data1, &client->nr_samples);

	if (client->nr_samples > 0) {
		/* fire another conversion for this */

		client->select_cb(client, 1);
		s3c_adc_convert(adc,client);
	} else {
		spin_lock(&adc->lock);
		(client->select_cb)(client, 0);
		adc->cur = NULL;

		s3c_adc_try(adc,client);
		spin_unlock(&adc->lock);
	}

exit:
	if (cpu == TYPE_S3C64XX) {
		/* Clear ADC interrupt */
		writel(0, adc->regs + S3C64XX_ADCCLRINT);
	}
	return IRQ_HANDLED;
}

static int s3c_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adc_device *adc;
	struct resource *regs;
	int ret;
	unsigned tmp;

	adc = kzalloc(sizeof(struct adc_device), GFP_KERNEL);
	if (adc == NULL) {
		dev_err(dev, "failed to allocate adc_device\n");
		return -ENOMEM;
	}

	spin_lock_init(&adc->lock);

	adc->pdev = pdev;
	adc->prescale = S3C2410_ADCCON_PRSCVL(49);

	adc->vdd = regulator_get(dev, "vdd");
	if (IS_ERR(adc->vdd)) {
		dev_err(dev, "operating without regulator \"vdd\" .\n");
		ret = PTR_ERR(adc->vdd);
		goto err_alloc;
	}

	adc->irq = platform_get_irq(pdev, 1);
	if (adc->irq <= 0) {
		dev_err(dev, "failed to get adc irq\n");
		ret = -ENOENT;
		goto err_reg;
	}

	ret = request_irq(adc->irq, s3c_adc_irq, 0, dev_name(dev), adc);
	if (ret < 0) {
		dev_err(dev, "failed to attach adc irq\n");
		goto err_reg;
	}

	adc->clk = clk_get(dev, "adc");
	if (IS_ERR(adc->clk)) {
		dev_err(dev, "failed to get adc clock\n");
		ret = PTR_ERR(adc->clk);
		goto err_irq;
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(dev, "failed to find registers\n");
		ret = -ENXIO;
		goto err_clk;
	}

	adc->regs = ioremap(regs->start, resource_size(regs));
	if (!adc->regs) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_clk;
	}

	ret = regulator_enable(adc->vdd);
	if (ret)
		goto err_ioremap;

	clk_enable(adc->clk);

	tmp = adc->prescale | S3C2410_ADCCON_PRSCEN;
	if (platform_get_device_id(pdev)->driver_data == TYPE_S3C64XX) {
		/* Enable 12-bit ADC resolution */
		tmp |= S3C64XX_ADCCON_RESSEL;  //12-bit by default
	}else{
		tmp |= S3C2410_ADCCON_RESSEL_12bit;//12bit;//lhh ~modify
	}
	writel(tmp, adc->regs + S3C2410_ADCCON);

	dev_info(dev, "attached adc driver\n");

	platform_set_drvdata(pdev, adc);
	adc_dev = adc;

	return 0;

 err_ioremap:
	iounmap(adc->regs);
 err_clk:
	clk_put(adc->clk);

 err_irq:
	free_irq(adc->irq, adc);
 err_reg:
	regulator_put(adc->vdd);
 err_alloc:
	kfree(adc);
	return ret;
}

static int __devexit s3c_adc_remove(struct platform_device *pdev)
{
	struct adc_device *adc = platform_get_drvdata(pdev);

	iounmap(adc->regs);
	free_irq(adc->irq, adc);
	clk_disable(adc->clk);
	regulator_disable(adc->vdd);
	regulator_put(adc->vdd);
	clk_put(adc->clk);
	kfree(adc);

	return 0;
}

#ifdef CONFIG_PM
static int s3c_adc_suspend(struct device *dev)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct adc_device *adc = platform_get_drvdata(pdev);
	unsigned long flags;
	u32 con;

	spin_lock_irqsave(&adc->lock, flags);

	con = readl(adc->regs + S3C2410_ADCCON);
	con |= S3C2410_ADCCON_STDBM;
	writel(con, adc->regs + S3C2410_ADCCON);

	disable_irq(adc->irq);
	spin_unlock_irqrestore(&adc->lock, flags);
	clk_disable(adc->clk);
	regulator_disable(adc->vdd);

	return 0;
}

static int s3c_adc_resume(struct device *dev)
{
	struct platform_device *pdev = container_of(dev,
			struct platform_device, dev);
	struct adc_device *adc = platform_get_drvdata(pdev);
	int ret;
	unsigned long tmp;

	ret = regulator_enable(adc->vdd);
	if (ret)
		return ret;
	clk_enable(adc->clk);
	enable_irq(adc->irq);

	tmp = adc->prescale | S3C2410_ADCCON_PRSCEN;
	/* Enable 12-bit ADC resolution */
	if (platform_get_device_id(pdev)->driver_data != TYPE_S3C64XX)
		tmp |= S3C64XX_ADCCON_RESSEL;
	writel(tmp, adc->regs + S3C2410_ADCCON);

	return 0;
}

#else
#define s3c_adc_suspend NULL
#define s3c_adc_resume NULL
#endif

static struct platform_device_id s3c_adc_driver_ids[] = {
	{
		.name           = "s3c24xx-adc",
		.driver_data    = TYPE_S3C24XX,
	}, {
		.name           = "s3c64xx-adc",
		.driver_data    = TYPE_S3C64XX,
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, s3c_adc_driver_ids);

static const struct dev_pm_ops adc_pm_ops = {
	.suspend	= s3c_adc_suspend,
	.resume		= s3c_adc_resume,
};

static struct platform_driver s3c_adc_driver = {
	.id_table	= s3c_adc_driver_ids,
	.driver		= {
		.name	= "s3c-adc",
		.owner	= THIS_MODULE,
		.pm	= &adc_pm_ops,
	},
	.probe		= s3c_adc_probe,
	.remove		= __devexit_p(s3c_adc_remove),
};

static int __init adc_init(void)
{
	int ret;

	ret = platform_driver_register(&s3c_adc_driver);
	if (ret)
		printk(KERN_ERR "%s: failed to add adc driver\n", __func__);

	return ret;
}

module_init(adc_init);
