/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

#include <mach/regs-lcd.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>

#include <plat/s3c2410.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>

#include <plat/common-EmbedSky.h>

#include <mach/gpio-fns.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

//#define DEBUG
#undef DEBUG
#ifdef DEBUG
#define dbg(args...)  printk(args)
#else
#define dbg(args...)
#endif

static struct platform_device *p_bl_device=NULL;
static int tq2440_bl_init(struct device *dev);

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
    dbg("-------------------->%s\n", __func__);

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify){
		brightness = pb->notify(pb->dev, brightness);
        dbg("brightness=%d\n", brightness);
    }


	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
	} else {
        dbg("brightness=%d, pb->period=%d, pb->lth_brightness =%d, max=%d\n",\
            brightness, pb->period, pb->lth_brightness, max);
		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);
        dbg("------------------------------------------------------------\n");
        dbg("brightness=%d, pb->period=%d, pb->lth_brightness =%d, max=%d\n",\
            brightness, pb->period, pb->lth_brightness, max);
        
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;
    dbg("-------------------->%s\n", __func__);

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}
    dbg("-------------------->%s, P0\n", __func__);

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0){
            dbg("data->init(&pdev->dev) failed.\n");
			return ret;
        }
	}
    dbg("-------------------->%s, P1\n", __func__);
	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
    dbg("-------------------->%s, P2\n", __func__);
	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
    dbg("data->lth_brightness =%d, data->pwm_period_ns=%d, data->max_brightness=%d\n",\
        data->lth_brightness, data->pwm_period_ns, data->max_brightness);
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;
    dbg("-------------------->%s, P3\n", __func__);
	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "2416pwm-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int tq2440_bl_init(struct device *dev)
{
    dbg("Init pwm gpio init .\n");
    s3c_gpio_setpull(S3C2410_GPB(0),0);   // GPB0不上拉
    s3c_gpio_cfgpin(S3C2410_GPB(0),S3C2410_GPB0_TOUT0); // GPB0设置为TOUT0
    return 0;

}

static int __init pwm_backlight_init(void)
{
    int ret, id;
    struct platform_pwm_backlight_data *p_bl_data;
    dbg("-------------------->%s\n", __func__);
	ret = platform_driver_register(&pwm_backlight_driver);
    if(ret){
        goto exit_free_device;
    }
#if 0    
    ret = platform_device_register(&s3c_backlight_device);
    if(ret){
        goto exit_unregister_driver;
    }
#else
    /* 动态分配平台设备 id = 0*/
    id = 0;
    if ((p_bl_device = platform_device_alloc("2416pwm-bl", id)) == NULL) {
		ret = -ENOMEM;
        dbg("platform_device_alloc failed.\n");
		goto exit_unregister_driver;
	}
    /* timer 0 */
    p_bl_device->dev.parent = &s3c_device_timer[0].dev;
    /* platform 中会
     * 	kfree(pa->pdev.dev.platform_data);
     *	kfree(pa->pdev.resource);
     *	kfree(pa);
     */
    /* 分配平台驱动私有数据 */
    p_bl_data = kzalloc(sizeof(struct platform_pwm_backlight_data), GFP_KERNEL);
    /* pwm_id 对应timer0 */
    p_bl_data->pwm_id =0;
    p_bl_data->max_brightness     = 100;  //最大亮度
    p_bl_data->dft_brightness       = 100 ;      //当前亮度
    p_bl_data->pwm_period_ns    = 5000000;  //这就是前面说的T0，即输出时钟周期
    p_bl_data->init         = tq2440_bl_init;  //端口初始化
    p_bl_data->exit           = NULL;

    p_bl_device->dev.platform_data = p_bl_data;
    dbg("-------------------->platform_device_alloc OK.\n");

	if ((ret = platform_device_add(p_bl_device))){
        printk("platform device pwm-bl failed.\n");
        goto exit_free_device;
    }

#endif
    dbg("-------------------->2416pwm-bl init OK.\n");
  return 0;
exit_free_device:
    platform_device_put(p_bl_device);
exit_unregister_driver:
    platform_driver_unregister(&pwm_backlight_driver);
    return ret;
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
    dbg("-------------------->%s\n", __func__);
    
    platform_device_unregister(p_bl_device);
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

