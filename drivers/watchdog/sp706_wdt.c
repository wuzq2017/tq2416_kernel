/*
 * drivers/char/watchdog/sp706-wdt.c
 *
 * Watchdog driver for ARM SP805 watchdog module
 *
 * Copyright (C) 2010 ST Microelectronics
 * Viresh Kumar<viresh.kumar@st.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2 or later. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/device.h>
#include <linux/resource.h>
#include <linux/amba/bus.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>
#include <mach/gpio-fns.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <asm/atomic.h>
#include <asm/unistd.h>

#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/kthread.h>

#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/kthread.h>
#include <linux/delay.h>

#include <plat/gpio-cfg.h>
#include <linux/gpio.h>

//#define DEBUG
#ifdef DEBUG
#define DBG(args...)  printk(KERN_INFO args)
#else
#define DBG(args...)
#endif

/* default timeout in 1.6 seconds */
#define DEVICE_NAME	"sp706-wdt"
#define WTD_KEEPALIVE   0

static DEFINE_SPINLOCK(wdt_lock);

//宁波中控
#define FEED_DOG_PIN 	S3C2410_GPF(5)
#define FEED_DOG_DIR_IN    S3C2410_GPG5_INP
#define FEED_DOG_DIR_OUT   S3C2410_GPG5_OUTP

#define GPIO_BASE (0x56000050)
#define GPIO_CON  (GPIO_BASE)
#define GPIO_DAT (GPIO_BASE + 4)
#define GPIO_PULL (GPIO_BASE + 8)


static void sp706_keepalive(void)
{
    int old_val;
	spin_lock(&wdt_lock);    
    old_val = s3c2410_gpio_getpin(FEED_DOG_PIN);
    if(old_val){
        s3c2410_gpio_setpin(FEED_DOG_PIN, 0);
    }else{
        s3c2410_gpio_setpin(FEED_DOG_PIN, 1);
    }
    spin_unlock(&wdt_lock);
    DBG("sp706: feed dog.\n");
}

static void sp706_wdt_start(void)
{
    spin_lock(&wdt_lock);
    s3c2410_gpio_cfgpin(FEED_DOG_PIN, FEED_DOG_DIR_OUT);
    s3c2410_gpio_setpin(FEED_DOG_PIN, 1);
    spin_unlock(&wdt_lock);
    DBG("Start sp706 watch dog.\n");
}

#if 0 //无法停止关门狗
static void sp706_wdt_stop(void)
{
    spin_lock(&wdt_lock);
    s3c2410_gpio_cfgpin(FEED_DOG_PIN, FEED_DOG_DIR_IN);
    s3c2410_gpio_pullup(FEED_DOG_PIN, S3C_GPIO_PULL_NONE);
    spin_unlock(&wdt_lock);
    DBG("Stop sp706 watch dog.\n");
    
}
#endif
static int sp706_wdt_stop(void)
{

    unsigned int *p_reg;
    unsigned int reg;
    spin_lock(&wdt_lock);
    /* 输入，禁止上拉 */
    s3c2410_gpio_cfgpin(FEED_DOG_PIN, FEED_DOG_DIR_IN);
    p_reg = (unsigned int *)ioremap(GPIO_PULL,0x100);
    if(p_reg == NULL){
        
        printk("sp706 ioremap failed.\n");
        return -ENXIO;
    }
    reg = readl(p_reg);
    reg &= ~(0x3 << 10);
    writel(reg, p_reg);
    DBG("reg = %x\n",reg);
    iounmap(p_reg);
    DBG("iounmap.\n");
    spin_unlock(&wdt_lock);
    printk("Stop sp706 watch dog.\n");
    return 0;
}


static int sp706_wdt_open(struct inode *inode, struct file *file)
{
    sp706_wdt_start();
    return 0;
}

static int sp706_wdt_close(struct inode *inode, struct file *file)
{
    sp706_wdt_stop();
    return 0;
}

/*
 *
 */
static long sp706_wdt_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
//	void __user *argp = (void __user *)arg;
//	int __user *p = argp;
    DBG("ioctl cmd=%d \n", cmd);
    switch(cmd){
    case WTD_KEEPALIVE:
        sp706_keepalive();
        break;
    default:

        return -ENOTTY;
    }
    return 0;
}

static const struct file_operations sp706_wdt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= sp706_wdt_ioctl,
	.open		= sp706_wdt_open,
    .release      = sp706_wdt_close,
};


static struct miscdevice sp706_wdt_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &sp706_wdt_fops,
};


static int __init sp706_wdt_init(void)
{
    int ret;
    ret = gpio_request(FEED_DOG_PIN, "sp706");
    if(ret < 0){
        printk("Fail to request gpio for sp706.\n");
		return ret;
    }

    ret = misc_register(&sp706_wdt_misc);
	if (ret < 0) {
		printk("Failed to register sp706 device\n");
        goto ERR1;
	}
    
    sp706_wdt_stop();
//    sp706_wdt_start();
    printk("sp706 Watchdog driver initialised ok.\n");
    return 0;
ERR1:
    gpio_free(FEED_DOG_PIN);

    return ret;
}


static void __exit sp706_wdt_exit(void)
{
    //以防万一,再次关闭看门狗
    sp706_wdt_stop();
    gpio_free(FEED_DOG_PIN);
    misc_deregister(&sp706_wdt_misc);
	printk("sp706 module exit!\n");
}

module_init(sp706_wdt_init);
module_exit(sp706_wdt_exit);

MODULE_AUTHOR("Enzo Fang <feixiangniao26@gmail.com>");
MODULE_DESCRIPTION("SP706 Watchdog Driver");
MODULE_LICENSE("GPL");
