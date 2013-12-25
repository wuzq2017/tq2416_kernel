/***********************************************************************************
* drivers/char/tq2416_leds.c
* 功能简要： 
*	该驱动注册一个字符设备“/dev/EmbedSky-leds”, 用于4个LED。
* 函数简介：
*	setGPMDAT_For_LED(int ON_OFF,int which_led), 用于点光亮LED，或者灭掉LED	
* 提供的外部接口：
*       ioctol(struct inode *inode,struct file *file,unsigned int brightness);
*	用于LED的亮，灭。
* 调用实例：
*	提供控制台，命令式的测试程序。
*	提供QT4界面化的测试程序
*
*************************************************************************************/
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/unistd.h>

#include <mach/gpio-fns.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>



#define DEBUG_ME_LEDS			0
#define DEVICE_NAME				"led"

/* 应用程序执行ioctl(fd, cmd, arg)时的第2个参数 */
#define IOCTL_GPIO_ON			1
#define IOCTL_GPIO_OFF			0

/* 用来指定LED所用的GPIO引脚 */

static unsigned long gpio_table [] =
{
	S3C2410_GPB(5),
	S3C2410_GPB(6),
	S3C2410_GPA(23),
//	S3C2410_GPA(24),
};

/* 用来指定GPIO引脚的功能：输出 */
static unsigned int gpio_cfg_table [] =
{
	S3C2410_GPB5_OUTP,
	S3C2410_GPB6_OUTP,
	S3C2410_GPA23_OUTP,
//	S3C2410_GPA24_OUTP,
};
#if DEBUG_ME_LEDS
tq2416_debug_leds(unsigned int cmd,unsigned long arg)
{
	s3c2410_gpio_setpin(gpio_table[arg], cmd);
}
#endif
static int tq2416_gpio_open(struct inode *inode, struct file *file)
{
#if DEBUG_ME_LEDS
	printk("leds on\n");
	tq2416_debug_leds(1,0);
	mdelay(1000);
	printk("leds off\n");
	tq2416_debug_leds(0,0);
	mdelay(1000);
	printk("leds on\n");
	tq2416_debug_leds(1,0);
	mdelay(1000);
	printk("leds off\n");
	tq2416_debug_leds(0,0);
	mdelay(1000);
	printk("leds on\n");
	tq2416_debug_leds(1,0);
	mdelay(1000);
	printk("leds off\n");
	tq2416_debug_leds(0,0);
#endif
	return 0;

}
static long tq2416_gpio_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	if (arg > sizeof(gpio_table)/sizeof(gpio_table[0]))
	{
		return -EINVAL;
	}
	switch(cmd)
	{
		case IOCTL_GPIO_ON:
			// 设置指定引脚的输出电平为1
			if(arg==3)
			{
				__raw_writel(__raw_readl(S3C2410_GPADAT)|(1<<24), S3C2410_GPADAT);
			}
			else
			{
				s3c2410_gpio_cfgpin(gpio_table[arg], gpio_cfg_table[arg]);
				s3c2410_gpio_setpin(gpio_table[arg], 1);
			}
			return 0;

		case IOCTL_GPIO_OFF:
			// 设置指定引脚的输出电平为0
			if(arg==3)
			{
				__raw_writel(__raw_readl(S3C2410_GPADAT)&(~(1<<24)), S3C2410_GPADAT);
			}
			else
			{
				s3c2410_gpio_setpin(gpio_table[arg], 0);
			}
			return 0;

		default:
			return -EINVAL;
	}
}
/*驱动接口设置*/
static struct file_operations dev_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =	tq2416_gpio_ioctl,
	.open 			=	tq2416_gpio_open,
};
/*设备结构的设置*/
static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};
/*初始化设备，配置对应的IO，以及注册设备*/
static int __init dev_init(void)
{
	int ret;

	int i;

	__raw_writel(__raw_readl(S3C2416_GPBSEL)&(~(1<<0)), S3C2416_GPBSEL);
	for (i = 0; i < sizeof(gpio_table)/sizeof(gpio_table[0]); i++)
	{
		s3c_gpio_cfgpin(gpio_table[i], gpio_cfg_table[i]);
		s3c2410_gpio_setpin(gpio_table[i], 1);
		if(i<2)
			s3c_gpio_setpull(gpio_table[i], S3C_GPIO_PULL_NONE);
	}
	__raw_writel(__raw_readl(S3C2410_GPACON)&(~(1<<24)), S3C2410_GPACON);
	__raw_writel(__raw_readl(S3C2410_GPADAT)|(1<<24), S3C2410_GPADAT);

	ret = misc_register(&misc);
	if(ret)
	{
		printk (KERN_ERR "register miscdev \"%s\" failed!\n", DEVICE_NAME);
		return ret;
		
	}
	printk (DEVICE_NAME" initialized\n");

	return ret;
}
/*注销设备*/
static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}

module_init(dev_init);
module_exit(dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("www.embedsky.net");
MODULE_DESCRIPTION("LED driver for EmbedSky SKY2416/TQ2416 Board");
