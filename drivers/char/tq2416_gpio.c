
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



#define DEVICE_NAME				"tq2416-gpio"

/* 应用程序执行ioctl(fd, cmd, arg)时的第2个参数 */
#define IOCTL_GPIO_ON			1
#define IOCTL_GPIO_OFF			0


static long tq2416_gpio_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{

	switch(cmd)
	{
		case IOCTL_GPIO_ON:
			// 设置指定引脚的输出电平为1
			
				s3c2410_gpio_setpin(S3C2410_GPA(6), 1);
			    return 0;
		
		case IOCTL_GPIO_OFF:
			// 设置指定引脚的输出电平为0		
				s3c2410_gpio_setpin(S3C2410_GPA(6), 0);		
			    return 0;

		default:
			return -EINVAL;
	}
}
/*驱动接口设置*/
static struct file_operations dev_fops = {
	.owner			=	THIS_MODULE,
	.unlocked_ioctl =	tq2416_gpio_ioctl,
	
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

	s3c2410_gpio_setpin(S3C2410_GPA(6), 0);


	ret = misc_register(&misc);
	if(ret)
	{
		printk (KERN_ERR "register miscdev \"%s\" failed!\n", DEVICE_NAME);
		return ret;
		
	}
	printk (DEVICE_NAME" 2016.6.3 initialized\n");

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
