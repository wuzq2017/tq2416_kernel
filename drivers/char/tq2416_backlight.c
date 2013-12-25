/*************************************************************************************************************************
* drivers/char/tq2416_backlight.c
* 功能简要： 
*	该驱动注册一个字符设备“/dev/Backlight”, 用于控制LCD背光程度。
* 提供的接口：
*       ioctol(struct inode *inode,struct file *file,unsigned int brightness);
*	用于调控LCD背光亮度。系统中将以命令方式控制
*函数简介：
*	tq2416_backlight_start(void)//设置完timer0后，置位S3C_TCON_T0START，使其开始运算
*	tq2416_backlight_off(void) //在背光为0时，设置GPF14 为输入，清零S3C_TCON_T0START，停止timer0
*	tq2416_backlight_setTimer0 //用于更新设置timer0	
*	tq2416_backlight_ioctl(struct inode *inode,struct file *file,unsigned int brightness,unsigned long Val)//外部接口
*	tq2416_backlight_init(void) //模块初始化函数
*	tq2416_backlight_exit(void) //模块被卸载时调用的函数
* 调用实例：
*	backlight val backlight 是命令，val是亮度设置值，取值[0,100]
*	backlight 0   表示关闭背光
*	backlight 100 表示100%背光亮度
****************************************************************************************************************************/
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/clk.h>
#include <plat/regs-timer.h>
#include <mach/gpio-fns.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/miscdevice.h>

#define DEVICE_NAME "bkl"
#if 0
#define PRESCALER (255-1)

#define  DEBUG_BACKLIGHT  1   //for debug


/*
*函数用于在设定好定时器1的PWM模式后，
*开始生效定时器0
*/
static int tq2416_backlight_start(void)
{
        unsigned long tcon;
        tcon = __raw_readl(S3C2410_TCON);//读取时钟控制寄存器
        tcon |= S3C2410_TCON_T0START;//开始位置1,让定时器开始工作
        tcon &= ~S3C2410_TCON_T0MANUALUPD;//之前已经设置好了TCNTB1 以及 TCMPB1，这里无须任何操作
        __raw_writel(tcon,S3C2410_TCON);//将设置写回寄存器
#if DEBUG_BACKLIGHT
	printk("tq2416_backlight_start....\n\n");
#endif
	return 0;
}

/*
* 关闭背光灯
*/
static int tq2416_backlight_off(void)
{
	unsigned long tcon;
	//Set it as input so that the backligt turned off
	//将GPF14设置为输入
	s3c_gpio_cfgpin(S3C2410_GPB(0),(0x00<<28));
	tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T0START;//starting
	tcon &= ~S3C2410_TCON_T0START;//清除定时器0开始位来停止定时器0
	tcon &= ~S3C2410_TCON_T0MANUALUPD;//停止更新功能
	__raw_writel(tcon, S3C2410_TCON); //将设置结果写入TCON是设置生效
	return 0;
}

/*设置定时器0来修改背光亮度*/
static int tq2416_backlight_setTimer0(unsigned int brightness)
{
	unsigned long tcon;//用于存放时钟控制寄存器的数值
        unsigned long tcnt;//用于存放TCNTB0的数值
        unsigned long tcmp;//用于存放TCMPB0的数值
        unsigned long tcfg1;//用于存放定时器配置寄存器1的数值
        unsigned long tcfg0;//用于存放定时器配置寄存器0的数值  

	tcnt = 101;  /* TCNTB0的初始数值 */

        /* 读取定时器当前的寄存器的配置数值，以便初始化和修改	*/
        tcon = __raw_readl(S3C2410_TCON);
        tcfg1 =__raw_readl(S3C2410_TCFG1);
        tcfg0 =__raw_readl(S3C2410_TCFG0);
	/*
	定时器始终输入频率 = PCLK / ( {分频 + 1} ) / {分割值}
	{分频数值} = 1~255
	{分割数值} = 1, 2, 4, 8, 16, TCLK*/
	/*设置GPF14为输出*/
	s3c_gpio_cfgpin(S3C2410_GPC(0),S3C_GPIO_SFN(2));
	//设置分割数值＝＝2
	tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
	tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;
	//设置分频为254
	tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
	tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
	//设置定时器0为自动加载模式
	tcon &= ~(7<<0);
	tcon |= S3C2410_TCON_T0RELOAD;
	//将各个配置数值写入定时器0的对应寄存器中，使之生效
	__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
	//使能定时器0的手动更新位，因为我们要更新它的TCNTB0以及TCMPB0
	tcon |= S3C2410_TCON_T0MANUALUPD;//timer0
	__raw_writel(tcon,S3C2410_TCON);	
	
	tcnt = 101;//设置TCNTB0==101
	 __raw_writel(tcnt,S3C2410_TCNTB(0));

        tcmp = brightness;//设置TCMPB0的数值，有用户输入的背光灯亮度数值决定
	__raw_writel(tcmp, S3C2410_TCMPB(0));
	return 0;
}
#endif
/*
*用于控制背光灯亮度
*其亮度取决于用户传递的 brightness 数值： 0 为关闭；100为 100％亮度
*/
static long tq2416_backlight_ioctl(struct file *file,unsigned int cmd,unsigned long arg)
{
	unsigned int cmd_t =cmd;
	if(cmd > 1)
	{
		cmd_t=1;
	}
	switch(cmd_t)
	{
		case 0:
			//关闭背光
			s3c_gpio_cfgpin(S3C2410_GPC(0),0x1<<0);
			s3c2410_gpio_setpin(S3C2410_GPC(0), 0);
			return 0;

		case 1:
			//打开背光
			s3c_gpio_cfgpin(S3C2410_GPC(0),0x1<<0);	
			s3c2410_gpio_setpin(S3C2410_GPC(0), 1);
			return 0;

		default:
			return -EINVAL;
	}
	return 0;
}

/*
*用户接口设置
*
*/
static struct file_operations tq2416_backlight_fops=
{
        .owner			=	THIS_MODULE,
        .unlocked_ioctl	=	tq2416_backlight_ioctl,
};
static struct miscdevice tq2416_backlight_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &tq2416_backlight_fops,
};
/*设备初始化：设备注册；初始亮度为100*/
static int __init tq2416_backlight_init(void)
{
	int ret;
	s3c_gpio_cfgpin(S3C2410_GPC(0),0x1<<0);
	ret = misc_register(&tq2416_backlight_misc);
	if(ret)
	{
		printk (KERN_ERR "register miscdev \"%s\" failed!\n", DEVICE_NAME);
		return ret;
		
	}
	printk(DEVICE_NAME " tq2416-backlight initialized done...\n");
	return 0;
}

/*注销模块*/
static void __exit tq2416_backlight_exit(void)
{
	misc_deregister(&tq2416_backlight_misc);
	printk("Goodbye Backlight module !\n");
}

module_init(tq2416_backlight_init);
module_exit(tq2416_backlight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("www.embedsky.net");
MODULE_DESCRIPTION("Backlight driver for EmbedSky SKY2416/TQ2416 Board");
