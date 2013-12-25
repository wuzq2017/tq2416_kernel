/***********************************************************************************
* drivers/char/tq2416_beep.c
* 功能简要： 
*	该驱动注册一个字符设备“/dev/tq2416-beep”, 用于控制蜂鸣器。
* 提供的接口：
*       ioctol(struct inode *inode,struct file *file,unsigned int brightness);
*	用于调控蜂鸣器。改蜂鸣器的有效调整值为［1，100］
*	在驱动中是以固定的频率，仅通过调整有效时序宽度来控制蜂鸣器。
* 调用实例：
*	提供控制台，命令式的测试程序。
*	提供QT4界面化的测试程序
*
*************************************************************************************/
#include <linux/miscdevice.h>
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

#define DEVICE_NAME "beep"
#define BEEP_ON		1
#define BEEP_OFF	0

#define PRESCALER (255-1)

#define   DEBUG_BEEP  0  //for debug

/*
*函数用于在设定好定时器1的PWM模式后，
*开始生效定时器1
*/
static int tq2416_beep_start(void)
{
        unsigned long tcon;
        tcon = __raw_readl(S3C2410_TCON);//读取时钟控制寄存器
        tcon |= S3C2410_TCON_T0START;//开始位置1,让定时器开始工作
        tcon &= ~S3C2410_TCON_T0MANUALUPD;//之前已经设置好了TCNTB1 以及 TCMPB1，这里无须任何操作
        __raw_writel(tcon,S3C2410_TCON);//将设置写回寄存器
#if DEBUG_BEEP
	printk("call tq2416_beep_start....\n\n");
#endif
        return 0;
}

/*
*函数用于在更新定时器1的TCTB1以及TCMPB1的数值，
*通过更新
*/
static long tq2416_beep_ioctl(struct file *file,unsigned int CMD_ON_OFF, unsigned long Val)
{
	unsigned long tcon;//用于存放时钟控制寄存器的数值
	unsigned long tcnt;//用于存放TCNTB1的数值
	unsigned long tcmp;//用于存放TCMPB1的数值
	unsigned long tcfg1;//用于存放定时器配置寄存器1的数值
	unsigned long tcfg0;//用于存放定时器配置寄存器0的数值
	unsigned long pclk;
	struct clk *clk;
	tcnt = 0xffffffff;  /* 默认的TCTB1数值*/

	/* 读取TCON，TCFG0以及TCFG1寄存器的数值*/
	tcon = __raw_readl(S3C2410_TCON );
	tcfg1 =__raw_readl(S3C2410_TCFG1);
	tcfg0 =__raw_readl(S3C2410_TCFG0);

	/*定时器输入频率 = PCLK / ( {预分频数值 + 1} ) / {分割数值}
	*{预分频数值} = 1~255，由TCFG0配置寄存器来配置
	*{分割数值} = 1, 2, 4, 8, 16, TCLK，由TCFG1配置寄存器来配置
	*/
	clk = clk_get(NULL, "timers");
	if (IS_ERR(clk))
		panic("failed to get clock for pwm timer");
	clk_enable(clk); //从平台时钟队列中获取clk
	pclk = clk_get_rate(clk);
	//如果蜂鸣器要关闭
	if(CMD_ON_OFF<=BEEP_OFF)
	{
		//将GPF15引脚设置为输入
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(0));
		//将定时器控制寄存器中的TIMER1开始位设置为0
		tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T0START;//starting 
		tcon &= ~S3C2410_TCON_T0START;
		__raw_writel(tcon, S3C2410_TCON); //stop the timer1
	}
	else//更新TCTB1，TCMPB1的数值
	{
		//配置GPF15为输出
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(2));
		//清零TCFG1[4:7]
                tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		//设置分割值为2
                tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;//set [4:7]== 1/2
		//清零预分频位TCFG0[0:7]
                tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		//设置预分频数值，这里是254
                tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
		//清零TCON[8:10]
                tcon &= ~(7<<0); //set bit [0:2] to zero
		//设置定时器工作模式为自动加载模式(auto-reload)
                tcon |= S3C2410_TCON_T0RELOAD ;
		//将配置好的TCON，TCFG0，TCFG1的数值写回寄存器
		__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
		//准备更新TCMPB0数值，先置1 TCON[1]
		tcon |= S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon, S3C2410_TCON);	
		//TCNTB1==200，改数值决定了PWM的频率
		tcnt = 200;
		__raw_writel(tcnt, S3C2410_TCNTB(0));
		//改变TCMPB0的数值，改数值决定PWM的频宽
		tcmp = Val;
		__raw_writel(tcmp, S3C2410_TCMPB(0));
		//开始启动定时器
		tq2416_beep_start();

	}
	return 0;
}
#if DEBUG_BEEP
/*
*函数用于在更新定时器1的TCTB1以及TCMPB1的数值，
*通过更新
*/
static int tq2416_beep_debug(unsigned int CMD_ON_OFF, unsigned long Val)
{
	unsigned long tcon;//用于存放时钟控制寄存器的数值
	unsigned long tcnt;//用于存放TCNTB1的数值
	unsigned long tcmp;//用于存放TCMPB1的数值
	unsigned long tcfg1;//用于存放定时器配置寄存器1的数值
	unsigned long tcfg0;//用于存放定时器配置寄存器0的数值
	unsigned long pclk;
	struct clk *clk;
	tcnt = 0xffffffff;  /* 默认的TCTB1数值*/

        /* 读取TCON，TCFG0以及TCFG1寄存器的数值*/
        tcon = __raw_readl(S3C2410_TCON );
        tcfg1 =__raw_readl(S3C2410_TCFG1);
        tcfg0 =__raw_readl(S3C2410_TCFG0);

	/*定时器输入频率 = PCLK / ( {预分频数值 + 1} ) / {分割数值}
	*{预分频数值} = 1~255，由TCFG0配置寄存器来配置

	*{分割数值} = 1, 2, 4, 8, 16, TCLK，由TCFG1配置寄存器来配置
	*/
        clk = clk_get(NULL, "timers");
        if (IS_ERR(clk))
                panic("failed to get clock for pwm timer");
        clk_enable(clk); //从平台时钟队列中获取clk
        pclk = clk_get_rate(clk);
	
	//如果蜂鸣器要关闭
	if(CMD_ON_OFF<=BEEP_OFF)
	{
		//将GPF15引脚设置为输入
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(0));
		//将定时器控制寄存器中的TIMER1开始位设置为0
		tcon = __raw_readl(S3C2410_TCON);
        	tcon |= S3C2410_TCON_T0START;//starting 
		tcon &= ~S3C2410_TCON_T0START;
		__raw_writel(tcon, S3C2410_TCON); //stop the timer1
	}
	else//更新TCTB1，TCMPB1的数值
	{
		//配置GPF15为输出
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(2));
		//清零TCFG1[4:7]
                tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		//设置分割值为2
                tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;//set [4:7]== 1/2
		//清零预分频位TCFG0[0:7]
                tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		//设置预分频数值，这里是254
                tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
		//清零TCON[8:10]
                tcon &= ~(7<<8); //set bit [8:10] to zero
		//设置定时器工作模式为自动加载模式(auto-reload)
                tcon |= S3C2410_TCON_T0RELOAD ;
		//将配置好的TCON，TCFG0，TCFG1的数值写回寄存器
		__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
		//准备更新TCMPB1数值，先置1 TCON[9]
		tcon |= S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon, S3C2410_TCON);	
		//TCNTB1==200，改数值决定了PWM的频率
		tcnt = 200;
       		 __raw_writel(tcnt, S3C2410_TCNTB(0));
		//改变TCMPB1的数值，改数值决定PWM的频宽
        	tcmp = Val;
        	__raw_writel(tcmp, S3C2410_TCMPB(0));
		//开始启动定时器
		tq2416_beep_start();

	}
	return 0;
}
#endif
//when open beep device, this function will be called
static int tq2416_beep_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO " beep opened\n");
#if DEBUG_BEEP
	printk(" beep value is : 20.....\n\n");	
	tq2416_beep_debug(BEEP_ON, 20);
	mdelay(1000);
	printk(" beep value is : 40.....\n\n");
	tq2416_beep_debug(BEEP_ON, 40);
	mdelay(1000);
	printk(" beep value is : 80.....\n\n");
	tq2416_beep_debug(BEEP_ON, 80);
	mdelay(1000);
	printk(" beep value is : 100.....\n\n");
	tq2416_beep_debug(BEEP_ON, 100);
	mdelay(2000);
	printk(" beep off.....\n\n");
	tq2416_beep_debug(BEEP_OFF, 100);//off
	printk(KERN_INFO " beep opened done.....\n\n");
#endif
	return 0;

}
/*接口注册*/
static struct file_operations tq2416_beep_fops=
{
        .owner			=	THIS_MODULE,
        .unlocked_ioctl	=	tq2416_beep_ioctl,
		.open 			= 	tq2416_beep_open,
};
/*设备结构的设置*/
static struct miscdevice tq2416_beep_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &tq2416_beep_fops,
};
/*设备初始化函数*/
static int  __init tq2416_beep_init(void)
{
	int ret;
	ret = misc_register(&tq2416_beep_misc);
	if(ret)
	{
		printk (KERN_ERR "register miscdev \"%s\" failed!\n", DEVICE_NAME);
		return ret;	
	}
	printk(DEVICE_NAME " tq2416-beep initialized done...\n");
#if DEBUG_BEEP
	tq2416_beep_debug(1,100);
#endif
	return 0;
}

/*卸载函数*/
static void __exit tq2416_beep_exit(void)
{
	misc_deregister(&tq2416_beep_misc);
	printk("Goodbye tq2416-beep module !\n");
} 

module_init(tq2416_beep_init);
module_exit(tq2416_beep_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("www.embedsky.net");
MODULE_DESCRIPTION("Beep driver for EmbedSky SKY2416/TQ2416 Board");
