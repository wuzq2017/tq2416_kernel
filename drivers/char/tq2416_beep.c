/***********************************************************************************
* drivers/char/tq2416_beep.c
* ���ܼ�Ҫ�� 
*	������ע��һ���ַ��豸��/dev/tq2416-beep��, ���ڿ��Ʒ�������
* �ṩ�Ľӿڣ�
*       ioctol(struct inode *inode,struct file *file,unsigned int brightness);
*	���ڵ��ط��������ķ���������Ч����ֵΪ��1��100��
*	�����������Թ̶���Ƶ�ʣ���ͨ��������Чʱ���������Ʒ�������
* ����ʵ����
*	�ṩ����̨������ʽ�Ĳ��Գ���
*	�ṩQT4���滯�Ĳ��Գ���
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
*�����������趨�ö�ʱ��1��PWMģʽ��
*��ʼ��Ч��ʱ��1
*/
static int tq2416_beep_start(void)
{
        unsigned long tcon;
        tcon = __raw_readl(S3C2410_TCON);//��ȡʱ�ӿ��ƼĴ���
        tcon |= S3C2410_TCON_T0START;//��ʼλ��1,�ö�ʱ����ʼ����
        tcon &= ~S3C2410_TCON_T0MANUALUPD;//֮ǰ�Ѿ����ú���TCNTB1 �Լ� TCMPB1�����������κβ���
        __raw_writel(tcon,S3C2410_TCON);//������д�ؼĴ���
#if DEBUG_BEEP
	printk("call tq2416_beep_start....\n\n");
#endif
        return 0;
}

/*
*���������ڸ��¶�ʱ��1��TCTB1�Լ�TCMPB1����ֵ��
*ͨ������
*/
static long tq2416_beep_ioctl(struct file *file,unsigned int CMD_ON_OFF, unsigned long Val)
{
	unsigned long tcon;//���ڴ��ʱ�ӿ��ƼĴ�������ֵ
	unsigned long tcnt;//���ڴ��TCNTB1����ֵ
	unsigned long tcmp;//���ڴ��TCMPB1����ֵ
	unsigned long tcfg1;//���ڴ�Ŷ�ʱ�����üĴ���1����ֵ
	unsigned long tcfg0;//���ڴ�Ŷ�ʱ�����üĴ���0����ֵ
	unsigned long pclk;
	struct clk *clk;
	tcnt = 0xffffffff;  /* Ĭ�ϵ�TCTB1��ֵ*/

	/* ��ȡTCON��TCFG0�Լ�TCFG1�Ĵ�������ֵ*/
	tcon = __raw_readl(S3C2410_TCON );
	tcfg1 =__raw_readl(S3C2410_TCFG1);
	tcfg0 =__raw_readl(S3C2410_TCFG0);

	/*��ʱ������Ƶ�� = PCLK / ( {Ԥ��Ƶ��ֵ + 1} ) / {�ָ���ֵ}
	*{Ԥ��Ƶ��ֵ} = 1~255����TCFG0���üĴ���������
	*{�ָ���ֵ} = 1, 2, 4, 8, 16, TCLK����TCFG1���üĴ���������
	*/
	clk = clk_get(NULL, "timers");
	if (IS_ERR(clk))
		panic("failed to get clock for pwm timer");
	clk_enable(clk); //��ƽ̨ʱ�Ӷ����л�ȡclk
	pclk = clk_get_rate(clk);
	//���������Ҫ�ر�
	if(CMD_ON_OFF<=BEEP_OFF)
	{
		//��GPF15��������Ϊ����
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(0));
		//����ʱ�����ƼĴ����е�TIMER1��ʼλ����Ϊ0
		tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T0START;//starting 
		tcon &= ~S3C2410_TCON_T0START;
		__raw_writel(tcon, S3C2410_TCON); //stop the timer1
	}
	else//����TCTB1��TCMPB1����ֵ
	{
		//����GPF15Ϊ���
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(2));
		//����TCFG1[4:7]
                tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		//���÷ָ�ֵΪ2
                tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;//set [4:7]== 1/2
		//����Ԥ��ƵλTCFG0[0:7]
                tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		//����Ԥ��Ƶ��ֵ��������254
                tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
		//����TCON[8:10]
                tcon &= ~(7<<0); //set bit [0:2] to zero
		//���ö�ʱ������ģʽΪ�Զ�����ģʽ(auto-reload)
                tcon |= S3C2410_TCON_T0RELOAD ;
		//�����úõ�TCON��TCFG0��TCFG1����ֵд�ؼĴ���
		__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
		//׼������TCMPB0��ֵ������1 TCON[1]
		tcon |= S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon, S3C2410_TCON);	
		//TCNTB1==200������ֵ������PWM��Ƶ��
		tcnt = 200;
		__raw_writel(tcnt, S3C2410_TCNTB(0));
		//�ı�TCMPB0����ֵ������ֵ����PWM��Ƶ��
		tcmp = Val;
		__raw_writel(tcmp, S3C2410_TCMPB(0));
		//��ʼ������ʱ��
		tq2416_beep_start();

	}
	return 0;
}
#if DEBUG_BEEP
/*
*���������ڸ��¶�ʱ��1��TCTB1�Լ�TCMPB1����ֵ��
*ͨ������
*/
static int tq2416_beep_debug(unsigned int CMD_ON_OFF, unsigned long Val)
{
	unsigned long tcon;//���ڴ��ʱ�ӿ��ƼĴ�������ֵ
	unsigned long tcnt;//���ڴ��TCNTB1����ֵ
	unsigned long tcmp;//���ڴ��TCMPB1����ֵ
	unsigned long tcfg1;//���ڴ�Ŷ�ʱ�����üĴ���1����ֵ
	unsigned long tcfg0;//���ڴ�Ŷ�ʱ�����üĴ���0����ֵ
	unsigned long pclk;
	struct clk *clk;
	tcnt = 0xffffffff;  /* Ĭ�ϵ�TCTB1��ֵ*/

        /* ��ȡTCON��TCFG0�Լ�TCFG1�Ĵ�������ֵ*/
        tcon = __raw_readl(S3C2410_TCON );
        tcfg1 =__raw_readl(S3C2410_TCFG1);
        tcfg0 =__raw_readl(S3C2410_TCFG0);

	/*��ʱ������Ƶ�� = PCLK / ( {Ԥ��Ƶ��ֵ + 1} ) / {�ָ���ֵ}
	*{Ԥ��Ƶ��ֵ} = 1~255����TCFG0���üĴ���������

	*{�ָ���ֵ} = 1, 2, 4, 8, 16, TCLK����TCFG1���üĴ���������
	*/
        clk = clk_get(NULL, "timers");
        if (IS_ERR(clk))
                panic("failed to get clock for pwm timer");
        clk_enable(clk); //��ƽ̨ʱ�Ӷ����л�ȡclk
        pclk = clk_get_rate(clk);
	
	//���������Ҫ�ر�
	if(CMD_ON_OFF<=BEEP_OFF)
	{
		//��GPF15��������Ϊ����
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(0));
		//����ʱ�����ƼĴ����е�TIMER1��ʼλ����Ϊ0
		tcon = __raw_readl(S3C2410_TCON);
        	tcon |= S3C2410_TCON_T0START;//starting 
		tcon &= ~S3C2410_TCON_T0START;
		__raw_writel(tcon, S3C2410_TCON); //stop the timer1
	}
	else//����TCTB1��TCMPB1����ֵ
	{
		//����GPF15Ϊ���
		s3c_gpio_cfgpin(S3C2410_GPB(0),S3C_GPIO_SFN(2));
		//����TCFG1[4:7]
                tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
		//���÷ָ�ֵΪ2
                tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;//set [4:7]== 1/2
		//����Ԥ��ƵλTCFG0[0:7]
                tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
		//����Ԥ��Ƶ��ֵ��������254
                tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
		//����TCON[8:10]
                tcon &= ~(7<<8); //set bit [8:10] to zero
		//���ö�ʱ������ģʽΪ�Զ�����ģʽ(auto-reload)
                tcon |= S3C2410_TCON_T0RELOAD ;
		//�����úõ�TCON��TCFG0��TCFG1����ֵд�ؼĴ���
		__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
		//׼������TCMPB1��ֵ������1 TCON[9]
		tcon |= S3C2410_TCON_T0MANUALUPD;
		__raw_writel(tcon, S3C2410_TCON);	
		//TCNTB1==200������ֵ������PWM��Ƶ��
		tcnt = 200;
       		 __raw_writel(tcnt, S3C2410_TCNTB(0));
		//�ı�TCMPB1����ֵ������ֵ����PWM��Ƶ��
        	tcmp = Val;
        	__raw_writel(tcmp, S3C2410_TCMPB(0));
		//��ʼ������ʱ��
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
/*�ӿ�ע��*/
static struct file_operations tq2416_beep_fops=
{
        .owner			=	THIS_MODULE,
        .unlocked_ioctl	=	tq2416_beep_ioctl,
		.open 			= 	tq2416_beep_open,
};
/*�豸�ṹ������*/
static struct miscdevice tq2416_beep_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &tq2416_beep_fops,
};
/*�豸��ʼ������*/
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

/*ж�غ���*/
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
