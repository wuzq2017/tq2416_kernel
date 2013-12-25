/*************************************************************************************************************************
* drivers/char/tq2416_backlight.c
* ���ܼ�Ҫ�� 
*	������ע��һ���ַ��豸��/dev/Backlight��, ���ڿ���LCD����̶ȡ�
* �ṩ�Ľӿڣ�
*       ioctol(struct inode *inode,struct file *file,unsigned int brightness);
*	���ڵ���LCD�������ȡ�ϵͳ�н������ʽ����
*������飺
*	tq2416_backlight_start(void)//������timer0����λS3C_TCON_T0START��ʹ�俪ʼ����
*	tq2416_backlight_off(void) //�ڱ���Ϊ0ʱ������GPF14 Ϊ���룬����S3C_TCON_T0START��ֹͣtimer0
*	tq2416_backlight_setTimer0 //���ڸ�������timer0	
*	tq2416_backlight_ioctl(struct inode *inode,struct file *file,unsigned int brightness,unsigned long Val)//�ⲿ�ӿ�
*	tq2416_backlight_init(void) //ģ���ʼ������
*	tq2416_backlight_exit(void) //ģ�鱻ж��ʱ���õĺ���
* ����ʵ����
*	backlight val backlight �����val����������ֵ��ȡֵ[0,100]
*	backlight 0   ��ʾ�رձ���
*	backlight 100 ��ʾ100%��������
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
*�����������趨�ö�ʱ��1��PWMģʽ��
*��ʼ��Ч��ʱ��0
*/
static int tq2416_backlight_start(void)
{
        unsigned long tcon;
        tcon = __raw_readl(S3C2410_TCON);//��ȡʱ�ӿ��ƼĴ���
        tcon |= S3C2410_TCON_T0START;//��ʼλ��1,�ö�ʱ����ʼ����
        tcon &= ~S3C2410_TCON_T0MANUALUPD;//֮ǰ�Ѿ����ú���TCNTB1 �Լ� TCMPB1�����������κβ���
        __raw_writel(tcon,S3C2410_TCON);//������д�ؼĴ���
#if DEBUG_BACKLIGHT
	printk("tq2416_backlight_start....\n\n");
#endif
	return 0;
}

/*
* �رձ����
*/
static int tq2416_backlight_off(void)
{
	unsigned long tcon;
	//Set it as input so that the backligt turned off
	//��GPF14����Ϊ����
	s3c_gpio_cfgpin(S3C2410_GPB(0),(0x00<<28));
	tcon = __raw_readl(S3C2410_TCON);
        tcon |= S3C2410_TCON_T0START;//starting
	tcon &= ~S3C2410_TCON_T0START;//�����ʱ��0��ʼλ��ֹͣ��ʱ��0
	tcon &= ~S3C2410_TCON_T0MANUALUPD;//ֹͣ���¹���
	__raw_writel(tcon, S3C2410_TCON); //�����ý��д��TCON��������Ч
	return 0;
}

/*���ö�ʱ��0���޸ı�������*/
static int tq2416_backlight_setTimer0(unsigned int brightness)
{
	unsigned long tcon;//���ڴ��ʱ�ӿ��ƼĴ�������ֵ
        unsigned long tcnt;//���ڴ��TCNTB0����ֵ
        unsigned long tcmp;//���ڴ��TCMPB0����ֵ
        unsigned long tcfg1;//���ڴ�Ŷ�ʱ�����üĴ���1����ֵ
        unsigned long tcfg0;//���ڴ�Ŷ�ʱ�����üĴ���0����ֵ  

	tcnt = 101;  /* TCNTB0�ĳ�ʼ��ֵ */

        /* ��ȡ��ʱ����ǰ�ļĴ�����������ֵ���Ա��ʼ�����޸�	*/
        tcon = __raw_readl(S3C2410_TCON);
        tcfg1 =__raw_readl(S3C2410_TCFG1);
        tcfg0 =__raw_readl(S3C2410_TCFG0);
	/*
	��ʱ��ʼ������Ƶ�� = PCLK / ( {��Ƶ + 1} ) / {�ָ�ֵ}
	{��Ƶ��ֵ} = 1~255
	{�ָ���ֵ} = 1, 2, 4, 8, 16, TCLK*/
	/*����GPF14Ϊ���*/
	s3c_gpio_cfgpin(S3C2410_GPC(0),S3C_GPIO_SFN(2));
	//���÷ָ���ֵ����2
	tcfg1 &= ~S3C2410_TCFG1_MUX0_MASK;
	tcfg1 |= S3C2410_TCFG1_MUX0_DIV2;
	//���÷�ƵΪ254
	tcfg0 &= ~S3C2410_TCFG_PRESCALER0_MASK;
	tcfg0 |= (PRESCALER) << S3C2410_TCFG_PRESCALER0_SHIFT;
	//���ö�ʱ��0Ϊ�Զ�����ģʽ
	tcon &= ~(7<<0);
	tcon |= S3C2410_TCON_T0RELOAD;
	//������������ֵд�붨ʱ��0�Ķ�Ӧ�Ĵ����У�ʹ֮��Ч
	__raw_writel(tcfg1, S3C2410_TCFG1);
        __raw_writel(tcfg0, S3C2410_TCFG0);
        __raw_writel(tcon, S3C2410_TCON);
		
	//ʹ�ܶ�ʱ��0���ֶ�����λ����Ϊ����Ҫ��������TCNTB0�Լ�TCMPB0
	tcon |= S3C2410_TCON_T0MANUALUPD;//timer0
	__raw_writel(tcon,S3C2410_TCON);	
	
	tcnt = 101;//����TCNTB0==101
	 __raw_writel(tcnt,S3C2410_TCNTB(0));

        tcmp = brightness;//����TCMPB0����ֵ�����û�����ı����������ֵ����
	__raw_writel(tcmp, S3C2410_TCMPB(0));
	return 0;
}
#endif
/*
*���ڿ��Ʊ��������
*������ȡ�����û����ݵ� brightness ��ֵ�� 0 Ϊ�رգ�100Ϊ 100������
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
			//�رձ���
			s3c_gpio_cfgpin(S3C2410_GPC(0),0x1<<0);
			s3c2410_gpio_setpin(S3C2410_GPC(0), 0);
			return 0;

		case 1:
			//�򿪱���
			s3c_gpio_cfgpin(S3C2410_GPC(0),0x1<<0);	
			s3c2410_gpio_setpin(S3C2410_GPC(0), 1);
			return 0;

		default:
			return -EINVAL;
	}
	return 0;
}

/*
*�û��ӿ�����
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
/*�豸��ʼ�����豸ע�᣻��ʼ����Ϊ100*/
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

/*ע��ģ��*/
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
