/*************************************************************************************************
* drivers/char/tq2416_ADC.c
* ���ܼ�Ҫ�� 
*	������ע��һ���ַ��豸��/dev/adc��, ����ʵ��ADC��
* �ṩ�Ľӿڣ�
*       ioctol(struct inode *inode,struct file *file,unsigned long arg);
*	����ѡ��ͨ������������λ�������������������ĸ�ֵ�� cmd =1 ,������� ͨ���� cmd=2,������� ����λ��
*	����ֻ��ѡ�� 0==>AN0,1==>AN1,2==>AN2,3==>AN3 ͨ����
*	����ͨ��Ԥ����LCD������λֻ��ѡ�� 12 ���� 10
*	���õ�ͨ��ֵ��������λ��ֵ���ڸ÷�Χ�ڣ��Ᵽ����һ�ε���ֵ�������κθı䡣
****************************************************************************************************/

#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <mach/map.h>


#define DEVICE_NAME				"adc"
#define PORT_SELECTED			1
#define BIT_SELECTED			0
#define ADC_ENV(x)				(S3C2410_ADCCON_PRSCVL(49))
static DEFINE_MUTEX(adc_mutex);
struct tq2416adc {
	struct s3c_adc_client *client;
	struct clk	*adc_clock;
	void __iomem 	*base_addr;
	int  adc_port;
	int  adc_data;
};

static struct tq2416adc _adc;
/*
static unsigned long data_for_ADCCON;
static unsigned long data_for_ADCTSC;

static void s3c_adc_save_SFR_on_ADC(void) {
	
	data_for_ADCCON = readl(_adc.base_addr+S3C2410_ADCCON);
	data_for_ADCTSC = readl(_adc.base_addr+S3C2410_ADCTSC);
}

static void s3c_adc_restore_SFR_on_ADC(void) {
	
	writel(data_for_ADCCON, _adc.base_addr+S3C2410_ADCCON);
	writel(data_for_ADCTSC, _adc.base_addr+S3C2410_ADCTSC);
}*/



static void tq2416_adc_select(struct s3c_adc_client *client,
				unsigned select)
{
}
/**
 * tq2416_adc_conversion - ADC conversion callback
 * @client: The client that was registered with the ADC core.
 * @data0: The reading from ADCDAT0.
 * @data1: The reading from ADCDAT1.
 * @left: The number of samples left.
 *
 * Called when a conversion has finished.
 */
static void tq2416_adc_conversion(struct s3c_adc_client *client,
				  unsigned data0, unsigned data1,
				  unsigned *left)
{
#ifdef CONFIG_TQ2416_DEBUG_ADC
	printk("%s: %d,%d\n", __func__, data0&0xfff, data1);
#endif
}


	

/*Open���������豸�Ľӿ�*/
static int tq2416_adc_open(struct inode *inode, struct file *file)
{
#ifdef CONFIG_TQ2416_DEBUG_ADC
	printk(KERN_INFO "tq2416_adc_open() entered\n");	
#endif
	
	return 0;
}


/*�ر��豸�Ľӿ�*/
static int tq2416_adc_close(struct inode *inode, struct file *file)
{
	return 0;
}


/*
*�����ȡ��ѯ�ķ�ʽ����ͣ�ļ��ADCCON[15]��ת������λ���ж��Ƿ����һ��ת��
*��������ת����ͨ���ж϶�ȡ������λ����ȡ���ݣ��������ݿ������û��ռ�����
*/
static ssize_t tq2416_adc_read(struct file *file, char __user * buffer,size_t size, loff_t * pos)
{
	mutex_lock(&adc_mutex);
	_adc.adc_data=s3c_adc_read(_adc.client, _adc.adc_port);//
	mutex_unlock(&adc_mutex);
#ifdef CONFIG_TQ2416_DEBUG_ADC
	printk("_adc.date==%d\n",_adc.adc_data);
#endif
	/*����ȡ����ADת�����ֵ�������ϲ�Ӧ�ó���*/
	if(copy_to_user(buffer, (char *)&_adc.adc_data, sizeof(_adc.adc_data)))
	{
		printk("copy data to user space failed !\n");
		return -EFAULT;		
	}

	return sizeof(_adc.adc_data);
}

/*
*ioctl �ӿ������޸�ͨ������ת��������λ
*1 ��ʾҪ�޸�ͨ��
*2 ��ʾҪ�޸�ת��������λ
*/
static long tq2416_adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)//struct inode *inode, 
{
	unsigned int temp= (unsigned int)arg;
//	printk(KERN_INFO " s3c_adc_ioctl(cmd:: %d) bit: %d \n", cmd, temp);

	switch (cmd)
	{
	case PORT_SELECTED://�ı�ͨ������
#ifdef CONFIG_TQ2416_DEBUG_ADC
		printk("set channel port Number\n");
#endif
		if (temp >= 4)
		{
			printk(" %d is already reserved for TouchScreen\n", _adc.adc_port);
		}
		else
		{
#ifdef CONFIG_TQ2416_DEBUG_ADC
			printk(" now switch channel %d \n", temp);
#endif
			_adc.adc_port = temp;
		}
		return 0;
	case BIT_SELECTED://�ı�����λ����
#ifdef CONFIG_TQ2416_DEBUG_ADC
		printk("set channel bit Number\n");
#endif
		if(temp ==12 || temp ==10)
		{
			_adc.client->data_bit=temp;
#ifdef CONFIG_TQ2416_DEBUG_ADC
			printk("the data bits is %d \n",temp);
#endif
		}
		else
		{
#ifdef CONFIG_TQ2416_DEBUG_ADC
			printk("nice guy,data bits should be 12 or 10 !\n");
			printk("%d-bit \n",temp);
#endif
		}
		return 0;
	default:
		printk("unknowed cmd :%d!\n", cmd);
		return -ENOIOCTLCMD;
	}
}

/*�ӿں�������*/
static struct file_operations s3c_adc_fops = 
{
	.owner			= THIS_MODULE,
	.read			= tq2416_adc_read,
	.open			= tq2416_adc_open,
	.release 		= tq2416_adc_close,
	.unlocked_ioctl	= tq2416_adc_ioctl,
};

/*�豸�ṹ����*/
static struct miscdevice s3c_adc_miscdev = 
{
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= DEVICE_NAME,
	.fops		= &s3c_adc_fops,
};

static char banner[] __initdata = KERN_INFO "TQ2416 ADC driver\n";
/*
*�豸��ʼ��
*�����������ַ�����룻ƽ̨ʱ�ӵĻ�ȡ���Լ��豸�ṹ��ע��
*����ɹ�:����0
�����ʧ��:ע�����������ռ䣻ע��ʱ�ӣ�
*/
int __init tq2416_adc_init(void)
{
	int ret=0;	

	printk(banner);
	_adc.base_addr = ioremap(S3C24XX_PA_ADC, S3C24XX_SZ_ADC);//SZ_4K

	if(_adc.base_addr ==  NULL){
		ret = -ENOENT;
		goto err_map;
	}
	_adc.adc_clock = clk_get(NULL, "adc");

	if(IS_ERR(_adc.adc_clock)){
		printk("failed to fine ADC clock source: %s \n",KERN_ERR);
		goto err_clk;
	}

	clk_enable(_adc.adc_clock);	

	ret = misc_register(&s3c_adc_miscdev);

	if (ret) {
		printk(DEVICE_NAME "can't register major number\n");
		goto err_clk;
	}

	_adc.client = kzalloc(sizeof(struct s3c_adc_client), GFP_KERNEL);

	if (!_adc.client) {
		printk("no memory for adc client\n");
		goto err_clk;
	}

	_adc.client->is_ts = 0;
	_adc.client->select_cb = tq2416_adc_select;
	_adc.client->convert_cb = tq2416_adc_conversion;
	_adc.adc_port=2;
	_adc.client->data_bit=12;

	printk(KERN_INFO "TQ2416 ADC driver successfully probed.\n");

	return 0;

err_clk:
	clk_disable(_adc.adc_clock);
	clk_put(_adc.adc_clock);

err_map:
	iounmap(_adc.base_addr);

	return ret;
}
/*
*ע��ģ��ӿں���
*ADCʱ�ӵ�ע�����豸�ṹ��ע��
*
*/
static void __exit tq2416_adc_exit(void)
{
	iounmap(_adc.base_addr); /*�ͷ������ַӳ��ռ�*/

	if (_adc.adc_clock)   /*���κ�����ʱ��*/
	{
		clk_disable(_adc.adc_clock);    
		clk_put(_adc.adc_clock);
		_adc.adc_clock = NULL;
	}
	misc_deregister(&s3c_adc_miscdev);//*ע��mi
}

module_init(tq2416_adc_init);
module_exit(tq2416_adc_exit);

MODULE_AUTHOR("www.embedsky.net");
MODULE_DESCRIPTION("TQ2416 ADC driver");
MODULE_LICENSE("GPL");
