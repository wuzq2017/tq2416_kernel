/*************************************************************************************************
* drivers/char/tq2416_ADC.c
* 功能简要： 
*	该驱动注册一个字符设备“/dev/adc”, 用于实验ADC。
* 提供的接口：
*       ioctol(struct inode *inode,struct file *file,unsigned long arg);
*	用于选择通道和设置数据位，由命令来决定更新哪个值。 cmd =1 ,代表更新 通道。 cmd=2,代表更新 数据位。
*	这里只能选择 0==>AN0,1==>AN1,2==>AN2,3==>AN3 通道，
*	其他通道预留给LCD；数据位只能选择 12 或者 10
*	设置的通道值或者数据位的值不在该范围内，这保持上一次的数值，不作任何改变。
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


	

/*Open函数，打开设备的接口*/
static int tq2416_adc_open(struct inode *inode, struct file *file)
{
#ifdef CONFIG_TQ2416_DEBUG_ADC
	printk(KERN_INFO "tq2416_adc_open() entered\n");	
#endif
	
	return 0;
}


/*关闭设备的接口*/
static int tq2416_adc_close(struct inode *inode, struct file *file)
{
	return 0;
}


/*
*这里采取轮询的方式，不停的检测ADCCON[15]的转换换成位来判断是否完成一次转换
*如果完成了转换，通过判断读取的数据位来读取数据，并件数据拷贝到用户空间数组
*/
static ssize_t tq2416_adc_read(struct file *file, char __user * buffer,size_t size, loff_t * pos)
{
	mutex_lock(&adc_mutex);
	_adc.adc_data=s3c_adc_read(_adc.client, _adc.adc_port);//
	mutex_unlock(&adc_mutex);
#ifdef CONFIG_TQ2416_DEBUG_ADC
	printk("_adc.date==%d\n",_adc.adc_data);
#endif
	/*将读取到的AD转换后的值发往到上层应用程序*/
	if(copy_to_user(buffer, (char *)&_adc.adc_data, sizeof(_adc.adc_data)))
	{
		printk("copy data to user space failed !\n");
		return -EFAULT;		
	}

	return sizeof(_adc.adc_data);
}

/*
*ioctl 接口用于修改通道或者转换的数据位
*1 表示要修改通道
*2 表示要修改转换的数据位
*/
static long tq2416_adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)//struct inode *inode, 
{
	unsigned int temp= (unsigned int)arg;
//	printk(KERN_INFO " s3c_adc_ioctl(cmd:: %d) bit: %d \n", cmd, temp);

	switch (cmd)
	{
	case PORT_SELECTED://改变通道命令
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
	case BIT_SELECTED://改变数据位命令
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

/*接口函数设置*/
static struct file_operations s3c_adc_fops = 
{
	.owner			= THIS_MODULE,
	.read			= tq2416_adc_read,
	.open			= tq2416_adc_open,
	.release 		= tq2416_adc_close,
	.unlocked_ioctl	= tq2416_adc_ioctl,
};

/*设备结构设置*/
static struct miscdevice s3c_adc_miscdev = 
{
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= DEVICE_NAME,
	.fops		= &s3c_adc_fops,
};

static char banner[] __initdata = KERN_INFO "TQ2416 ADC driver\n";
/*
*设备初始化
*包括：虚拟地址的申请；平台时钟的获取；以及设备结构的注册
*如果成功:返回0
＊如果失败:注销申请的虚拟空间；注销时钟；
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
*注销模块接口函数
*ADC时钟的注销，设备结构的注销
*
*/
static void __exit tq2416_adc_exit(void)
{
	iounmap(_adc.base_addr); /*释放虚拟地址映射空间*/

	if (_adc.adc_clock)   /*屏蔽和销毁时钟*/
	{
		clk_disable(_adc.adc_clock);    
		clk_put(_adc.adc_clock);
		_adc.adc_clock = NULL;
	}
	misc_deregister(&s3c_adc_miscdev);//*注销mi
}

module_init(tq2416_adc_init);
module_exit(tq2416_adc_exit);

MODULE_AUTHOR("www.embedsky.net");
MODULE_DESCRIPTION("TQ2416 ADC driver");
MODULE_LICENSE("GPL");
