/* arch/arm/mach-s3c2416/mach-smdk2416.c
 *
 * Copyright (c) 2009 Yauhen Kharuzhy <jekhor@gmail.com>,
 *	as part of OpenInkpot project
 * Copyright (c) 2009 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>

#include <mach/idle.h>
#include <mach/leds-gpio.h>
#include <plat/iic.h>

#include <plat/s3c2416.h>
#include <plat/gpio-cfg.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/sdhci.h>
#include <plat/udc.h>

#include <plat/regs-fb-v4.h>
#include <plat/fb.h>

#ifdef CONFIG_DM9000
#include <linux/dm9000.h>
#include <mach/regs-mem.h>
#endif

#include <plat/common-smdk.h>

static struct map_desc smdk2416_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdk2416_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON | 0x50,
		.ufcon	     = UFCON,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
};

void smdk2416_hsudc_gpio_init(void)
{
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_UP);
	s3c_gpio_setpull(S3C2410_GPF(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(1));
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

void smdk2416_hsudc_gpio_uninit(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
	s3c_gpio_setpull(S3C2410_GPH(14), S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(S3C2410_GPH(14), S3C_GPIO_SFN(0));
}

struct s3c24xx_hsudc_platdata smdk2416_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = smdk2416_hsudc_gpio_init,
	.gpio_uninit = smdk2416_hsudc_gpio_uninit,
};

#ifdef CONFIG_DM9000
#define MACH_TQ2416_DM9K_BASE		(S3C2410_CS4 + 0x300)
/* DM9000AEP 10/100 ethernet controller */

static struct resource tq2416_dm9k_resource[] = {
	[0] = {
		.start = MACH_TQ2416_DM9K_BASE,
		.end   = MACH_TQ2416_DM9K_BASE + 3,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = MACH_TQ2416_DM9K_BASE + 8,
		.end   = MACH_TQ2416_DM9K_BASE + 8 + 3,
		.flags = IORESOURCE_MEM
	},
	[2] = {
		.start = IRQ_EINT4,
		.end   = IRQ_EINT4,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

/*
 * The DM9000 has no eeprom, and it's MAC address is set by
 * the bootloader before starting the kernel.
 */
static struct dm9000_plat_data tq2416_dm9k_pdata = {
	.flags		= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device tq2416_device_eth = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(tq2416_dm9k_resource),
	.resource	= tq2416_dm9k_resource,
	.dev		= {
		.platform_data	= &tq2416_dm9k_pdata,
	},
};

#endif

//LCD
#if defined(CONFIG_FB_S3C_LCD480272)
#define S3CFB_HFP			2	/* front porch */
#define S3CFB_HSW			41	/* hsync width */
#define S3CFB_HBP			2	/* back porch */

#define S3CFB_VFP			2	/* front porch */
#define S3CFB_VSW			10	/* vsync width */
#define S3CFB_VBP			2	/* back porch */

#define S3CFB_HRES			480	/* horizon pixel  x resolition */
#define S3CFB_VRES			272	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	480	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	544	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		480	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		272	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	80	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	7//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"T43 480*272"

#elif defined(CONFIG_FB_S3C_LCD640480)
#define S3CFB_HFP			16	/* front porch */
#define S3CFB_HSW			10	/* hsync width */
#define S3CFB_HBP			144	/* back porch */

#define S3CFB_VFP			32	/* front porch */
#define S3CFB_VSW			2	/* vsync width */
#define S3CFB_VBP			13	/* back porch */

#define S3CFB_HRES			640	/* horizon pixel  x resolition */
#define S3CFB_VRES			480	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	640	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	960	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		640	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		480	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	1//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"W57 640*480"

#elif defined(CONFIG_FB_S3C_LCD800480)
#define S3CFB_HFP			200	/* front porch */
#define S3CFB_HSW			48	/* hsync width */
#define S3CFB_HBP			40	/* back porch */

#define S3CFB_VFP			130	/* front porch */
#define S3CFB_VSW			3	/* vsync width */
#define S3CFB_VBP			29	/* back porch */

#define S3CFB_HRES			800	/* horizon pixel  x resolition */
#define S3CFB_VRES			480	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	800	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	960	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		800	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		480	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	3//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"A70 800*480"

#elif defined(CONFIG_FB_S3C_LCD800600)
#define S3CFB_HFP			15	/* front porch */
#define S3CFB_HSW			95	/* hsync width */
#define S3CFB_HBP			47	/* back porch */

#define S3CFB_VFP			12	/* front porch */
#define S3CFB_VSW			4	/* vsync width */
#define S3CFB_VBP			16	/* back porch */

#define S3CFB_HRES			800	/* horizon pixel  x resolition */
#define S3CFB_VRES			600	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	800	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	1200	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		800	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		600	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	4//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"A104 800*600"

#elif defined(CONFIG_FB_S3C_LCD1024768)
#define S3CFB_HFP			15	/* front porch */
#define S3CFB_HSW			95	/* hsync width */
#define S3CFB_HBP			47	/* back porch */

#define S3CFB_VFP			12	/* front porch */
#define S3CFB_VSW			4	/* vsync width */
#define S3CFB_VBP			16	/* back porch */

#define S3CFB_HRES			1024	/* horizon pixel  x resolition */
#define S3CFB_VRES			768	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	1024	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	1536	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		1024	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		768	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	4//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"A104 1024*768"

#elif defined(CONFIG_FB_S3C_VGA640480)
#define S3CFB_HFP			16	/* front porch */
#define S3CFB_HSW			10	/* hsync width */
#define S3CFB_HBP			144	/* back porch */

#define S3CFB_VFP			32	/* front porch */
#define S3CFB_VSW			2	/* vsync width */
#define S3CFB_VBP			13	/* back porch */

#define S3CFB_HRES			640	/* horizon pixel  x resolition */
#define S3CFB_VRES			480	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	640	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	960	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		640	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		480	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	4//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"VGA 640*480"

#elif defined(CONFIG_FB_S3C_VGA800600)
#define S3CFB_HFP			15 /* front porch */
#define S3CFB_HSW			95 /* hsync width */
#define S3CFB_HBP			47 /* back porch */

#define S3CFB_VFP			12 /* front porch */
#define S3CFB_VSW			4  /* vsync width */
#define S3CFB_VBP			16 /* back porch */

#define S3CFB_HRES			800	/* horizon pixel  x resolition */
#define S3CFB_VRES			600	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	800	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	1200	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		800	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		600	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	60	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	4//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"VGA 800*600"

#elif defined(CONFIG_FB_S3C_VGA1024768)
#define S3CFB_HFP			199	/* front porch */
#define S3CFB_HSW			5	/* hsync width */
#define S3CFB_HBP			1	/* back porch */

#define S3CFB_VFP			1	/* front porch */
#define S3CFB_VSW			1	/* vsync width */
#define S3CFB_VBP			1	/* back porch */

#define S3CFB_HRES			1024	/* horizon pixel  x resolition */
#define S3CFB_VRES			768	/* line cnt       y resolution */

#define S3CFB_HRES_VIRTUAL	1024	/* horizon pixel  x resolition */
#define S3CFB_VRES_VIRTUAL	1536	/* line cnt       y resolution */

#define S3CFB_HRES_OSD		1024	/* horizon pixel  x resolition */
#define S3CFB_VRES_OSD		768	/* line cnt       y resolution */

#define S3CFB_VFRAME_FREQ	40	/* frame rate freq */

#define S3CFB_PIXEL_CLOCK	4//(S3CFB_VFRAME_FREQ * (S3CFB_HFP + S3CFB_HSW + S3CFB_HBP + S3CFB_HRES) * (S3CFB_VFP + S3CFB_VSW + S3CFB_VBP + S3CFB_VRES))
#define LCD_TYPE			"VGA 1024*768"

#endif

struct s3c_fb_pd_win smdk2416_fb_win[] = {
	[0] = {
		/* think this is the same as the smdk6410 */
		.win_mode	= {
			.pixclock		= S3CFB_PIXEL_CLOCK,
			.left_margin	= S3CFB_HFP,	/* for HFPD*/
			.right_margin	= S3CFB_HBP,	/* for HBPD*/
			.upper_margin	= S3CFB_VFP,	/* for VFPD*/
			.lower_margin	= S3CFB_VBP,	/* for VBPD*/
			.hsync_len		= S3CFB_HSW,	/* for HSPW*/
			.vsync_len		= S3CFB_VSW,	/* for VSPW*/

			.xres			= S3CFB_HRES,
			.yres			= S3CFB_VRES,
			.name			= LCD_TYPE,
		},
#if defined(CONFIG_FB_S3C_BPP_8)
		.default_bpp		= 8,
#elif defined(CONFIG_FB_S3C_BPP_16)
		.default_bpp		= 16,
#elif defined(CONFIG_FB_S3C_BPP_24)
		.default_bpp		= 24,
#elif defined(CONFIG_FB_S3C_BPP_32)
		.default_bpp		= 32,
#endif
		.max_bpp			= 32,

		.virtual_y			= S3CFB_VRES_VIRTUAL,
		.virtual_x			= S3CFB_HRES_VIRTUAL,
	},
#if (CONFIG_FB_S3C_NUM == 2)
	[1] = {
		/* think this is the same as the smdk6410 */
		.win_mode	= {
			.left_margin	= S3CFB_HFP,	/* for HFPD*/
			.right_margin	= S3CFB_HBP,	/* for HBPD*/
			.upper_margin	= S3CFB_VFP,	/* for VFPD*/
			.lower_margin	= S3CFB_VBP,	/* for VBPD*/
			.hsync_len		= S3CFB_HSW,	/* for HSPW*/
			.vsync_len		= S3CFB_VSW,	/* for VSPW*/

			.xres			= S3CFB_HRES,
			.yres			= S3CFB_VRES,
			.name			= LCD_TYPE,
		},
#if defined(CONFIG_FB_S3C_BPP_8)
		.default_bpp		= 8,
#elif defined(CONFIG_FB_S3C_BPP_16)
		.default_bpp		= 16,
#elif defined(CONFIG_FB_S3C_BPP_24)
		.default_bpp		= 24,
#elif defined(CONFIG_FB_S3C_BPP_32)
		.default_bpp		= 32,
#endif
		.max_bpp			= 32,

		.virtual_y			= S3CFB_VRES_VIRTUAL,
		.virtual_x			= S3CFB_HRES_VIRTUAL,
	},
#endif
};

static void s3c2416_fb_gpio_setup_24bpp(void)
{
	unsigned int gpio;

	for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPD(0); gpio <= S3C2410_GPD(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

static struct s3c_fb_platdata smdk2416_fb_platdata = {
	.win[0]		= &smdk2416_fb_win[0],
#if (CONFIG_FB_S3C_NUM == 2)
	.win[1]		= &smdk2416_fb_win[1],
#endif
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc0_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= S3C2410_GPF(1),
	.ext_cd_gpio_invert	= 1,
};

static struct s3c_sdhci_platdata smdk2416_hsmmc1_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_NONE,
};

static struct platform_device *smdk2416_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_wdt,
	&s3c_device_ohci,
	&s3c_device_i2c0,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_usb_hsudc,
#ifdef CONFIG_DM9000
	&tq2416_device_eth,
#endif
#ifdef CONFIG_RTC_DRV_S3C
	&s3c_device_rtc,
#endif
};

static void __init smdk2416_map_io(void)
{
	s3c24xx_init_io(smdk2416_iodesc, ARRAY_SIZE(smdk2416_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdk2416_uartcfgs, ARRAY_SIZE(smdk2416_uartcfgs));
}

static void __init smdk2416_machine_init(void)
{
	s3c_i2c0_set_platdata(NULL);
	s3c_fb_set_platdata(&smdk2416_fb_platdata);

	s3c_sdhci0_set_platdata(&smdk2416_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&smdk2416_hsmmc1_pdata);

	s3c24xx_hsudc_set_platdata(&smdk2416_hsudc_platdata);

	gpio_request(S3C2410_GPB(4), "USBHost Power");
	gpio_direction_output(S3C2410_GPB(4), 1);

	gpio_request(S3C2410_GPB(3), "Display Power");
	gpio_direction_output(S3C2410_GPB(3), 1);

	gpio_request(S3C2410_GPB(1), "Display Reset");
	gpio_direction_output(S3C2410_GPB(1), 1);

	platform_add_devices(smdk2416_devices, ARRAY_SIZE(smdk2416_devices));
	smdk_machine_init();
}

MACHINE_START(SMDK2416, "SMDK2416")
	/* Maintainer: Yauhen Kharuzhy <jekhor@gmail.com> */
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= smdk2416_map_io,
	.init_machine	= smdk2416_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
