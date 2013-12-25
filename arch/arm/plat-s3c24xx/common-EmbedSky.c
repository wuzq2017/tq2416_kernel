/* linux/arch/arm/plat-s3c24xx/common-EmbedSky.c
 *
 * Copyright (c) 2012 EmbedSky Tech
 *
 * Common code for TQ2440 and TQ2416 boards
 *
 * http://www.embedsky.net
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/sysdev.h>
#include <linux/platform_device.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>
#include <mach/leds-gpio.h>

#include <plat/nand.h>

#include <plat/common-EmbedSky.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/pm.h>

/* LED devices */

static struct s3c24xx_led_platdata smdk_pdata_led4 = {
	.gpio		= S3C2410_GPF(4),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.name		= "led4",
	.def_trigger	= "timer",
};

static struct s3c24xx_led_platdata smdk_pdata_led5 = {
	.gpio		= S3C2410_GPF(5),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.name		= "led5",
	.def_trigger	= "nand-disk",
};

static struct s3c24xx_led_platdata smdk_pdata_led6 = {
	.gpio		= S3C2410_GPF(6),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.name		= "led6",
};

static struct s3c24xx_led_platdata smdk_pdata_led7 = {
	.gpio		= S3C2410_GPF(7),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.name		= "led7",
};

static struct platform_device smdk_led4 = {
	.name		= "s3c24xx_led",
	.id		= 0,
	.dev		= {
		.platform_data = &smdk_pdata_led4,
	},
};

static struct platform_device smdk_led5 = {
	.name		= "s3c24xx_led",
	.id		= 1,
	.dev		= {
		.platform_data = &smdk_pdata_led5,
	},
};

static struct platform_device smdk_led6 = {
	.name		= "s3c24xx_led",
	.id		= 2,
	.dev		= {
		.platform_data = &smdk_pdata_led6,
	},
};

static struct platform_device smdk_led7 = {
	.name		= "s3c24xx_led",
	.id		= 3,
	.dev		= {
		.platform_data = &smdk_pdata_led7,
	},
};

/* NAND parititon from 2.4.18-swl5 */

static struct mtd_partition tq_default_nand_part[] = {
	[0] = {
		.name		= "Bootloader",
		.offset		= 0,
		.size		= 0x100000,
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	[1] = {
		.name		= "LOGO",
		.offset		= 0x100000,
		.size		= 0x200000,
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	[2] = {
		.name		= "Kernel",
		.offset		= 0x300000,
		.size		= 0x400000,
		.mask_flags	= MTD_CAP_NANDFLASH,
	},
	[3] = {
		.name		= "ROOTFS",
		.offset		= 0x700000,
		.size		= MTDPART_SIZ_FULL,//0xf900000
	},
};

static struct s3c2410_nand_set tq_nand_sets[] = {
	[0] = {
		.name		= "nandflash0",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(tq_default_nand_part),
		.partitions	= tq_default_nand_part,
	},
};

/* choose a set of timings which should suit most 512Mbit
 * chips and beyond.
*/

static struct s3c2410_platform_nand tq_nand_info = {
	.tacls		= 20,
	.twrph0		= 60,
	.twrph1		= 20,
	.nr_sets	= ARRAY_SIZE(tq_nand_sets),
	.sets		= tq_nand_sets,
};

/* devices we initialise */

static struct platform_device __initdata *tq_devs[] = {
	&s3c_device_nand,
//	&smdk_led4,
//	&smdk_led5,
//	&smdk_led6,
//	&smdk_led7,
};

void __init tq_machine_init(void)
{
	/* Configure the LEDs (even if we have no LED support)*/

#if 0
	s3c_gpio_cfgpin(S3C2410_GPF(4), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPF(5), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPF(6), S3C2410_GPIO_OUTPUT);
	s3c_gpio_cfgpin(S3C2410_GPF(7), S3C2410_GPIO_OUTPUT);

	s3c2410_gpio_setpin(S3C2410_GPF(4), 1);
	s3c2410_gpio_setpin(S3C2410_GPF(5), 1);
	s3c2410_gpio_setpin(S3C2410_GPF(6), 1);
	s3c2410_gpio_setpin(S3C2410_GPF(7), 1);
#endif

//	if (machine_is_smdk2443())
		tq_nand_info.twrph0 = 50;

	s3c_nand_set_platdata(&tq_nand_info);

	platform_add_devices(tq_devs, ARRAY_SIZE(tq_devs));

	s3c_pm_init();
}
