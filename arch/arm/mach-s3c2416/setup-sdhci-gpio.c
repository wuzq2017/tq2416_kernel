/* linux/arch/arm/plat-s3c2416/setup-sdhci-gpio.c
 *
 * Copyright 2010 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 *
 * S3C2416 - Helper functions for setting up SDHCI device(s) GPIO (HSMMC)
 *
 * Based on mach-s3c64xx/setup-sdhci-gpio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/sdhci.h>

void s3c2416_setup_sdhci0_cfg_gpio(struct platform_device *dev, int width)
{
	s3c_gpio_cfgrange_nopull(S3C2410_GPE(5), 2 + width, S3C_GPIO_SFN(2));
}

void s3c2416_setup_sdhci1_cfg_gpio(struct platform_device *dev, int width)
{
	struct s3c_sdhci_platdata *pdata = dev->dev.platform_data;

	/* Set all the necessary GPH pins to special-function 2 */
	s3c_gpio_cfgrange_nopull(S3C2410_GPL(0), width, S3C_GPIO_SFN(2));
	s3c_gpio_cfgrange_nopull(S3C2410_GPL(8), 2, S3C_GPIO_SFN(2));

	if (pdata->cd_type == S3C_SDHCI_CD_INTERNAL) {
		s3c_gpio_setpull(S3C2410_GPF(0), S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(S3C2410_GPF(0), S3C_GPIO_SFN(0));
	}
}
