/* linux/arch/arm/mach-s3c2416/dev-keypad.c
 * package: TQ2416-Linux-2.6.36
 * author: Paul
 * Copyright 2011 EmbedSky
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
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <plat/devs.h>
/******************************************************************************
 * TQ2416's keypad; 
		number		function		gpio
		KEY1		up				GPF6
		KEY2		down			GPG1
		KEY3		left			GPF7
		KEY4		right			GPF5
		KEY5		esc				GPG3
		KEY6		enter			GPG2
******************************************************************************/

static struct gpio_keys_button gpio_buttons[] = 
{
	{//UP 1
		.gpio		= S3C2410_GPF(6),
		.code		= KEY_UP,
		.desc		= "UP",
		.active_low	= 1,
		.wakeup		= 0,
	},
	{//DOWN 2
		.gpio		= S3C2410_GPG(1),
		.code		=  KEY_DOWN,
		.desc		= "DOWN",
		.active_low	= 1,
		.wakeup		= 0,

	},
	{//LEFT 3
		.gpio		= S3C2410_GPF(7),
		.code		= KEY_LEFT,
		.desc		= "LEFT",
		.active_low	= 1,
		.wakeup		= 0,
	},
	{//RIGHT 4
		.gpio		= S3C2410_GPF(5),
		.code		= KEY_RIGHT,
		.desc		= "RIGHT",
		.active_low	= 1,
		.wakeup		= 0,

	},
	{//ESC 5
		.gpio		= S3C2410_GPG(3),
		.code		= KEY_ESC,
		.desc		= "ESC",
		.active_low	= 1,
		.wakeup		= 0,

	},
	{//ENTER 6
		.gpio		= S3C2410_GPG(2),
		.code		= KEY_KPENTER,
		.desc		= "ENTER",
		.active_low	= 1,
		.wakeup		= 0,
	},
};

static struct gpio_keys_platform_data gpio_button_data = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

struct platform_device s3c_device_gpio_button = {
	.name		= "tq2416-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &gpio_button_data,
	}
};
