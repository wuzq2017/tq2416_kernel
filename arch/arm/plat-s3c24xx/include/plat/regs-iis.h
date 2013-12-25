/* arch/arm/mach-s3c2410/include/mach/regs-iis.h
 *
 * Copyright (c) 2003 Simtec Electronics <linux@simtec.co.uk>
 *		      http://www.simtec.co.uk/products/SWLINUX/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * S3C2410 IIS register definition
*/

#ifndef __ASM_ARCH_REGS_IIS_H
#define __ASM_ARCH_REGS_IIS_H

#define S3C2410_IISCON	 (0x00)  //both s3c2416 and s3c2410 are the same
#define S3C2410_IISMOD	 (0x04)  //both s3c2416 and s3c2410 are the same


#ifdef CONFIG_CPU_S3C2416  //for s3c2416
//IISCON 00 
#define S3C2410_IISCON_LRINDEX	  (1<<11)
#define S3C2416_IISCON_FTXURINTEN	(0x1<<16)
#define S3C2416_IISCON_TXDMACTIVE	(0x1<<2)
#define S3C2416_IISCON_RXDMACTIVE	(0x1<<1)
#define S3C2416_IISCON_I2SACTIVE	(0x1<<0)
//IISMOD  04

#define S3C2416_IISMOD_BLC_MASK			(0x3<<13)
#define S3C2416_IISMOD_BLC_16BITS		(0x0<<13)
#define S3C2416_IISMOD_BLC_8BITS		(0x1<<13)
#define S3C2416_IISMOD_BLC_24BITS		(0x2<<13)

#define S3C2416_IISCON_LRP_L_H			(0x1<<7)//low for left ,high for right
#define S3C2416_IISCON_LRP_L_L			(0x0<<7)//low for left ,high for right


#define S3C2416_IISMOD_INTERNAL_CLK		(0x0<<12)
#define S3C2416_IISMOD_EXTERNAL_CLK		(0x1<<12)


#define S3C2416_IISMOD_CLK_MASK	(0x7<<10)
#define S3C2416_IISMOD_MASTER_PCLK	(0x0<<10)//master mode use PCLK
#define S3C2416_IISMOD_MASTER_CLKAUDIO	(0x1<<10)//master mode use CLKAUDIO
#define S3C2416_IISMOD_SLAVE_PCLK	(0x2<<10)//slave mode use PCLK
#define S3C2416_IISMOD_SLAVE_CLKAUDIO  	(0x3<<10)//slave mode use CLKAUDIO


#define S3C2416_IISMOD_MODE_MASK		(0x3<<8)
#define S3C2416_IISMOD_TXRXMODE		(0x2<<8)

#define S3C2416_IISMOD_FM_MASK			(0x3<<5)
#define S3C2416_IISMOD_IIS			(0x0<<5)
#define S3C2416_IISMOD_MSB			(0x1<<5)
#define S3C2416_IISMOD_LSB			(0x2<<5)
#define S3C2416_IISMOD_FS_MASK			(0x3<<3)
#define S3C2416_IISMOD_768FS			(0x3<<3)
#define S3C2416_IISMOD_384FS			(0x2<<3)
#define S3C2416_IISMOD_512FS			(0x1<<3)
#define S3C2416_IISMOD_256FS			(0x0<<3)
#define S3C2410_IISMOD_256FS	  		(0x0<<3)
#define S3C2410_IISMOD_384FS	  		(0x2<<3)
#define S3C2416_IISMOD_BFS_MASK			(0x3<<1)
#define S3C2416_IISMOD_24FS			(0x3<<1)
#define S3C2416_IISMOD_16FS			(0x2<<1)
#define S3C2416_IISMOD_48FS			(0x1<<1)
#define S3C2416_IISMOD_32FS			(0x0<<1)
#define S3C2410_IISMOD_32FS	  		(0x0<<1)
#define S3C2416_IISMOD_8BIT	  		(0x1<<0)
#define S3C2416_IISMOD_16BIT	  		(0x0<<0)
//IISFIC  08
#define S3C2416_IISFIC				(0x08)
#define S3C2416_IIS_TX_FLUSH			(0x1<<15)
#define S3C2416_IIS_RX_FLUSH			(0x1<<7)
//IISPSR  0C
#define S3C2410_IISPSR		(0x0c)
#define S3C2416_IISPSR				(0x0c)
#define S3C2416_IISPSR_PSRAEN	 		(1<<15)
#define S3C2416_IISPSR_PS_MASK	 		(0x3F<<8)
#define S3C2416_IISPSR_INTSHIFT		(8)
//IISTXD  10
#define S3C2416_IISFIFOTX  	(0x10)
//IISRXD  14
#define S3C2416_IISFIFORX  	(0x14)

#else  //non S3C2416
//IISTXD  10
#define S3C2416_IISFIFOTX  	(0x10)
//IISRXD  14
#define S3C2416_IISFIFORX  	(0x14)
#define S3C2410_IISCON_LRINDEX	  (1<<8)
#define S3C2410_IISCON_TXFIFORDY  (1<<7)
#define S3C2410_IISCON_RXFIFORDY  (1<<6)
#define S3C2410_IISCON_TXDMAEN	  (1<<5)
#define S3C2410_IISCON_RXDMAEN	  (1<<4)
#define S3C2410_IISCON_TXIDLE	  (1<<3)
#define S3C2410_IISCON_RXIDLE	  (1<<2)
#define S3C2410_IISCON_PSCEN	  (1<<1)
#define S3C2410_IISCON_IISEN	  (1<<0)

#define S3C2410_IISMOD	 (0x04)

#define S3C2440_IISMOD_MPLL	  (1<<9)
#define S3C2410_IISMOD_SLAVE	  (1<<8)
#define S3C2410_IISMOD_NOXFER	  (0<<6)
#define S3C2410_IISMOD_RXMODE	  (1<<6)
#define S3C2410_IISMOD_TXMODE	  (2<<6)
#define S3C2410_IISMOD_TXRXMODE	  (3<<6)
#define S3C2410_IISMOD_LR_LLOW	  (0<<5)
#define S3C2410_IISMOD_LR_RLOW	  (1<<5)
#define S3C2410_IISMOD_IIS	  (0<<4)
#define S3C2410_IISMOD_MSB	  (1<<4)
#define S3C2410_IISMOD_8BIT	  (0<<3)
#define S3C2410_IISMOD_16BIT	  (1<<3)
#define S3C2410_IISMOD_BITMASK	  (1<<3)
#define S3C2410_IISMOD_256FS	  (0<<2)
#define S3C2410_IISMOD_384FS	  (1<<2)
#define S3C2410_IISMOD_16FS	  (0<<0)
#define S3C2410_IISMOD_32FS	  (1<<0)
#define S3C2410_IISMOD_48FS	  (2<<0)
#define S3C2410_IISMOD_FS_MASK	  (3<<0)

#define S3C2410_IISPSR		(0x08)
#define S3C2410_IISPSR_INTMASK	(31<<5)
#define S3C2410_IISPSR_INTSHIFT	(5)
#define S3C2410_IISPSR_EXTMASK	(31<<0)
#define S3C2410_IISPSR_EXTSHFIT	(0)

#define S3C2410_IISFCON  (0x0c)

#define S3C2410_IISFCON_TXDMA	  (1<<15)
#define S3C2410_IISFCON_RXDMA	  (1<<14)
#define S3C2410_IISFCON_TXENABLE  (1<<13)
#define S3C2410_IISFCON_RXENABLE  (1<<12)
#define S3C2410_IISFCON_TXMASK	  (0x3f << 6)
#define S3C2410_IISFCON_TXSHIFT	  (6)
#define S3C2410_IISFCON_RXMASK	  (0x3f)
#define S3C2410_IISFCON_RXSHIFT	  (0)

#define S3C2410_IISFIFO  (0x10)
#endif

#endif /* __ASM_ARCH_REGS_IIS_H */
