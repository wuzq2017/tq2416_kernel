/* arch/arm/mach-s3c2410/include/mach/regs-s3c2416-mem.h
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
 * S3C2416 memory register definitions
*/

#ifndef __ASM_ARM_REGS_S3C2416_MEM
#define __ASM_ARM_REGS_S3C2416_MEM

#ifndef S3C2416_MEMREG
#define S3C2416_MEMREG(x)		(S3C24XX_VA_MEMCTRL + (x))
#endif

#define S3C2416_EBIREG(x)		(S3C2416_VA_EBI + (x))

#define S3C2416_SSMCREG(x)		(S3C2416_VA_SSMC + (x))
#define S3C2416_SSMC(x, o)		(S3C2416_SSMCREG((x * 0x20) + (o)))

#define S3C2416_BANKCFG			S3C2416_MEMREG(0x00)
#define S3C2416_BANKCON1		S3C2416_MEMREG(0x04)
#define S3C2416_BANKCON2		S3C2416_MEMREG(0x08)
#define S3C2416_BANKCON3		S3C2416_MEMREG(0x0C)

#define S3C2416_REFRESH			S3C2416_MEMREG(0x10)
#define S3C2416_TIMEOUT			S3C2416_MEMREG(0x14)

/* EBI control registers */

#define S3C2416_EBI_PR0			S3C2416_EBIREG(0x00)
#define S3C2416_EBI_PR1			S3C2416_EBIREG(0x04)
#define S3C2416_EBI_BANKCFG		S3C2416_EBIREG(0x08)

/* SSMC control registers */

#define S3C2416_SSMC_BANK(x)	S3C2416_SSMC(x, 0x00)
#define S3C2416_SMBIDCYR(x)		S3C2416_SSMC(x, 0x00)
#define S3C2416_SMBWSTRDR(x)	S3C2416_SSMC(x, 0x04)
#define S3C2416_SMBWSTWRR(x)	S3C2416_SSMC(x, 0x08)
#define S3C2416_SMBWSTOENR(x)	S3C2416_SSMC(x, 0x0C)
#define S3C2416_SMBWSTWENR(x)	S3C2416_SSMC(x, 0x10)
#define S3C2416_SMBCR(x)		S3C2416_SSMC(x, 0x14)
#define S3C2416_SMBSR(x)		S3C2416_SSMC(x, 0x18)
#define S3C2416_SMBWSTBRDR(x)	S3C2416_SSMC(x, 0x1C)

#endif /*  __ASM_ARM_REGS_S3C2416_MEM */
