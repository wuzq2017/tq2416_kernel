#ifndef __ASM_ARCH_REGS_GPIO_2416_H
#define __ASM_ARCH_REGS_GPIO_2416_H __FILE__
#include <mach/gpio-nrs.h>
#include <mach/regs-gpio.h>

#define S3C2410_GPE0	S3C2410_GPE(0)
#define S3C2410_GPE1	S3C2410_GPE(1)
#define S3C2410_GPE2	S3C2410_GPE(2)
#define S3C2410_GPE3	S3C2410_GPE(3)
#define S3C2410_GPE4	S3C2410_GPE(4)
#define S3C2410_GPE5	S3C2410_GPE(5)
#define S3C2410_GPE6	S3C2410_GPE(6)
#define S3C2410_GPE7	S3C2410_GPE(7)
#define S3C2410_GPE8	S3C2410_GPE(8)
#define S3C2410_GPE9	S3C2410_GPE(9)
#define S3C2410_GPE10	S3C2410_GPE(10)
#define S3C2410_GPE11	S3C2410_GPE(11)
#define S3C2410_GPE12	S3C2410_GPE(12)
#define S3C2410_GPE13	S3C2410_GPE(13)
#define S3C2410_GPE14	S3C2410_GPE(14)
#define S3C2410_GPE15	S3C2410_GPE(15)





#define S3c2416_GPF_FN(x,b) (x<<b*2)
#define S3c2416_GPG_FN(x,b) (x<<b*2)

#define S3C2416_GPF0_INPUT  S3c2416_GPF_FN(0x01,0)
#define S3C2416_GPF0_OUTP   S3c2416_GPF_FN(0x02,0)


#define S3C2416_GPF1_INPUT  S3c2416_GPF_FN(0x01,1)
#define S3C2416_GPF1_OUTP   S3c2416_GPF_FN(0x02,1)

#define S3C2416_GPF2_INPUT  S3c2416_GPF_FN(0x01,2)
#define S3C2416_GPF2_OUTP   S3c2416_GPF_FN(0x02,2)


#define S3C2416_GPF3_INPUT  S3c2416_GPF_FN(0x01,3)
#define S3C2416_GPF3_OUTP   S3c2416_GPF_FN(0x02,3)

#define S3C2416_GPF4_INPUT  S3c2416_GPF_FN(0x01,4)
#define S3C2416_GPF4_OUTP   S3c2416_GPF_FN(0x02,4)

#define S3C2416_GPF5_INPUT  S3c2416_GPF_FN(0x01,5)
#define S3C2416_GPF5_OUTP   S3c2416_GPF_FN(0x02,5)

#define S3C2416_GPF6_INPUT  S3c2416_GPF_FN(0x01,6)
#define S3C2416_GPF6_OUTP   S3c2416_GPF_FN(0x02,6)

#define S3C2416_GPF7_INPUT  S3c2416_GPF_FN(0x01,7)
#define S3C2416_GPF7_OUTP   S3c2416_GPF_FN(0x02,7)

#define S3C2416_GPG0_INPUT  S3c2416_GPG_FN(0x01,0)
#define S3C2416_GPG0_OUTP   S3c2416_GPG_FN(0x02,0)

#define S3C2416_GPG1_INPUT  S3c2416_GPG_FN(0x01,1)
#define S3C2416_GPG1_OUTP   S3c2416_GPG_FN(0x02,1)

#define S3C2416_GPG2_INPUT  S3c2416_GPG_FN(0x01,2)
#define S3C2416_GPG2_OUTP   S3c2416_GPG_FN(0x02,2)

#endif
