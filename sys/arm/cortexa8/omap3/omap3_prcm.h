/*
 * Copyright (c) 2010
 *	Ben Gray <ben.r.gray@gmail.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Ben Gray.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BEN GRAY ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL BEN GRAY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * Texas Instruments - OMAP3xxx series processors
 *
 * Reference:
 *  OMAP35x Applications Processor
 *   Technical Reference Manual
 *  (omap35xx_techref.pdf)
 */
#ifndef _OMAP3_PRCM_H_
#define _OMAP3_PRCM_H_


/*
#define OMAP3_MODULE_DOMAIN_IVA2			1
#define OMAP3_MODULE_DOMAIN_OCP				2
#define OMAP3_MODULE_DOMAIN_MPU				3
#define OMAP3_MODULE_DOMAIN_CORE			4
#define OMAP3_MODULE_DOMAIN_SGX				5
#define OMAP3_MODULE_DOMAIN_WKUP			6
#define OMAP3_MODULE_DOMAIN_CLOCK_CONTROL	7
#define OMAP3_MODULE_DOMAIN_DSS				8
#define OMAP3_MODULE_DOMAIN_CAM				9
#define OMAP3_MODULE_DOMAIN_PER				10
#define OMAP3_MODULE_DOMAIN_EMU				11
#define OMAP3_MODULE_DOMAIN_GLOBAL			12
#define OMAP3_MODULE_DOMAIN_NEON			13
#define OMAP3_MODULE_DOMAIN_USBHOST			14
*/

#define OMAP3_MODULE_REG_OFFSET(x)	(((x) >> 8) & 0xffffff)
#define OMAP3_MODULE_REG_BIT(x)		(((x) >> 0) & 0xff)

#define OMAP3_MODULE_CREATE(r,b)	((((r) & 0xffffff) << 8) | \
									 (((b) & 0xff) << 0))



/*
 * MMC/SD/SDIO Module(s) functional and interface clocks
 */
#define OMAP3_MODULE_MMC1_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 24)
#define OMAP3_MODULE_MMC1_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 24)

#define OMAP3_MODULE_MMC2_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 25)
#define OMAP3_MODULE_MMC2_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 25)

#define OMAP3_MODULE_MMC3_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 30)
#define OMAP3_MODULE_MMC3_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 30)

/*
 * I2C Module(s) functional and interface clocks
 */
#define OMAP3_MODULE_I2C1_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 15)
#define OMAP3_MODULE_I2C1_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 15)

#define OMAP3_MODULE_I2C2_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 16)
#define OMAP3_MODULE_I2C2_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 16)

#define OMAP3_MODULE_I2C3_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 17)
#define OMAP3_MODULE_I2C3_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 17)

/*
 * UART Module(s) functional and interface clocks
 */
#define OMAP3_MODULE_UART1_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 13)
#define OMAP3_MODULE_UART1_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 13)

#define OMAP3_MODULE_UART2_FCLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 14)
#define OMAP3_MODULE_UART2_ICLK			\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 14)


/*
 * General Purpose Timers Module(s) functional and interface clocks
 */
#define OMAP3_MODULE_GPTIMER10_FCLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 11)
#define OMAP3_MODULE_GPTIMER10_ICLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 11)

#define OMAP3_MODULE_GPTIMER11_FCLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN1_CORE, 12)
#define OMAP3_MODULE_GPTIMER11_ICLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN1_CORE, 12)


/*
 * USB Module(s) functional and interface clocks
 */
#define OMAP3_MODULE_USBTTL_FCLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN3_CORE, 2)
#define OMAP3_MODULE_USBTTL_ICLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN3_CORE, 2)

#define OMAP3_MODULE_USBHOST_120M_FCLK	\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN_USBHOST, 1)
#define OMAP3_MODULE_USBHOST_48M_FCLK	\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_FCLKEN_USBHOST, 0)
#define OMAP3_MODULE_USBHOST_ICLK		\
	OMAP3_MODULE_CREATE(OMAP35XX_CM_ICLKEN_USBHOST, 0)





int
omap3_prcm_enable_clk(unsigned int module);

int
omap3_prcm_disable_clk(unsigned int module);


#define PRCM_USE_SYSCLK		1
#define PRCM_USE_32KCLK		0

int
omap3_prcm_set_gptimer_clksrc(int timer, int sys_clk);


#endif   /* _OMAP3_PRCM_H_ */
