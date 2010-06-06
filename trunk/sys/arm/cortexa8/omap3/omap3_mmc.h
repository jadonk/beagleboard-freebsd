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

#ifndef _OMAP3_MMC_H_
#define _OMAP3_MMC_H_

/**
 * Header file for the OMAP3530 MMC/SD/SDIO driver.
 *
 * Simply contains register bit flags.
 */

#define OMAP3_MMC_REF_CLK			96000000	/* 96Mhz */

#define MMCHS_STAT_BADA				(1UL << 29)
#define MMCHS_STAT_CERR				(1UL << 28)
#define MMCHS_STAT_ACE				(1UL << 24)
#define MMCHS_STAT_DEB				(1UL << 22)
#define MMCHS_STAT_DCRC				(1UL << 21)
#define MMCHS_STAT_DTO				(1UL << 20)
#define MMCHS_STAT_CIE				(1UL << 19)
#define MMCHS_STAT_CEB				(1UL << 18)
#define MMCHS_STAT_CCRC				(1UL << 17)
#define MMCHS_STAT_CTO				(1UL << 16)
#define MMCHS_STAT_ERRI				(1UL << 15)
#define MMCHS_STAT_OBI				(1UL << 9)
#define MMCHS_STAT_CIRQ				(1UL << 8)
#define MMCHS_STAT_BRR				(1UL << 5)
#define MMCHS_STAT_BWR				(1UL << 4)
#define MMCHS_STAT_BGE				(1UL << 2)
#define MMCHS_STAT_TC				(1UL << 1)
#define MMCHS_STAT_CC				(1UL << 0)

#define MMCHS_STAT_CLEAR_MASK		0x3BFF8337UL


#define MMCHS_SYSCTL_SRD			(1UL << 26)
#define MMCHS_SYSCTL_SRC			(1UL << 25)
#define MMCHS_SYSCTL_SRA			(1UL << 24)
#define MMCHS_SYSCTL_DTO(x)			(((x) & 0xf) << 16)
#define MMCHS_SYSCTL_DTO_MASK		MMCHS_SYSCTL_DTO(0xf)
#define MMCHS_SYSCTL_CLKD(x)		(((x) & 0x3ff) << 6)
#define MMCHS_SYSCTL_CLKD_MASK		MMCHS_SYSCTL_CLKD(0x3ff)
#define MMCHS_SYSCTL_CEN			(1UL << 2)
#define MMCHS_SYSCTL_ICS			(1UL << 1)
#define MMCHS_SYSCTL_ICE			(1UL << 0)

#define MMCHS_HCTL_OBWE				(1UL << 27)
#define MMCHS_HCTL_REM				(1UL << 26)
#define MMCHS_HCTL_INS				(1UL << 25)
#define MMCHS_HCTL_IWE				(1UL << 24)
#define MMCHS_HCTL_IBG				(1UL << 19)
#define MMCHS_HCTL_RWC				(1UL << 18)
#define MMCHS_HCTL_CR				(1UL << 17)
#define MMCHS_HCTL_SBGR				(1UL << 16)
#define MMCHS_HCTL_SDVS_MASK		MMCHS_HCTL_SDVS(0x3)
#define MMCHS_HCTL_SDVS(x)			(((x) & 0x3) << 9)
#define MMCHS_HCTL_SDBP				(1UL << 8)
#define MMCHS_HCTL_DTW				(1UL << 1)

#define MMCHS_CAPA_VS18				(1UL << 26)
#define MMCHS_CAPA_VS30				(1UL << 25)
#define MMCHS_CAPA_VS33				(1UL << 24)



#define MMCHS_CMD_CMD_TYPE_IO_ABORT	(3UL << 21)
#define MMCHS_CMD_CMD_TYPE_FUNC_SEL	(2UL << 21)
#define MMCHS_CMD_CMD_TYPE_SUSPEND	(1UL << 21)
#define MMCHS_CMD_CMD_TYPE_OTHERS	(0UL << 21)
#define MMCHS_CMD_CMD_TYPE_MASK		(3UL << 22)

#define MMCHS_CMD_DP				(1UL << 21)
#define MMCHS_CMD_CICE				(1UL << 20)
#define MMCHS_CMD_CCCE				(1UL << 19)

#define MMCHS_CMD_RSP_TYPE_MASK		(3UL << 16)
#define MMCHS_CMD_RSP_TYPE_NO		(0UL << 16)
#define MMCHS_CMD_RSP_TYPE_136		(1UL << 16)
#define MMCHS_CMD_RSP_TYPE_48		(2UL << 16)
#define MMCHS_CMD_RSP_TYPE_48_BSY	(3UL << 16)

#define MMCHS_CMD_MSBS				(1UL << 5)
#define MMCHS_CMD_DDIR				(1UL << 4)
#define MMCHS_CMD_ACEN				(1UL << 2)
#define MMCHS_CMD_BCE				(1UL << 1)
#define MMCHS_CMD_DE				(1UL << 0)



#define MMCHS_CON_CLKEXTFREE		(1UL << 16)
#define MMCHS_CON_PADEN				(1UL << 15)
#define MMCHS_CON_OBIE				(1UL << 14)
#define MMCHS_CON_OBIP				(1UL << 13)
#define MMCHS_CON_CEATA				(1UL << 12)
#define MMCHS_CON_CTPL				(1UL << 11)

#define MMCHS_CON_DVAL_8_4MS		(3UL << 9)
#define MMCHS_CON_DVAL_1MS			(2UL << 9)
#define MMCHS_CON_DVAL_231US		(1UL << 9)
#define MMCHS_CON_DVAL_33US			(0UL << 9)
#define MMCHS_CON_DVAL_MASK			(3UL << 9)

#define MMCHS_CON_WPP				(1UL << 8)
#define MMCHS_CON_CDP				(1UL << 7)
#define MMCHS_CON_MIT				(1UL << 6)
#define MMCHS_CON_DW8				(1UL << 5)
#define MMCHS_CON_MODE				(1UL << 4)
#define MMCHS_CON_STR				(1UL << 3)
#define MMCHS_CON_HR				(1UL << 2)
#define MMCHS_CON_INIT				(1UL << 1)
#define MMCHS_CON_OD				(1UL << 0)




#endif /* _OMAP3_MMC_H_ */
