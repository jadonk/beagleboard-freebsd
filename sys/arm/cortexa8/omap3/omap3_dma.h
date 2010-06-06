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


/**
 * DMA device driver interface for the TI OMAP3530 SoC
 *
 * See the omap3_dma.c file for implementation details.
 *
 * Reference:
 *  OMAP35x Applications Processor
 *   Technical Reference Manual
 *  (omap35xx_techref.pdf)
 */
#ifndef _OMAP3_DMA_H_
#define _OMAP3_DMA_H_


#define DMA_ENDIAN_BIG			0x1
#define DMA_ENDIAN_LITTLE		0x0

#define DMA_BURST_NONE			0x0
#define DMA_BURST_16			0x1
#define DMA_BURST_32			0x2
#define DMA_BURST_64			0x3

#define DMA_DATA_8BITS_SCALAR	0x0
#define DMA_DATA_16BITS_SCALAR	0x1
#define DMA_DATA_32BITS_SCALAR	0x2

#define DMA_ADDR_CONSTANT		0x0
#define DMA_ADDR_POST_INCREMENT	0x1
#define DMA_ADDR_SINGLE_INDEX	0x2
#define DMA_ADDR_DOUBLE_INDEX	0x3


/**
 * Status flags for the DMA callback
 *
 */
#define DMA_STATUS_DROP					(1UL << 1) 
#define DMA_STATUS_HALF					(1UL << 2) 
#define DMA_STATUS_FRAME				(1UL << 3) 
#define DMA_STATUS_LAST					(1UL << 4) 
#define DMA_STATUS_BLOCK				(1UL << 5) 
#define DMA_STATUS_SYNC					(1UL << 6) 
#define DMA_STATUS_PKT					(1UL << 7) 
#define DMA_STATUS_TRANS_ERR			(1UL << 8) 
#define DMA_STATUS_SECURE_ERR			(1UL << 9) 
#define DMA_STATUS_SUPERVISOR_ERR		(1UL << 10) 
#define DMA_STATUS_MISALIGNED_ADRS_ERR	(1UL << 11) 
#define DMA_STATUS_DRAIN_END			(1UL << 12) 

#define DMA_SYNC_FRAME					(1UL << 0)
#define DMA_SYNC_BLOCK					(1UL << 1)
#define DMA_SYNC_PACKET					(DMA_SYNC_FRAME	| DMA_SYNC_BLOCK)
#define DMA_SYNC_TRIG_ON_SRC			(1UL << 8)
#define DMA_SYNC_TRIG_ON_DST			(1UL << 9)


#define DMA_IRQ_FLAG_DROP				(1UL << 1)
#define DMA_IRQ_FLAG_HALF_FRAME_COMPL	(1UL << 2)
#define DMA_IRQ_FLAG_FRAME_COMPL		(1UL << 3)
#define DMA_IRQ_FLAG_START_LAST_FRAME	(1UL << 4)
#define DMA_IRQ_FLAG_BLOCK_COMPL		(1UL << 5)
#define DMA_IRQ_FLAG_ENDOF_PKT			(1UL << 7)
#define DMA_IRQ_FLAG_DRAIN				(1UL << 12)


#define DMA_TRIGGER_SYS_DMA_REQ0	1 /* External DMA request 0 (system expansion) */
#define DMA_TRIGGER_SYS_DMA_REQ1	2 /* External DMA request 1 (system expansion) */
#define DMA_TRIGGER_GPMC_DMA		3 /* GPMC request from prefetch engine */
#define DMA_TRIGGER_DSS_LINE_TRIGGER	5 /* Display subsystem—frame update request */
#define DMA_TRIGGER_SYS_DMA_REQ2	6 /* External DMA request 2 (system expansion) */
#define DMA_TRIGGER_AES_1_DMA_TX	8 /* AES crypto-accelerator—transmit request */
#define DMA_TRIGGER_AES_1_DMA_RX	9 /* AES crypto-accelerator—receive request */
#define DMA_TRIGGER_DES_1_DMA_TX	10 /* DES/3DES crypto-accelerator—transmit request */
#define DMA_TRIGGER_DES_1_DMA_RX	11 /* DES/3DES crypto-accelerator—receive request */
#define DMA_TRIGGER_SHA2MD5_DMA_RX	12 /* SHA2-/MD5 crypto-accelerator—receive request */
#define DMA_TRIGGER_SPI3_DMA_TX0	14 /* McSPI module 3—transmit request channel 0 */
#define DMA_TRIGGER_SPI3_DMA_RX0	15 /* McSPI module 3—receive request channel 0 */
#define DMA_TRIGGER_MCBSP3_DMA_TX	16 /* MCBSP module 3—transmit request */
#define DMA_TRIGGER_MCBSP3_DMA_RX	17 /* MCBSP module 3—receive request */
#define DMA_TRIGGER_MCBSP4_DMA_TX	18 /* MCBSP module 4—transmit request */
#define DMA_TRIGGER_MCBSP4_DMA_RX	19 /* MCBSP module 4—receive request */
#define DMA_TRIGGER_MCBSP5_DMA_TX	20 /* MCBSP module 5—transmit request */
#define DMA_TRIGGER_MCBSP5_DMA_RX	21 /* MCBSP module 5—receive request */
#define DMA_TRIGGER_SPI3_DMA_TX1	22 /* McSPI module 3—transmit request channel 1 */
#define DMA_TRIGGER_SPI3_DMA_RX1	23 /* McSPI module 3—receive request channel 1 */
#define DMA_TRIGGER_I2C3_DMA_TX		24 /* I2C module 3—transmit request */
#define DMA_TRIGGER_I2C3_DMA_RX		25 /* I2C module 3—receive request */
#define DMA_TRIGGER_I2C1_DMA_TX		26 /* I2C module 1—transmit request */
#define DMA_TRIGGER_I2C1_DMA_RX		27 /* I2C module 1—receive request */
#define DMA_TRIGGER_I2C2_DMA_TX		28 /* I2C module 2—transmit request */
#define DMA_TRIGGER_I2C2_DMA_RX		29 /* I2C module 2—receive request */
#define DMA_TRIGGER_MCBSP1_DMA_TX	30 /* MCBSP module 1—transmit request */
#define DMA_TRIGGER_MCBSP1_DMA_RX	31 /* MCBSP module 1—receive request */
#define DMA_TRIGGER_MCBSP2_DMA_TX	32 /* MCBSP module 2—transmit request */
#define DMA_TRIGGER_MCBSP2_DMA_RX	33 /* MCBSP module 2—receive request */
#define DMA_TRIGGER_SPI1_DMA_TX0	34 /* McSPI module 1—transmit request channel 0 */
#define DMA_TRIGGER_SPI1_DMA_RX0	35 /* McSPI module 1—receive request channel 0 */
#define DMA_TRIGGER_SPI1_DMA_TX1	36 /* McSPI module 1—transmit request channel 1 */
#define DMA_TRIGGER_SPI1_DMA_RX1	37 /* McSPI module 1—receive request channel 1 */
#define DMA_TRIGGER_SPI1_DMA_TX2	38  /* McSPI module 1—transmit request channel 2 */
#define DMA_TRIGGER_SPI1_DMA_RX2	39 /* McSPI module 1—receive request channel 2 */
#define DMA_TRIGGER_SPI1_DMA_TX3	40 /* McSPI module 1—transmit request channel 3 */
#define DMA_TRIGGER_SPI1_DMA_RX3	41 /* McSPI module 1—receive request channel 3 */
#define DMA_TRIGGER_SPI2_DMA_TX0	42 /* McSPI module 2—transmit request channel 0 */
#define DMA_TRIGGER_SPI2_DMA_RX0	43 /* McSPI module 2—receive request channel 0 */
#define DMA_TRIGGER_SPI2_DMA_TX1	44 /* McSPI module 2—transmit request channel 1 */
#define DMA_TRIGGER_SPI2_DMA_RX1	45 /* McSPI module 2—receive request channel 1 */
#define DMA_TRIGGER_MMC2_DMA_TX		46 /* MMC/SD2 transmit request */
#define DMA_TRIGGER_MMC2_DMA_RX		47 /* MMC/SD2 receive request */
#define DMA_TRIGGER_UART1_DMA_TX	48 /* UART module 1—transmit request */
#define DMA_TRIGGER_UART1_DMA_RX	49 /* UART module 1—receive request */
#define DMA_TRIGGER_UART2_DMA_TX	50 /* UART module 2—transmit request */
#define DMA_TRIGGER_UART2_DMA_RX	51 /* UART module 2—receive request */
#define DMA_TRIGGER_UART3_DMA_TX	52 /* UART module 3—transmit request */
#define DMA_TRIGGER_UART3_DMA_RX	53 /* UART module 3—receive request */
#define DMA_TRIGGER_MMC1_DMA_TX		60	/* MMC/SD1 transmit request */
#define DMA_TRIGGER_MMC1_DMA_RX		61	/* MMC/SD1 receive request */
#define DMA_TRIGGER_MS_DMA			62	/* MS-PRO request */
#define DMA_TRIGGER_SYS_DMA_REQ3	63 /* External DMA request 3 (system expansion) */
#define DMA_TRIGGER_AES_2_DMA_TX	64 /* AES crypto-accelerator 2—transmit request */
#define DMA_TRIGGER_AES_2_DMA_RX	65 /* AES crypto-accelerator 2—receive request */
#define DMA_TRIGGER_DES_2_DMA_TX	66 /* DES/3DES crypto-accelerator 2—transmit request */
#define DMA_TRIGGER_DES_2_DMA_RX	67 /* DES/3DES crypto-accelerator 2—receive request */
#define DMA_TRIGGER_SHA1MD5_DMA_RX	68 /* SHA1/MD5 crypto-accelerator 2—receive request */
#define DMA_TRIGGER_SPI4_DMA_TX0	69 /* McSPI module 4—transmit request channel 0 */
#define DMA_TRIGGER_SPI4_DMA_RX0	70 /* McSPI module 4—receive request channel 0 */
#define DMA_TRIGGER_DSS_DMA0		71 /* Display subsystem DMA request 0 (DSI) */
#define DMA_TRIGGER_DSS_DMA1		72 /* Display subsystem DMA request 1 (DSI) */
#define DMA_TRIGGER_DSS_DMA2		73 /* Display subsystem DMA request 2 (DSI) */
#define DMA_TRIGGER_DSS_DMA3		74 /* Display subsystem DMA request 3 (DSI or RFBI) */
#define DMA_TRIGGER_MMC3_DMA_TX		76 /* MMC/SD3 transmit request */
#define DMA_TRIGGER_MMC3_DMA_RX		77 /* MMC/SD3 receive request */
#define DMA_TRIGGER_USIM_DMA_TX		78 /* USIM transmit request */
#define DMA_TRIGGER_USIM_DMA_RX		79 /* USIM receive request */





int
omap3_dma_activate_channel(unsigned int *ch,
						   void (*callback)(unsigned int ch, uint32_t status, void *data),
						   void *data);

int
omap3_dma_deactivate_channel(unsigned int ch);

int
omap3_dma_start_xfer(unsigned int ch, unsigned int src_paddr,
					 unsigned long dst_paddr,
					 unsigned int frmcnt, unsigned int elmcnt);

int
omap3_dma_start_xfer_packet(unsigned int ch, unsigned int src_paddr,
					        unsigned long dst_paddr, unsigned int frmcnt,
							unsigned int elmcnt, unsigned int pktsize);

int
omap3_dma_stop_xfer(unsigned int ch);

int
omap3_dma_enable_channel_irq(unsigned int ch, uint32_t flags);

int
omap3_dma_disable_channel_irq(unsigned int ch);

int
omap3_dma_get_channel_status(unsigned int ch, uint32_t *status);


int
omap3_dma_set_xfer_endianess(unsigned int ch, unsigned int src, unsigned int dst);

int
omap3_dma_set_xfer_burst(unsigned int ch, unsigned int src, unsigned int dst);

int
omap3_dma_set_xfer_data_type(unsigned int ch, unsigned int type);

int
omap3_dma_set_callback(unsigned int ch,
					   void (*callback)(unsigned int ch, uint32_t status, void *data),
					   void *data);

int
omap3_dma_sync_params(unsigned int ch, unsigned int trigger, unsigned int mode);

int
omap3_dma_set_addr_mode(unsigned int ch, unsigned int src_mode,
						unsigned int dst_mode);


#endif /* _OMAP3_DMA_H_ */
