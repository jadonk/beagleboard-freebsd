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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/interrupt.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>
#include <sys/timetc.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_dma.h>

/**
 * Kernel functions for using the DMA controller
 *
 *
 * DMA TRANSFERS:
 *   A DMA transfer block consists of a number of frames (FN). Each frame
 * consists of a number of elements, and each element can have a size of 8, 16,
 * or 32 bits.
 * 
 */


/**
 * The number of DMA channels possible on the controller.
 */
#define NUM_DMA_CHANNELS						32


/**
 * Various register field settings
 */
#define DMA4_CSDP_DATA_TYPE(x)					(((x) & 0x3) << 0) 
#define DMA4_CSDP_SRC_BURST_MODE(x)				(((x) & 0x3) << 7) 
#define DMA4_CSDP_DST_BURST_MODE(x)				(((x) & 0x3) << 14) 
#define DMA4_CSDP_SRC_ENDIANISM(x)				(((x) & 0x1) << 21) 
#define DMA4_CSDP_DST_ENDIANISM(x)				(((x) & 0x1) << 19) 
#define DMA4_CSDP_WRITE_MODE(x)					(((x) & 0x3) << 16) 
#define DMA4_CSDP_SRC_PACKED(x)					(((x) & 0x1) << 6) 
#define DMA4_CSDP_DST_PACKED(x)					(((x) & 0x1) << 13) 

#define DMA4_CCR_DST_ADDRESS_MODE(x)			(((x) & 0x3) << 14) 
#define DMA4_CCR_SRC_ADDRESS_MODE(x)			(((x) & 0x3) << 12) 
#define DMA4_CCR_READ_PRIORITY(x)				(((x) & 0x1) << 6) 
#define DMA4_CCR_WRITE_PRIORITY(x)				(((x) & 0x1) << 26) 
#define DMA4_CCR_SYNC_TRIGGER(x)				((((x) & 0x60) << 14) \
												 | ((x) & 0x1f))
#define	DMA4_CCR_FRAME_SYNC(x)					(((x) & 0x1) << 5)
#define	DMA4_CCR_BLOCK_SYNC(x)					(((x) & 0x1) << 18)
#define DMA4_CCR_SEL_SRC_DST_SYNC(x)			(((x) & 0x1) << 24)

#define DMA4_CCR_PACKET_TRANS					(DMA4_CCR_FRAME_SYNC(1) | \
												 DMA4_CCR_BLOCK_SYNC(1) )

#define DMA4_CSR_DROP							(1UL << 1) 
#define DMA4_CSR_HALF							(1UL << 2) 
#define DMA4_CSR_FRAME							(1UL << 3) 
#define DMA4_CSR_LAST							(1UL << 4) 
#define DMA4_CSR_BLOCK							(1UL << 5) 
#define DMA4_CSR_SYNC							(1UL << 6) 
#define DMA4_CSR_PKT							(1UL << 7) 
#define DMA4_CSR_TRANS_ERR						(1UL << 8) 
#define DMA4_CSR_SECURE_ERR						(1UL << 9) 
#define DMA4_CSR_SUPERVISOR_ERR					(1UL << 10) 
#define DMA4_CSR_MISALIGNED_ADRS_ERR			(1UL << 11) 
#define DMA4_CSR_DRAIN_END						(1UL << 12) 
#define DMA4_CSR_CLEAR_MASK						(0xffe)

#define DMA4_CICR_DROP_IE						(1UL << 1) 
#define DMA4_CICR_HALF_IE						(1UL << 2) 
#define DMA4_CICR_FRAME_IE						(1UL << 3) 
#define DMA4_CICR_LAST_IE						(1UL << 4) 
#define DMA4_CICR_BLOCK_IE						(1UL << 5) 
#define DMA4_CICR_PKT_IE						(1UL << 7) 
#define DMA4_CICR_TRANS_ERR_IE					(1UL << 8) 
#define DMA4_CICR_SECURE_ERR_IE					(1UL << 9) 
#define DMA4_CICR_SUPERVISOR_ERR_IE				(1UL << 10) 
#define DMA4_CICR_MISALIGNED_ADRS_ERR_IE		(1UL << 11) 
#define DMA4_CICR_DRAIN_IE						(1UL << 12) 




/**
 *	Data structure per DMA channel. 
 *
 *
 */
struct omap3_dma_channel {
	
	/* The configuration registers for the given channel, these are modified
	 * by the set functions and only written to the actual registers when a
	 * transaction is started.
	 */
	uint32_t			reg_csdp;
	uint32_t			reg_ccr;
	uint32_t			reg_cicr;
	
	/* Set when one of the configuration registers above change */
	uint32_t			need_reg_write;
	
	/* Callback function used when an interrupt is tripped on the given channel */
	void (*callback)(unsigned int ch, uint32_t ch_status, void *data);
	
	/* Callback data passed in the callback ... duh */
	void*				callback_data;
	
};

/**
 *	DMA driver context, allocated and stored globally, this driver is not
 *	intetned to ever be unloaded (see g_omap3_dma_sc).
 *
 */
struct omap3_dma_softc {
	bus_space_tag_t		sc_iotag;
	bus_space_handle_t	sc_ioh;
	device_t			sc_dev;
	struct resource*	sc_irq;
	
	/* I guess in theory we should have a mutex per DMA channel for register
	 * modifications. But since we know we are never going to be run on a SMP
	 * system, we can use just the single lock for all channels.
	 */
	struct mtx			sc_mtx;

	/* Bits in the sc_active_channels data field indicate if the channel has
	 * been activated.
	 */
	uint32_t			sc_active_channels;
	
	struct omap3_dma_channel
						sc_channel[NUM_DMA_CHANNELS];
	
};

static struct omap3_dma_softc *g_omap3_dma_sc = NULL;


/**
 *	Function prototypes
 *
 */
static void omap3_dma_intr(void *);




/**
 *	omap3_dma_readl - reads a 32-bit value from one of the DMA registers
 *	@sc: DMA device context
 *	@off: The offset of a register from the DMA register address range
 *
 *
 *	RETURNS:
 *	32-bit value read from the register.
 */
static inline uint32_t
omap3_dma_readl(struct omap3_dma_softc *sc, bus_size_t off)
{
	return bus_space_read_4(sc->sc_iotag, sc->sc_ioh, off);
}

/**
 *	omap3_dma_writel - writes a 32-bit value to one of the DMA registers
 *	@sc: DMA device context
 *	@off: The offset of a register from the DMA register address range
 *
 *
 *	RETURNS:
 *	32-bit value read from the register.
 */
static inline void
omap3_dma_writel(struct omap3_dma_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_iotag, sc->sc_ioh, off, val);
}






/**
 *	omap3_dma_test - test function, to be removed
 *
 *	Simple test function use to verify the DMA behaviour
 *
 *	RETURNS:
 *	32-bit value read from the register.
 */
#if 0
static void
omap3_dma_test(void)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	unsigned int ch;
	uint8_t *src;
	uint8_t *dst;
	vm_paddr_t src_pa;
	vm_paddr_t dst_pa;
	unsigned int timeout = 0;
	uint32_t status = 0;
	
	src = malloc(PAGE_SIZE, M_DEVBUF, M_NOWAIT);
	dst = malloc(PAGE_SIZE, M_DEVBUF, M_NOWAIT);
	
	src_pa = pmap_extract(pmap_kernel(), (vm_offset_t)src);
	dst_pa = pmap_extract(pmap_kernel(), (vm_offset_t)dst);
	
	printf("[BRG] %s : %d : src=%p [PA: 0x%08x]\n", __func__, __LINE__, src, src_pa);
	printf("[BRG] %s : %d : dst=%p [PA: 0x%08x]\n", __func__, __LINE__, dst, dst_pa);
	
	
	if (omap3_dma_activate_channel(&ch, NULL, NULL) != 0) {
		printf("[BRG] %s : %d : omap3_dma_activate_channel failed\n", __func__, __LINE__);
		goto err_out;
	}
	
	printf("[BRG] %s : %d : activated channel number %u\n", __func__, __LINE__, ch);
	
//	if (omap3_dma_sync_params(ch, 0, DMA_SYNC_BLOCK) != 0) {
//		printf("[BRG] %s : %d : omap3_dma_sync_params failed\n", __func__, __LINE__);
//		goto out;
//	}
	
	
	if (omap3_dma_enable_channel_irq(ch, DMA_IRQ_FLAG_BLOCK_COMPL) != 0) {
		printf("[BRG] %s : %d : omap3_dma_enable_channel_irq failed\n", __func__, __LINE__);
		goto out;
	}
	
	
	if (omap3_dma_start_xfer(ch, src_pa, dst_pa, (PAGE_SIZE / 4), 1) != 0) {
		printf("[BRG] %s : %d : omap3_dma_start_xfer failed\n", __func__, __LINE__);
		goto out;
	}
	
	
	
	
	
	while (status == 0 && timeout++ < 10000) {
		if (omap3_dma_get_channel_status(ch, &status) != 0) {
			printf("[BRG] %s : %d : omap3_dma_get_channel_status failed\n", __func__, __LINE__);
			break;
		}
		
		if ((timeout % 10) == 0) {
			printf("[BRG] : DMA4_CSAC 0x%08x : DMA4_CDAC 0x%08x : DMA4_CSR 0x%08x\n",
				   omap3_dma_readl(sc, OMAP35XX_SDMA_CSAC(ch)),
			       omap3_dma_readl(sc, OMAP35XX_SDMA_CDAC(ch)),
				   status);
		}
	}
	
	printf("[BRG] %s : %d : status 0x%04x : timeout %d\n", __func__, __LINE__, status, timeout);
	
out:
	if (omap3_dma_deactivate_channel(ch) != 0) {
		printf("[BRG] %s : %d : omap3_dma_deactivate_channel failed\n", __func__, __LINE__);
	}
	
err_out:
	free(src, M_DEVBUF);
	free(dst, M_DEVBUF);
}
#endif


/**
 *	omap3_dma_dbg_dump_regs - test function, to be removed
 *	@arg: 
 *
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
#if 0
static void
omap3_dma_dbg_dump_regs(unsigned int ch)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	
	/*
	 "\tDMA4_SYSSTATUS   0x%08x\n"
	 "\tDMA4_OCP_SYSCONFIG   0x%08x\n"
	 "\tDMA4_CAPS_0    0x%08x\n"
	 "\tDMA4_CAPS_2    0x%08x\n"
	 "\tDMA4_CAPS_3    0x%08x\n"
	 "\tDMA4_CAPS_4    0x%08x\n"
	 "\tDMA4_GCR       0x%08x\n"
	 "\tDMA4_CCR       0x%08x\n"
	 */
	
	printf("DMA register set (channel %u):\n"
		   "\tDMA4_IRQSTATUS_L 0x%08x 0x%08x 0x%08x 0x%08x\n"
		   "\tDMA4_IRQENABLE_L 0x%08x 0x%08x 0x%08x 0x%08x\n"
		   "\tDMA4_CCR       0x%08x\n"
		   "\tDMA4_CLNK_CTRL 0x%08x\n"
		   "\tDMA4_CICR      0x%08x\n"
		   "\tDMA4_CSR       0x%08x\n"
		   "\tDMA4_CSDP      0x%08x\n"
		   "\tDMA4_CEN       0x%08x\n"
		   "\tDMA4_CFN       0x%08x\n"
		   "\tDMA4_CSSA      0x%08x\n"
		   "\tDMA4_CDSA      0x%08x\n"
		   "\tDMA4_CSE       0x%08x\n"
		   "\tDMA4_CSF       0x%08x\n"
		   "\tDMA4_CDE       0x%08x\n"
		   "\tDMA4_CDF       0x%08x\n"
		   "\tDMA4_CSAC      0x%08x\n"
		   "\tDMA4_CDAC      0x%08x\n"
		   "\tDMA4_CCEN      0x%08x\n"
		   "\tDMA4_CCFN      0x%08x\n"
		   "\tDMA4_COLOR     0x%08x\n",
		   ch,
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQSTATUS_L(0)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQSTATUS_L(1)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQSTATUS_L(2)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQSTATUS_L(3)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(0)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(1)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(2)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(3)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CCR(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CLNK_CTRL(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CICR(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSR(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSDP(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CEN(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CFN(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSSA(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CDSA(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSE(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSF(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CDE(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CDF(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CSAC(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CDAC(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CCEN(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_CCFN(ch)),
		   omap3_dma_readl(sc, OMAP35XX_SDMA_COLOR(ch))
		   );
	
}
#endif




/**
 *	omap3_dma_probe - driver probe function
 *	@dev: dma device handle
 *
 *	
 *
 *	RETURNS:
 *	Always returns 0.
 */
static int
omap3_dma_probe(device_t dev)
{
	
	device_set_desc(dev, "TI OMAP3 DMA Controller");
	return (0);
}


/**
 *	omap3_dma_attach - driver attach function
 *	@dev: dma device handle
 *
 *	Initialises memory mapping/pointers to the DMA register set and requests
 *	IRQs. This is effectively the setup function for the driver.
 *
 *	RETURNS:
 *	0 on success or a negative error code failure.
 */
static int
omap3_dma_attach(device_t dev)
{
	struct omap3_dma_softc *sc = device_get_softc(dev);
	unsigned int i = 0;
	uint32_t val;
	int      rid = 0;
	void    *ihl;
	int      err;
	
	
	/* Setup the basics */
	sc->sc_dev = dev;
	sc->sc_iotag = &omap3_bs_tag;
	
	/* No channels active at the moment */
	sc->sc_active_channels = 0x00000000;
	
	
	/* Mutex to protect the shared data structures */
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "omap3_dma", MTX_SPIN);
	
	/* Map in the DMA controller register set */
	if (bus_space_map(sc->sc_iotag, OMAP35XX_SDMA_HWBASE, OMAP35XX_SDMA_SIZE, 0,
					  &sc->sc_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}
	
	
	/* For the DMA module there are no interface or functional clocks that need
	 * to be turned on ... I think.
	 */
	
	/* Setup configuration */
	val = omap3_dma_readl(sc, OMAP35XX_SDMA_GCR);
	
	/* Disable all interrupts */
	for (i=0; i<4; i++) {
		omap3_dma_writel(sc, OMAP35XX_SDMA_IRQENABLE_L(i),
						  0x00000000);
	}
	
	/* Do a soft-reset */
	omap3_dma_writel(sc, OMAP35XX_SDMA_OCP_SYSCONFIG, 0x0002);
	/* TODO: Add a timeout */
	while ((omap3_dma_readl(sc, OMAP35XX_SDMA_SYSSTATUS) & 0x1) == 0x0)
		continue;
	
	
	/* Install interrupt handlers for the for possible interrupts. Any channel
	 * can trip one of the four IRQs
	 */
	sc->sc_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, OMAP35XX_IRQ_SDMA0,
									OMAP35XX_IRQ_SDMA3, 4, RF_ACTIVE);
	if (sc->sc_irq == NULL)
		panic("Unable to setup the clock irq handler.\n");
	
	err = bus_setup_intr(dev, sc->sc_irq, INTR_TYPE_MISC | INTR_MPSAFE, NULL,
				         omap3_dma_intr, NULL, &ihl);
	if (err) {
		panic("%s: Cannot register IRQ", device_get_name(dev));
	}
	
	
	
	/* Store the DMA structure globally ... this driver should never be unloaded */
	g_omap3_dma_sc = sc;
	
	
	/* DEBUG : omap3_dma_test(); */
	
	
	return (0);
}


static device_method_t g_omap3_dma_methods[] = {
	DEVMETHOD(device_probe, omap3_dma_probe),
	DEVMETHOD(device_attach, omap3_dma_attach),
	{0, 0},
};

static driver_t g_omap3_dma_driver = {
	"omap3_dma",
	g_omap3_dma_methods,
	sizeof(struct omap3_dma_softc),
};
static devclass_t g_omap3_dma_devclass;

DRIVER_MODULE(omap3_dma, omap3, g_omap3_dma_driver, g_omap3_dma_devclass, 0, 0);





/**
 *	omap3_dma_intr - interrupt handler for all 4 DMA IRQs
 *	@arg: ignored
 *
 *	Called when any of the four DMA IRQs are triggered.
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_dma_intr(void *arg)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t intr;
	uint32_t csr;
	unsigned int ch, j;
	struct omap3_dma_channel* channel;
	
	mtx_lock(&sc->sc_mtx);
	
	for (j=0; j<4; j++) {

		/* Get the flag interrupts (enabled) */
		intr = omap3_dma_readl(sc,OMAP35XX_SDMA_IRQSTATUS_L(j));
		intr &= omap3_dma_readl(sc,OMAP35XX_SDMA_IRQENABLE_L(j));
		if (intr == 0x00000000)
			continue;
		
		
		/* Loop through checking the status bits */
		for (ch=0; ch<32; ch++) {
			if (intr & (1 << ch)) {
				channel = &sc->sc_channel[ch];
				
				/* Read the CSR regsiter and verify we don't have a spurious IRQ */
				csr = omap3_dma_readl(sc, OMAP35XX_SDMA_CSR(ch));
				if (csr == 0) {
					printf("Spurious DMA IRQ for channel %d\n", ch);
					continue;
				}
				
				/* Sanity check this channel is active */
				if ((sc->sc_active_channels & (1 << ch)) == 0) {
					printf("IRQ %d for a non-activated channel %d\n",
						   j, ch);
					continue;
				}
				
				/* Check the status error codes */
				if (csr & DMA4_CSR_DROP)
					printf("Synchronization event drop occurred during the"
						   "transfer on channel %u", ch);
				if (csr & DMA4_CSR_SECURE_ERR)
					printf("Secure transaction error event on channel %u", ch);
				if (csr & DMA4_CSR_MISALIGNED_ADRS_ERR)
					printf("Misaligned address error event on channel %u", ch);
				if (csr & DMA4_CSR_TRANS_ERR) {
					printf("Transaction error event on channel %u", ch);
					/* Apparently according to linux code, there is an errata
					 * that says the channel is not disabled upon this error.
					 * They explicitly disable the channel here .. since I
					 * haven't seen the errata, I'm going to ignore for now.
					 */
				}
				
				/* Clear the status flags for the IRQ */
				omap3_dma_writel(sc, OMAP35XX_SDMA_CSR(ch), DMA4_CSR_CLEAR_MASK);
				omap3_dma_writel(sc, OMAP35XX_SDMA_IRQSTATUS_L(j), (1 << ch));

				/* Call the callback for the given channel */
				if (channel->callback)
					channel->callback(ch, csr, channel->callback_data);
				
			}
		}
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return;
}



/**
 *	omap3_dma_activate_channel - driver attach function
 *	@dev: dma device handle
 *
 *	Note this function doesn't enable interrupts, for that you need to call
 *	omap3_dma_enable_channel_irq(). If not using IRQ to detect the end of the
 *	transfer, you can use omap3_dma_status_poll() to detect a change in the
 *	status.
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	0 on success, otherwise an error code
 */
int
omap3_dma_activate_channel(unsigned int *ch,
						   void (*callback)(unsigned int ch, uint32_t status, void *data),
						   void *data)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	struct omap3_dma_channel *channel = NULL;
	uint32_t adr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	if (ch == NULL)
		return (EINVAL);
	
	mtx_lock(&sc->sc_mtx);

	/* Check to see if all channels are in use */
	if (sc->sc_active_channels == 0xffffffff) {
		mtx_unlock(&sc->sc_mtx);
		return (ENOMEM);
	}
		
	/* Find the first non-active channel */
	*ch = ffs(sc->sc_active_channels);
		
	/* Mark the channel as active */
	sc->sc_active_channels |= (1 << *ch);
	
	
	
	/* Get the channel struct and populate the fields */
	channel = &sc->sc_channel[*ch];
	
	channel->callback = callback;
	channel->callback_data = data;
	
	channel->need_reg_write = 1;

	/* Set the default configuration for the DMA channel */
	channel->reg_csdp = DMA4_CSDP_DATA_TYPE(0x2)
		| DMA4_CSDP_SRC_BURST_MODE(0)
		| DMA4_CSDP_DST_BURST_MODE(0)
		| DMA4_CSDP_SRC_ENDIANISM(0)
		| DMA4_CSDP_DST_ENDIANISM(0)
		| DMA4_CSDP_WRITE_MODE(0)
		| DMA4_CSDP_SRC_PACKED(0)
		| DMA4_CSDP_DST_PACKED(0);
	
	channel->reg_ccr = DMA4_CCR_DST_ADDRESS_MODE(1)
		| DMA4_CCR_SRC_ADDRESS_MODE(1)
		| DMA4_CCR_READ_PRIORITY(0)
		| DMA4_CCR_WRITE_PRIORITY(0)
		| DMA4_CCR_SYNC_TRIGGER(0)
		| DMA4_CCR_FRAME_SYNC(0)
		| DMA4_CCR_BLOCK_SYNC(0);
	
	channel->reg_cicr = DMA4_CICR_TRANS_ERR_IE
		| DMA4_CICR_SECURE_ERR_IE
		| DMA4_CICR_SUPERVISOR_ERR_IE
		| DMA4_CICR_MISALIGNED_ADRS_ERR_IE;
	
	
	
	/* Clear all the channel registers, this should abort any transaction */
	for (adr = OMAP35XX_SDMA_CCR(*ch); adr <= OMAP35XX_SDMA_COLOR(*ch); adr += 4)
		omap3_dma_writel(sc, adr, 0x00000000);
	
	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}


/**
 *	omap3_dma_deactivate_channel - driver attach function
 *	@dev: dma device handle
 *
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_deactivate_channel(unsigned int ch)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	unsigned int j;
	unsigned int adr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	
	mtx_lock(&sc->sc_mtx);
	
	/* First check if the channel is currently active */
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EBUSY);
	}
	
	/* Mark the channel as inactive */
	sc->sc_active_channels &= ~(1 << ch);
	
	
	/* Disable all DMA interrupts for the channel. */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CICR(ch), 0);
	
	/* Make sure the DMA transfer is stopped. */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch), 0);

	
	/* Clear the CSR register and IRQ status register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSR(ch), DMA4_CSR_CLEAR_MASK);
	for (j=0; j<4; j++) {
		omap3_dma_writel(sc, OMAP35XX_SDMA_IRQSTATUS_L(j), (1 << ch));
	}
	

	
	/* Clear all the channel registers, this should abort any transaction */
	for (adr = OMAP35XX_SDMA_CCR(ch); adr <= OMAP35XX_SDMA_COLOR(ch); adr += 4)
		omap3_dma_writel(sc, adr, 0x00000000);
	
	
	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}


/**
 *	omap3_dma_disable_channel_irq - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@src_paddr: the source phsyical address
 *	@dst_paddr: the destination phsyical address
 *	@elmcnt: the number of elements in a frame, an element is either an 8, 16
 *           or 32-bit value as defined by omap3_dma_set_xfer_burst()
 *	@frmcnt: the number of frames per block
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_disable_channel_irq(unsigned int ch)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t irq_enable;
	unsigned int j;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}

	/* Disable all the individual error conditions */
	sc->sc_channel[ch].reg_cicr = 0x0000;
	omap3_dma_writel(sc, OMAP35XX_SDMA_CICR(ch), 0x0000);
	
	/* Disable the channel interrupt enable */
	for (j=0; j<4; j++) {
		irq_enable = omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(j));
		irq_enable &= ~(1 << ch);
	
		omap3_dma_writel(sc, OMAP35XX_SDMA_IRQENABLE_L(j), irq_enable);
	}
	
	/* Indicate the registers need to be rewritten on the next transaction */
	sc->sc_channel[ch].need_reg_write = 1;
	
	mtx_unlock(&sc->sc_mtx);
	
	return (0);
}

/**
 *	omap3_dma_enable_channel_irq - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@src_paddr: the source phsyical address
 *	@dst_paddr: the destination phsyical address
 *	@elmcnt: the number of elements in a frame, an element is either an 8, 16
 *           or 32-bit value as defined by omap3_dma_set_xfer_burst()
 *	@frmcnt: the number of frames per block
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_enable_channel_irq(unsigned int ch, uint32_t flags)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t irq_enable;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	/* Always enable the error interrupts if we have interrupts enabled */
	flags |= DMA4_CICR_TRANS_ERR_IE | DMA4_CICR_SECURE_ERR_IE |
	         DMA4_CICR_SUPERVISOR_ERR_IE | DMA4_CICR_MISALIGNED_ADRS_ERR_IE;

	sc->sc_channel[ch].reg_cicr = flags;

	/* Write the values to the register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CICR(ch), flags);
	
	
	/* Enable the channel interrupt enable */
	irq_enable = omap3_dma_readl(sc, OMAP35XX_SDMA_IRQENABLE_L(0));
	irq_enable |= (1 << ch);
	
	omap3_dma_writel(sc, OMAP35XX_SDMA_IRQENABLE_L(0), irq_enable);
	
	
	/* Indicate the registers need to be rewritten on the next transaction */
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);

	return (0);
}


/**
 *	omap3_dma_get_channel_status - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@src_paddr: the source phsyical address
 *	@dst_paddr: the destination phsyical address
 *	@elmcnt: the number of elements in a frame, an element is either an 8, 16
 *           or 32-bit value as defined by omap3_dma_set_xfer_burst()
 *	@frmcnt: the number of frames per block
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_get_channel_status(unsigned int ch, uint32_t *status)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t csr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	mtx_unlock(&sc->sc_mtx);

	csr = omap3_dma_readl(sc, OMAP35XX_SDMA_CSR(ch));
	
	if (status != NULL)
		*status = csr;
	
	return (0);
}


/**
 *	omap3_dma_start_xfer - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@src_paddr: the source phsyical address
 *	@dst_paddr: the destination phsyical address
 *	@elmcnt: the number of elements in a frame, an element is either an 8, 16
 *           or 32-bit value as defined by omap3_dma_set_xfer_burst()
 *	@frmcnt: the number of frames per block
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_start_xfer(unsigned int ch, unsigned int src_paddr,
					 unsigned long dst_paddr,
					 unsigned int frmcnt, unsigned int elmcnt)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	struct omap3_dma_channel *channel;
	uint32_t ccr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);

	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	channel = &sc->sc_channel[ch];
		
	/* a) Write the CSDP register */
	//if (channel->need_reg_write)
		omap3_dma_writel(sc, OMAP35XX_SDMA_CSDP(ch),
						  channel->reg_csdp | DMA4_CSDP_WRITE_MODE(1));
	
	
	/* b) Set the number of element per frame CEN[23:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CEN(ch), elmcnt);

	/* c) Set the number of frame per block CFN[15:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CFN(ch), frmcnt);
	
	/* d) Set the Source/dest start address index CSSA[31:0]/CDSA[31:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSSA(ch), src_paddr);
	omap3_dma_writel(sc, OMAP35XX_SDMA_CDSA(ch), dst_paddr);
	

	/* e) Write the CCR register */
	//if (channel->need_reg_write)
		omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch), channel->reg_ccr);
	
	
	/* f)  - Set the source element index increment CSEI[15:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSE(ch), 0x0001);
	
	/*     - Set the source frame index increment CSFI[15:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSF(ch), 0x0001);
	
	/*     - Set the destination element index increment CDEI[15:0]*/
	omap3_dma_writel(sc, OMAP35XX_SDMA_CDE(ch), 0x0001);
	
	/* - Set the destination frame index increment CDFI[31:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CDF(ch), 0x0001);

	
	
	/* Clear the status register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSR(ch), 0x1FFE);
	
	/* DEBUG: omap3_dma_dbg_dump_regs(ch); */

	/* Write the start-bit and away we go */
	ccr = omap3_dma_readl(sc, OMAP35XX_SDMA_CCR(ch));
	ccr |= (1 << 7);
	omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch), ccr);

	
	/* Clear the reg write flag */
	channel->need_reg_write = 0;

	mtx_unlock(&sc->sc_mtx);
	
	return (0);
}


/**
 *	omap3_dma_start_xfer_packet - driver attach function
 *	@ch: the channel number to use for the transfer
 *	@src_paddr: the source physical address
 *	@dst_paddr: the destination physical address
 *	@frmcnt: the number of frames to transfer
 *	@elmcnt: the number of elements in a frame, an element is either an 8, 16
 *           or 32-bit value as defined by omap3_dma_set_xfer_burst()
 *	@pktsize: the number of elements in each transfer packet
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_start_xfer_packet(unsigned int ch, unsigned int src_paddr,
					        unsigned long dst_paddr, unsigned int frmcnt,
							unsigned int elmcnt, unsigned int pktsize)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	struct omap3_dma_channel *channel;
	uint32_t ccr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	channel = &sc->sc_channel[ch];
	
	/* a) Write the CSDP register */
	if (channel->need_reg_write)
		omap3_dma_writel(sc, OMAP35XX_SDMA_CSDP(ch),
						 channel->reg_csdp | DMA4_CSDP_WRITE_MODE(1));
	
	
	/* b) Set the number of elements to transfer CEN[23:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CEN(ch), elmcnt);
	
	/* c) Set the number of frames to transfer CFN[15:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CFN(ch), frmcnt);
	
	/* d) Set the Source/dest start address index CSSA[31:0]/CDSA[31:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSSA(ch), src_paddr);
	omap3_dma_writel(sc, OMAP35XX_SDMA_CDSA(ch), dst_paddr);
	
	
	/* e) Write the CCR register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch),
					 channel->reg_ccr | DMA4_CCR_PACKET_TRANS);
	
	/* f)  - Set the source element index increment CSEI[15:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSE(ch), 0x0001);
	
	/*     - Set the packet size, this is dependent on the sync source */
	if (channel->reg_ccr & DMA4_CCR_SEL_SRC_DST_SYNC(1))
		omap3_dma_writel(sc, OMAP35XX_SDMA_CSF(ch), pktsize);
	else
		omap3_dma_writel(sc, OMAP35XX_SDMA_CDE(ch), pktsize);
	
	/* - Set the destination frame index increment CDFI[31:0] */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CDF(ch), 0x0001);
	
	
	
	/* Clear the status register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSR(ch), 0x1FFE);
	
	/* DEBUG omap3_dma_dbg_dump_regs(ch); */
	
	/* Write the start-bit and away we go */
	ccr = omap3_dma_readl(sc, OMAP35XX_SDMA_CCR(ch));
	ccr |= (1 << 7);
	omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch), ccr);
	
	
	/* Clear the reg write flag */
	channel->need_reg_write = 0;
	
	mtx_unlock(&sc->sc_mtx);
	
	return (0);
}


/**
 *	omap3_dma_stop_xfer - stops any currently active transfers
 *	@ch: the channel number to set the endianess of
 *
 *	This function call is effectively a NOP if no transaction is in progress.
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_stop_xfer(unsigned int ch)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	unsigned int j;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}

	
	/* Disable all DMA interrupts for the channel. */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CICR(ch), 0);
	
	/* Make sure the DMA transfer is stopped. */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CCR(ch), 0);
	
	
	/* Clear the CSR register and IRQ status register */
	omap3_dma_writel(sc, OMAP35XX_SDMA_CSR(ch), DMA4_CSR_CLEAR_MASK);
	for (j=0; j<4; j++) {
		omap3_dma_writel(sc, OMAP35XX_SDMA_IRQSTATUS_L(j), (1 << ch));
	}
	
	/* Configuration registers need to be re-written on the next xfer */
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);
	
	return (0);
}


/**
 *	omap3_dma_set_xfer_endianess - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@src: the source endianess (either DMA_ENDIAN_LITTLE or DMA_ENDIAN_BIG)
 *	@dst: the destination endianess (either DMA_ENDIAN_LITTLE or DMA_ENDIAN_BIG)
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_set_xfer_endianess(unsigned int ch, unsigned int src, unsigned int dst)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;

	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	sc->sc_channel[ch].reg_csdp &= ~DMA4_CSDP_SRC_ENDIANISM(1);
	sc->sc_channel[ch].reg_csdp |= DMA4_CSDP_SRC_ENDIANISM(src);

	sc->sc_channel[ch].reg_csdp &= ~DMA4_CSDP_DST_ENDIANISM(1);
	sc->sc_channel[ch].reg_csdp |= DMA4_CSDP_DST_ENDIANISM(dst);
	
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);

	return 0;
}


/**
 *	omap3_dma_set_xfer_burst - sets the source and destination burst settings
 *	@ch: the channel number to set the burst settings of
 *	@src: the source endianess (either DMA_BURST_NONE, DMA_BURST_16, DMA_BURST_32
 *	      or DMA_BURST_64)
 *	@dst: the destination endianess (either DMA_BURST_NONE, DMA_BURST_16,
 *	      DMA_BURST_32 or DMA_BURST_64)
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_set_xfer_burst(unsigned int ch, unsigned int src, unsigned int dst)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	sc->sc_channel[ch].reg_csdp &= ~DMA4_CSDP_SRC_BURST_MODE(0x3);
	sc->sc_channel[ch].reg_csdp |= DMA4_CSDP_SRC_BURST_MODE(src);
	
	sc->sc_channel[ch].reg_csdp &= ~DMA4_CSDP_DST_BURST_MODE(0x3);
	sc->sc_channel[ch].reg_csdp |= DMA4_CSDP_DST_BURST_MODE(dst);
	
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);

	return 0;
}


/**
 *	omap3_dma_set_xfer_data_type - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@type: the xfer data type (either DMA_DATA_8BITS_SCALAR, DMA_DATA_16BITS_SCALAR
 *	       or DMA_DATA_32BITS_SCALAR)
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_set_xfer_data_type(unsigned int ch, unsigned int type)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	sc->sc_channel[ch].reg_csdp &= ~DMA4_CSDP_DATA_TYPE(0x3);
	sc->sc_channel[ch].reg_csdp |= DMA4_CSDP_DATA_TYPE(type);
	
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}


/**
 *	omap3_dma_set_callback - driver attach function
 *	@dev: dma device handle
 *
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_set_callback(unsigned int ch,
					   void (*callback)(unsigned int ch, uint32_t status, void *data),
					   void *data)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	sc->sc_channel[ch].callback = callback;
	sc->sc_channel[ch].callback_data = data;
	
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}


/**
 *	omap3_dma_set_callback - driver attach function
 *	@dev: dma device handle
 *
 *	
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_sync_params(unsigned int ch, unsigned int trigger, unsigned int mode)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t ccr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	ccr = sc->sc_channel[ch].reg_ccr;
	
	ccr &= ~DMA4_CCR_SYNC_TRIGGER(0x7F);
	ccr |= DMA4_CCR_SYNC_TRIGGER(trigger + 1);
	
	if (mode & DMA_SYNC_FRAME)
		ccr |= DMA4_CCR_FRAME_SYNC(1);
	else
		ccr &= ~DMA4_CCR_FRAME_SYNC(1);
	
	if (mode & DMA_SYNC_BLOCK)
		ccr |= DMA4_CCR_BLOCK_SYNC(1);
	else
		ccr &= ~DMA4_CCR_BLOCK_SYNC(1);
	
	if (mode & DMA_SYNC_TRIG_ON_SRC)
		ccr |= DMA4_CCR_SEL_SRC_DST_SYNC(1);
	else
		ccr &= ~DMA4_CCR_SEL_SRC_DST_SYNC(1);
	
	sc->sc_channel[ch].reg_ccr = ccr;

	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}



/**
 *	omap3_dma_set_addr_mode - driver attach function
 *	@ch: the channel number to set the endianess of
 *	@rd_mode: the xfer source addressing mode (either DMA_ADDR_CONSTANT,
 *	          DMA_ADDR_POST_INCREMENT, DMA_ADDR_SINGLE_INDEX or
 *	          DMA_ADDR_DOUBLE_INDEX)
 *	@wr_mode: the xfer destination addressing mode (either DMA_ADDR_CONSTANT,
 *	          DMA_ADDR_POST_INCREMENT, DMA_ADDR_SINGLE_INDEX or
 *	          DMA_ADDR_DOUBLE_INDEX)
 *
 *
 *	LOCKING:
 *	DMA registers protected by internal mutex
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
int
omap3_dma_set_addr_mode(unsigned int ch, unsigned int src_mode,
						unsigned int dst_mode)
{
	struct omap3_dma_softc *sc = g_omap3_dma_sc;
	uint32_t ccr;
	
	/* Sanity check */
	if (sc == NULL)
		return (ENOMEM);
	
	mtx_lock(&sc->sc_mtx);
	
	if ((sc->sc_active_channels & (1 << ch)) == 0) {
		mtx_unlock(&sc->sc_mtx);
		return (EINVAL);
	}
	
	ccr = sc->sc_channel[ch].reg_ccr;
	
	ccr &= ~DMA4_CCR_SRC_ADDRESS_MODE(0x3);
	ccr |= DMA4_CCR_SRC_ADDRESS_MODE(src_mode);

	ccr &= ~DMA4_CCR_DST_ADDRESS_MODE(0x3);
	ccr |= DMA4_CCR_DST_ADDRESS_MODE(dst_mode);

	sc->sc_channel[ch].reg_ccr = ccr;
	
	sc->sc_channel[ch].need_reg_write = 1;

	mtx_unlock(&sc->sc_mtx);
	
	return 0;
}






