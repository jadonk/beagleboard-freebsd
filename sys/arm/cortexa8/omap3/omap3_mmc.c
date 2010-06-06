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
 * Driver for the MMC/SD/SDIO module on the TI OMAP3530 SoC.
 *
 * This driver is heavily based on the SD/MMC driver for the AT91 (at91_mci.c).
 *
 * It's important to realise that the MMC state machine is already in the kernel
 * and this driver only exposes the specific interfaces of the controller.
 *
 * This driver is still very much a work in progress, I've verified that basic
 * sector reading can be performed. But I've yet to test it with a file system
 * or even writing.  In addition I've only tested the driver with an SD card,
 * I've no idea if MMC cards work.
 * 
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/time.h>
#include <sys/timetc.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>

#include "mmcbr_if.h"
#include "mmcbus_if.h"


#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_mmc.h>
#include <arm/cortexa8/omap3/omap3_dma.h>
#include <arm/cortexa8/omap3/omap3_prcm.h>





/**
 *	Structure that stores the driver context
 */
struct omap3_mmc_softc {
	bus_space_tag_t		sc_iotag;
	bus_space_handle_t	sc_ioh;
	device_t			sc_dev;
	
	bus_dma_tag_t		sc_dmatag;
	bus_dmamap_t		sc_dmamap;
	int					sc_dmamapped;
	
	unsigned int		sc_dmach_rd;
	unsigned int		sc_dmach_wr;

	void*				sc_irq_h;
	struct resource*	sc_irq_res;	/* IRQ resource */

	struct mtx			sc_mtx;
	
	struct mmc_host		host;
	struct mmc_request*	req;
	struct mmc_command*	curcmd;
	
	int					flags;
#define CMD_STARTED		1
#define STOP_STARTED	2
	
	int bus_busy;		/* TODO: Needed ? */
	
	void *				sc_cmd_data_vaddr;
	int					sc_cmd_data_len;
	
};

/**
 *	Macros for driver mutex locking
 */
#define OMAP3_MMC_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define	OMAP3_MMC_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define OMAP3_MMC_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
			"mmc", MTX_DEF)
#define OMAP3_MMC_LOCK_DESTROY(_sc)		mtx_destroy(&_sc->sc_mtx);
#define OMAP3_MMC_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define OMAP3_MMC_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);





/**
 *	omap3_mmc_readl - reads a 32-bit value from a register 
 *	@sc: pointer to the driver context
 *	@off: register offset to read from
 *
 *	LOCKING:
 *	None
 *
 *	RETURNS:
 *	The 32-bit value read from the register
 */
static inline uint32_t
omap3_mmc_readl(struct omap3_mmc_softc *sc, bus_size_t off)
{
	return bus_space_read_4(sc->sc_iotag, sc->sc_ioh, off);
}

/**
 *	omap3_mmc_writel - writes a 32-bit value to a register 
 *	@sc: pointer to the driver context
 *	@off: register offset to write to
 *	@val: the value to write into the register
 *
 *	LOCKING:
 *	None
 *
 *	RETURNS:
 *	nothing
 */
static inline void
omap3_mmc_writel(struct omap3_mmc_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_iotag, sc->sc_ioh, off, val);
}



/* bus entry points */
static int omap3_mmc_probe(device_t dev);
static int omap3_mmc_attach(device_t dev);
static int omap3_mmc_detach(device_t dev);
static void omap3_mmc_intr(void *arg);
static void omap3_mmc_dma_intr(unsigned int ch, uint32_t status, void *data);
static void omap3_mmc_start(struct omap3_mmc_softc *sc);

/* helper routines */
static int omap3_mmc_activate(device_t dev);
static void omap3_mmc_deactivate(device_t dev);







/**
 *	omap3_mmc_init - initialises the MMC/SD/SIO controller
 *	@dev: mmc device handle
 *
 *	Called by the driver attach function during driver initialisation. This
 *	function is responsibly to setup the controller ready for transactions.
 *
 *	LOCKING:
 *	No locking, assumed to only be called during initialisation.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_init(device_t dev)
{
	struct omap3_mmc_softc *sc = device_get_softc(dev);
	uint32_t val;
	uint32_t i;
	
	/* 1: Enable the controller and interface/functional clocks */

	/* Set the EN_MMC1 bit (MMC SDIO 1 interface clock control) */
	omap3_prcm_enable_clk(OMAP3_MODULE_MMC1_FCLK);
	
	/* Set the EN_MMC1 bit (MMC1 functional clock control) */
	omap3_prcm_enable_clk(OMAP3_MODULE_MMC1_ICLK);
	
	
	
	
	/* 2: Issue a softreset to the controller */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCONFIG, 0x0002);
	while ((omap3_mmc_readl(sc, OMAP35XX_MMCHS_SYSSTATUS) & 0x01) == 0x00)
		continue;
	
	
	/* 3: MMCHS Controller Voltage Capabilities Initialization */
	val = omap3_mmc_readl(sc, OMAP35XX_MMCHS_CAPA);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_CAPA, (val | MMCHS_CAPA_VS30 |
											   MMCHS_CAPA_VS18));
	
	
	
	
	/* 4: MMCHS Controller Default Initialization */
	
	/* 4a: data bus width = 1, voltage = 1.8v, MMC bus power is on (not card's power) */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_HCTL, 0x00000b00);
	
	/* 4b: card's clock enable and card's clock frequency divider. */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, 0x0000a007);
	
	/* 4c: Set MMC bus mode to open drain. */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_CON, 0x00000001);
	
	
	
	/* 5: MMCHS Controller INIT Procedure Start */
	
	/* Prior to issuing any command, the MMCHS controller has to execute a special
	 * INIT procedure. The MMCHS controller has to generate a clock during 1ms.
	 * During the INIT procedure, the MMCHS controller generates 80 clock periods.
	 * In order to keep the 1ms gap, the MMCHS controller should be configured
	 * to generate a clock whose frequency is smaller or equal to 80 KHz. If the
	 * MMCHS controller divider bitfield width doesn't allow to choose big values,
	 * the MMCHS controller driver should perform the INIT procedure twice or
	 * three times. Twice is generally enough.
	 *
	 * The INIt procedure is executed by setting MMCHS1.MMCHS_CON[1] INIT
	 * bitfield to 1 and by sending a dummy command, writing 0x00000000 in
	 * MMCHS1.MMCHS_CMD register.
	 */
	for (i=0; i<3; i++) {
		
		/* sets MMCHS1.MMCHS_CON[1] INIT to 1 */
		val = omap3_mmc_readl(sc, OMAP35XX_MMCHS_CON);
		val |= 0x00000002;
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_CON, val);
		
		/* sends dummy command */
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_CMD, 0x00000000);
	}
	
	
	/* 6: MMCHS Controller Pre-card Identification Configuration */
	
	/* Before card identification starts, the MMCHS controller's configuration
	 * should change. MMC card's clock should now be 400 KHz according to MMC
	 * system spec requirements.
	 */
	
	/* data bus width = 1, voltage = 1.8v, MMC bus power is on (not card's power) */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_HCTL, 0x00000b00);

	/* card's clock enable and card's clock frequency divider. */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, 0x00003C07);

	/* Set MMC bus mode to open drain. */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_CON, 0x00000001);

	
}

	
/**
 *	omap3_mmc_fini - shutdown the MMC/SD/SIO controller
 *	@dev: mmc device handle
 *
 *	Responsible for shutting done the MMC controller, this function may be
 *	called as part of a reset sequence.
 *
 *	LOCKING:
 *	No locking, assumed to be called during tear-down/reset.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_fini(device_t dev)
{
	/* TODO: Cleanup properly */
	
	/* Disable the interrupts */

	/* Disable the functional and interface clocks */
	omap3_prcm_disable_clk(OMAP3_MODULE_MMC1_FCLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_MMC1_ICLK);
	
}



/**
 *	omap3_mmc_probe - probe function for the driver
 *	@dev: mmc device handle
 *
 *
 *
 *	RETURNS:
 *	always returns 0
 */
static int
omap3_mmc_probe(device_t dev)
{
	
	device_set_desc(dev, "TI OMAP3530 MMC/SD/SIO host bridge");
	return (0);
}

/**
 *	omap3_mmc_attach - attach function for the driver
 *	@dev: mmc device handle
 *
 *	Driver initialisation, sets-up the bus mappings, DMA mapping/channels and
 *	the actual controller by calling omap3_mmc_init().
 *
 *	RETURNS:
 *	Returns 0 on success or a negative error code.
 */
static int
omap3_mmc_attach(device_t dev)
{
	struct omap3_mmc_softc *sc = device_get_softc(dev);
	int err;
	device_t child;
	
	/* Save the device and bus tag */
	sc->sc_dev = dev;
	sc->sc_iotag = &omap3_bs_tag;
	
	/* Initiate the mtex lock */
	OMAP3_MMC_LOCK_INIT(sc);

	/* Indicate the DMA channels haven't yet been allocated */
	sc->sc_dmach_rd = (unsigned int)-1;
	sc->sc_dmach_wr = (unsigned int)-1;


	/* Activate the device */
	err = omap3_mmc_activate(dev);
	if (err)
		goto out;
	
	
	/* Allocate DMA tags and maps */
	err = bus_dma_tag_create(bus_get_dma_tag(dev), 1, 0,
							 BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL,
							 NULL, MAXPHYS, 1, MAXPHYS, BUS_DMA_ALLOCNOW, NULL,
							 NULL, &sc->sc_dmatag);
	if (err != 0)
		goto out;
	
	err = bus_dmamap_create(sc->sc_dmatag, 0,  &sc->sc_dmamap);
	if (err != 0)
		goto out;
	
					
	/* Activate a RX channel from the OMAP3 DMA driver */
	err = omap3_dma_activate_channel(&sc->sc_dmach_rd, omap3_mmc_dma_intr, sc);
	if (err != 0)
		goto out;
	err = omap3_dma_enable_channel_irq(sc->sc_dmach_rd, DMA_IRQ_FLAG_FRAME_COMPL);
	if (err != 0)
		goto out;
	err = omap3_dma_set_xfer_burst(sc->sc_dmach_rd, DMA_BURST_NONE, DMA_BURST_64);
	if (err != 0)
		goto out;
	err = omap3_dma_set_xfer_data_type(sc->sc_dmach_rd, DMA_DATA_32BITS_SCALAR);
	if (err != 0)
		goto out;
	err = omap3_dma_sync_params(sc->sc_dmach_rd, DMA_TRIGGER_MMC1_DMA_RX,
								DMA_SYNC_PACKET | DMA_SYNC_TRIG_ON_SRC);
	if (err != 0)
		goto out;
	err = omap3_dma_set_addr_mode(sc->sc_dmach_rd, DMA_ADDR_CONSTANT,
								  DMA_ADDR_POST_INCREMENT);
	if (err != 0)
		goto out;
	
	
	/* Activate and configure the TX DMA channel */
	err = omap3_dma_activate_channel(&sc->sc_dmach_wr, omap3_mmc_dma_intr, sc);
	if (err != 0)
		goto out;
	err = omap3_dma_enable_channel_irq(sc->sc_dmach_wr, DMA_IRQ_FLAG_FRAME_COMPL);
	if (err != 0)
		goto out;
	err = omap3_dma_set_xfer_burst(sc->sc_dmach_wr, DMA_BURST_64, DMA_BURST_NONE);
	if (err != 0)
		goto out;
	err = omap3_dma_set_xfer_data_type(sc->sc_dmach_wr, DMA_DATA_32BITS_SCALAR);
	if (err != 0)
		goto out;
	err = omap3_dma_sync_params(sc->sc_dmach_wr, DMA_TRIGGER_MMC1_DMA_TX,
								DMA_SYNC_PACKET | DMA_SYNC_TRIG_ON_DST);
	if (err != 0)
		goto out;
	err = omap3_dma_set_addr_mode(sc->sc_dmach_wr, DMA_ADDR_POST_INCREMENT,
								  DMA_ADDR_CONSTANT);
	if (err != 0)
		goto out;
	
	
	
	/* Shutdown and restart the controller */
	omap3_mmc_fini(dev);
	omap3_mmc_init(dev);
	
		
	/* Activate the interrupt and attach a handler */
	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
						 NULL, omap3_mmc_intr, sc, &sc->sc_irq_h);
	if (err != 0)
		goto out;
	
	
	/* Add host details */
	sc->host.f_min = OMAP3_MMC_REF_CLK / 1023;
	sc->host.f_max = OMAP3_MMC_REF_CLK;
	sc->host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->host.caps = MMC_CAP_4_BIT_DATA;

	child = device_add_child(dev, "mmc", 0);
	
	device_set_ivars(dev, &sc->host);
	err = bus_generic_attach(dev);

out:
	if (err) {
		OMAP3_MMC_LOCK_DESTROY(sc);
		omap3_mmc_deactivate(dev);

		if (sc->sc_dmach_rd != (unsigned int)-1)
			omap3_dma_deactivate_channel(sc->sc_dmach_rd);
		if (sc->sc_dmach_wr != (unsigned int)-1)
			omap3_dma_deactivate_channel(sc->sc_dmach_wr);
	}
	
	return (err);
}



/**
 *	omap3_mmc_detach - dettach function for the driver
 *	@dev: mmc device handle
 *
 *	Shutdowns the controll and release resources allocated by the driver.
 *
 *	RETURNS:
 *	Always returns 0.
 */
static int
omap3_mmc_detach(device_t dev)
{
	struct omap3_mmc_softc *sc = device_get_softc(dev);

	omap3_mmc_fini(dev);
	omap3_mmc_deactivate(dev);
	
	omap3_dma_deactivate_channel(sc->sc_dmach_wr);
	omap3_dma_deactivate_channel(sc->sc_dmach_rd);

	return (0);
}



/**
 *	omap3_mmc_activate - activates the driver
 *	@dev: mmc device handle
 *
 *	Maps in the register set and requests an IRQ handler for the MMC controller.
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	0 on sucess
 *	ENOMEM if failed to map register set
 */
static int
omap3_mmc_activate(device_t dev)
{
	struct omap3_mmc_softc *sc = device_get_softc(dev);
	int rid = 0;
	
	/* Map the register set for the MMC controller */
	if (bus_space_map(sc->sc_iotag, OMAP35XX_MMCHS1_HWBASE, OMAP35XX_MMCHS_SIZE,
					  0, &sc->sc_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
		sc->sc_ioh = 0;
	}

	/* Allocate an IRQ resource for the MMC controller */
	sc->sc_irq_res = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, OMAP35XX_IRQ_MMC1,
							            OMAP35XX_IRQ_MMC1, 1, RF_ACTIVE);
	if (sc->sc_irq_res == NULL)
		goto errout;
	
	return (0);

errout:
	omap3_mmc_deactivate(dev);
	return (ENOMEM);
}



/**
 *	omap3_mmc_deactivate - deactivates the driver
 *	@dev: mmc device handle
 *
 *	Unmaps the register set and releases the IRQ resource.
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_deactivate(device_t dev)
{
	struct omap3_mmc_softc *sc= device_get_softc(dev);
	
	/* Remove the IRQ handler */
	if (sc->sc_irq_h != NULL) {
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_h);
		sc->sc_irq_h = NULL;
	}
	
	/* Do the generic detach */
	bus_generic_detach(sc->sc_dev);
	
	/* Unmap the MMC controller registers */
	if (sc->sc_ioh != 0) {
		bus_space_unmap(sc->sc_iotag, sc->sc_ioh, OMAP35XX_MMCHS_SIZE);
		sc->sc_ioh = 0;
	}

	/* Release the IRQ resource */
	if (sc->sc_irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->sc_irq_res),
							 sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}
	
	return;
}


/**
 *	omap3_mmc_getaddr - called by the DMA function to simply return the phys address 
 *	@arg: caller supplied arg
 *	@segs: array of segments (although in our case should only be one)
 *	@nsegs: number of segments (in our case should be 1)
 *	@error:
 *
 *	This function is called by bus_dmamap_load() after it has compiled an array
 *	of segments, each segment is a phsyical chunk of memory. However in our case
 *	we should only have one segment, because we don't (yet?) support DMA scatter
 *	gather. To ensure we only have one segment, the DMA tag was created by
 *	bus_dma_tag_create() (called from omap_mmc_attach) with nsegments set to 1.
 *
 */
static void
omap3_mmc_getaddr(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	if (error != 0)
		return;

	*(bus_addr_t *)arg = segs[0].ds_addr;
}




/**
 *	omap3_mmc_intr - interrupt handler for DMA events triggered by the controller
 *	@ch: the dma channel number
 *	@status: bit field of the status bytes
 *	@data: callback data, in this case a pointer to the controller struct
 *	
 *
 *	LOCKING:
 *	Called from interrupt context
 *
 */
static void
omap3_mmc_dma_intr(unsigned int ch, uint32_t status, void *data)
{
	/* Ignore for now ... we don't need this interrupt as we already have the
	 * interrupt from the MMC controller.
	 */
}



/**
 *	omap3_mmc_intr_xfer_compl - called if a 'transfer complete; IRQ was received
 *	@sc: pointer to the driver context
 *	@cmd: the command that was sent previously
 *	
 *	This function is simply responsible for syncing up the DMA buffer.
 *
 *	LOCKING:
 *	Called from interrupt context
 *
 *	RETURNS:
 *	Return value indicates if the transaction is complete, not done = 0, done != 0
 */
static int
omap3_mmc_intr_xfer_compl(struct omap3_mmc_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmd_reg;

	
	/* Read command register to test whether this command was a read or write. */
	cmd_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_CMD);
	
	/* Sync-up the DMA buffer so the caller can access the new memory */
	if (cmd_reg & MMCHS_CMD_DDIR) {
		bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap, BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->sc_dmatag, sc->sc_dmamap);
	}
	else {
		bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_dmatag, sc->sc_dmamap);
	}
	sc->sc_dmamapped--;
	
	
	/* Debugging dump of the data received */
#if 0	
	{
		int i;
		uint8_t *p = (uint8_t*) sc->sc_cmd_data_vaddr;
		for (i=0; i<sc->sc_cmd_data_len; i++) {
			if ((i % 16) == 0)
				printf("\n0x%04x : ", i);
			printf("%02X ", *p++);
		}
		printf("\n");
	}
#endif
	
	/* We are done, transfer complete */
	return 1;
}


/**
 *	omap3_mmc_intr_xfer_compl - called if a 'command complete' IRQ was received
 *	@sc: pointer to the driver context
 *	@cmd: the command that was sent previously
 *	
 *
 *	LOCKING:
 *	Called from interrupt context
 *
 *	RETURNS:
 *	Return value indicates if the transaction is complete, not done = 0, done != 0
 */
static int
omap3_mmc_intr_cmd_compl(struct omap3_mmc_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmd_reg;
	

	/* Copy the response into the request struct ... if a response was
	 * expected */
	if (cmd != NULL && (cmd->flags & MMC_RSP_PRESENT)) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[3] = omap3_mmc_readl(sc, OMAP35XX_MMCHS_RSP10);
			cmd->resp[2] = omap3_mmc_readl(sc, OMAP35XX_MMCHS_RSP32);
			cmd->resp[1] = omap3_mmc_readl(sc, OMAP35XX_MMCHS_RSP54);
			cmd->resp[0] = omap3_mmc_readl(sc, OMAP35XX_MMCHS_RSP76);
		} else {
			cmd->resp[0] = omap3_mmc_readl(sc, OMAP35XX_MMCHS_RSP10);
		}
	}
	
	/* Check if the command was expecting some data transfer, if not
	 * we are done. */
	cmd_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_CMD);
	return ((cmd_reg & MMCHS_CMD_DP) == 0);
}


/**
 *	omap3_mmc_intr_error - handles error interrupts
 *	@sc: pointer to the driver context
 *	@cmd: the command that was sent previously
 *	@stat_reg: the value that was in the status register
 *	
 *
 *	LOCKING:
 *	Called from interrupt context
 *
 *	RETURNS:
 *	Return value indicates if the transaction is complete, not done = 0, done != 0
 */
static int
omap3_mmc_intr_error(struct omap3_mmc_softc *sc, struct mmc_command *cmd,
					 uint32_t stat_reg)
{
	uint32_t sysctl_reg;

	/* Ignore CRC errors on CMD2 and ACMD47, per relevant standards */
	if ((stat_reg & MMCHS_STAT_CCRC) && (cmd->opcode == MMC_SEND_OP_COND ||
										 cmd->opcode == ACMD_SD_SEND_OP_COND))
		cmd->error = MMC_ERR_NONE;
	else if (stat_reg & (MMCHS_STAT_CTO | MMCHS_STAT_DTO))
		cmd->error = MMC_ERR_TIMEOUT;
	else if (stat_reg & (MMCHS_STAT_CCRC | MMCHS_STAT_DCRC))
		cmd->error = MMC_ERR_BADCRC;
	else
		cmd->error = MMC_ERR_FAILED;
	
	
	/* Abort the DMA transfer (DDIR bit tells direction) */
	if (omap3_mmc_readl(sc, OMAP35XX_MMCHS_CMD) & MMCHS_CMD_DDIR)
		omap3_dma_stop_xfer(sc->sc_dmach_rd);
	else
		omap3_dma_stop_xfer(sc->sc_dmach_wr);
	
	
	/* If an error occure abort the DMA operation and free the dma map */
	if ((sc->sc_dmamapped > 0) && (cmd->error != MMC_ERR_NONE)) {
		bus_dmamap_unload(sc->sc_dmatag, sc->sc_dmamap);
		sc->sc_dmamapped--;
	}
	
	
	/* Data error occured? ... if so issue a soft reset for the data line */
	if (stat_reg & (MMCHS_STAT_DEB | MMCHS_STAT_DCRC | MMCHS_STAT_DTO)) {
		
		/* Reset the data lines and wait for it to complete */
		sysctl_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_SYSCTL);
		sysctl_reg |= MMCHS_SYSCTL_SRD;
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, sysctl_reg);
		
		/* TODO: This is probably not a good idea in an IRQ routine */
		while (omap3_mmc_readl(sc, OMAP35XX_MMCHS_SYSCTL) & MMCHS_SYSCTL_SRD)
			continue;
	}
	
	/* On any error the command is cancelled ... so we are done */
	return 1;
}



/**
 *	omap3_mmc_intr - interrupt handler for MMC/SD/SDIO controller
 *	@arg: pointer to the driver context
 *
 *	Interrupt handler for the MMC/SD/SDIO controller, responsible for handling
 *	the IRQ and clearing the status flags.
 *
 *	LOCKING:
 *	Called from interrupt context
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_intr(void *arg)
{
	struct omap3_mmc_softc *sc = (struct omap3_mmc_softc *) arg;
	uint32_t stat_reg;
	int done = 0;
	
	
	OMAP3_MMC_LOCK(sc);
	
	stat_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_STAT)
	         & (omap3_mmc_readl(sc, OMAP35XX_MMCHS_IE) | MMCHS_STAT_ERRI);


	if (stat_reg & MMCHS_STAT_ERRI) {
		/* An error has been tripped in the status register */
		done = omap3_mmc_intr_error(sc, sc->curcmd, stat_reg);
		
	} else {
		
		/* NOTE: This implementation could be a bit inefficent, I don't think
		 * it is necessary to handle both the 'command complete' and 'transfer
		 * complete' for data transfers ... presumably just transfer complete
		 * is enough.
		 */
		
		/* No error */
		sc->curcmd->error = MMC_ERR_NONE;

		/* Check if the command completed */
		if (stat_reg & MMCHS_STAT_CC) {
			done = omap3_mmc_intr_cmd_compl(sc, sc->curcmd);
		}
			
		/* Check if the transfer has completed */
		if (stat_reg & MMCHS_STAT_TC) {
			done = omap3_mmc_intr_xfer_compl(sc, sc->curcmd);
		}

	}
	
	/* Clear all the interrupt status bits by writing the value back */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_STAT, stat_reg);

	
	/* This may mark the command as done if there is no stop request */
	/* TODO: This is a bit ugly, needs fix-up */
	if (done)
		omap3_mmc_start(sc);
	
	OMAP3_MMC_UNLOCK(sc);
}



/**
 *	omap3_mmc_start_cmd - starts the given command
 *	@sc: pointer to the driver context
 *	@cmd: the command to start
 *	
 *	The call tree for this function is 
 *		- omap3_mmc_start_cmd
 *			- omap3_mmc_start
 *				- omap3_mmc_request
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_start_cmd(struct omap3_mmc_softc *sc, struct mmc_command *cmd)
{
	uint32_t cmd_reg, con_reg, ise_reg;
	struct mmc_data *data;
	struct mmc_request *req;
	void *vaddr;
	bus_addr_t paddr;
	uint32_t pktsize;
	
	
	sc->curcmd = cmd;
	data = cmd->data;
	req = cmd->mrq;
	
	/* By default no bits should be set for the CON register */
	con_reg = 0x0;
	
	/* Load the command into bits 29:24 of the CMD register */
	cmd_reg = (uint32_t)(cmd->opcode & 0x3F) << 24;
	
	/* Set the default set of interrupts */
	ise_reg = (MMCHS_STAT_CERR | MMCHS_STAT_CTO | MMCHS_STAT_CC | MMCHS_STAT_CEB);
	
	/* Enable CRC checking if requested */
	if (cmd->flags & MMC_RSP_CRC)
		ise_reg |= MMCHS_STAT_CCRC;
	
	/* Enable reply index checking if the response supports it */
	if (cmd->flags & MMC_RSP_OPCODE)
		ise_reg |= MMCHS_STAT_CIE;

		
	/* Set the expected response length */
	if (MMC_RSP(cmd->flags) == MMC_RSP_NONE) {
		cmd_reg |= MMCHS_CMD_RSP_TYPE_NO;
	} else {
		if (cmd->flags & MMC_RSP_136)
			cmd_reg |= MMCHS_CMD_RSP_TYPE_136;
		else if (cmd->flags & MMC_RSP_BUSY)
			cmd_reg |= MMCHS_CMD_RSP_TYPE_48_BSY;
		else
			cmd_reg |= MMCHS_CMD_RSP_TYPE_48;
		
		/* Enable command index/crc checks if necessary expected */
		if (cmd->flags & MMC_RSP_CRC)
			cmd_reg |= MMCHS_CMD_CCCE;
		if (cmd->flags & MMC_RSP_OPCODE)
			cmd_reg |= MMCHS_CMD_CICE;
	}
			
	/* Set the bits for the special commands CMD12 (MMC_STOP_TRANSMISSION) and
	 * CMD52 (SD_IO_RW_DIRECT) */
	if (cmd->opcode == MMC_STOP_TRANSMISSION)
		cmd_reg |= MMCHS_CMD_CMD_TYPE_IO_ABORT;
	
	/* Put the controller in open drain mode */
	if (sc->host.ios.bus_mode == opendrain)
		con_reg |= MMCHS_CON_OD;
	
	/* Check if there is any data to write */
	if (data == NULL) {
		
		/* The no data case is fairly simple */
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_CON, con_reg);
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_IE, ise_reg);
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_ISE, ise_reg);
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_ARG, cmd->arg);
		omap3_mmc_writel(sc, OMAP35XX_MMCHS_CMD, cmd_reg);
		return;
	}
	
	/* Indicate that data is present */
	cmd_reg |= MMCHS_CMD_DP | MMCHS_CMD_MSBS | MMCHS_CMD_BCE;
	
	/* Indicate a read operation */
	if (data->flags & MMC_DATA_READ)
		cmd_reg |= MMCHS_CMD_DDIR;

	
	//if (data->flags & (MMC_DATA_READ | MMC_DATA_WRITE))
	//	cmd_reg |= MCI_CMDR_TRCMD_START;
	
	/* Streaming mode */
	if (data->flags & MMC_DATA_STREAM) {
		con_reg |= MMCHS_CON_STR;
	}
	
	/* Multi-block mode */
	if (data->flags & MMC_DATA_MULTI) {
		cmd_reg |= MMCHS_CMD_MSBS;
	}
	
	
	/* Enable extra interrupt sources for the transfer */
	ise_reg |= (MMCHS_STAT_TC | MMCHS_STAT_DTO | MMCHS_STAT_DEB | MMCHS_STAT_CEB);
	if (cmd->flags & MMC_RSP_CRC)
		ise_reg |= MMCHS_STAT_DCRC;
	
	/* Enable the DMA transfer bit */
	cmd_reg |= MMCHS_CMD_DE;
	
	/* Set the block size and block count */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_BLK, (1 << 16) | data->len);
	
		
	
	/* Setup the DMA stuff */
	if (data->flags & (MMC_DATA_READ | MMC_DATA_WRITE)) {
		
		vaddr = cmd->data->data;
		data->xfer_len = 0;
		
		/* Map the buffer buf into bus space using the dmamap map. */
		if (bus_dmamap_load(sc->sc_dmatag, sc->sc_dmamap, vaddr, data->len,
							omap3_mmc_getaddr, &paddr, 0) != 0) {
			if (req->cmd->flags & STOP_STARTED)
				req->stop->error = MMC_ERR_NO_MEMORY;
			else
				req->cmd->error = MMC_ERR_NO_MEMORY;
			sc->req = NULL;
			sc->curcmd = NULL;
			req->done(req);
			return;
		}
				
		
		/* Calculate the packet size, the max packet size is 512 bytes
		 * (or 128 32-bit elements).
		 */
		pktsize = min((data->len / 4), (512 / 4));

		/* Sync the DMA buffer and setup the DMA controller */
		if (data->flags & MMC_DATA_READ) {
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap, BUS_DMASYNC_PREREAD);
			omap3_dma_start_xfer_packet(sc->sc_dmach_rd,
								 (OMAP35XX_MMCHS1_HWBASE + OMAP35XX_MMCHS_DATA),
								 paddr, 1, (data->len / 4), pktsize);
		} else {
			bus_dmamap_sync(sc->sc_dmatag, sc->sc_dmamap, BUS_DMASYNC_PREWRITE);
			omap3_dma_start_xfer_packet(sc->sc_dmach_wr, paddr,
								 (OMAP35XX_MMCHS1_HWBASE + OMAP35XX_MMCHS_DATA),
								 1, (data->len / 4), pktsize);
		}

		/* Increase the mapped count */
		sc->sc_dmamapped++;
		
		sc->sc_cmd_data_vaddr = vaddr;
		sc->sc_cmd_data_len = data->len;
	}

/*	
	printf("MMCHS_CON 0x%08x\n", con_reg);
	printf("MMCHS_IE  0x%08x\n", ise_reg);
	printf("MMCHS_ISE 0x%08x\n", ise_reg);
	printf("MMCHS_CMD 0x%08x\n", cmd_reg);  // 0x123a0033
	printf("MMCHS_ARG 0x%08x\n", cmd->arg);
	printf("MMCHS_BLK 0x%08x\n", omap3_mmc_readl(sc, OMAP35XX_MMCHS_BLK));
	printf("MMCHS_PSTATE 0x%08x\n", omap3_mmc_readl(sc, OMAP35XX_MMCHS_PSTATE));
*/
	
	/* Finally kick off the command */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_CON, con_reg);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_IE, ise_reg);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_ISE, ise_reg);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_ARG, cmd->arg);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_CMD, cmd_reg);

	/* and we're done */
}




/**
 *	omap3_mmc_start - starts a request stored in the driver context
 *	@sc: pointer to the driver context
 *
 *	This function is called by omap3_mmc_request() in response to a read/write
 *	request from the MMC core module.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_mmc_start(struct omap3_mmc_softc *sc)
{
	struct mmc_request *req;
	
	
	/* Sanity check we have a request */
	req = sc->req;
	if (req == NULL)
		return;
	
	/* assert locked */
	if (!(sc->flags & CMD_STARTED)) {
		sc->flags |= CMD_STARTED;
		omap3_mmc_start_cmd(sc, req->cmd);
		return;
	}
	
	if (!(sc->flags & STOP_STARTED) && req->stop) {
		sc->flags |= STOP_STARTED;
		omap3_mmc_start_cmd(sc, req->stop);
		return;
	}
	
	/* We must be done -- bad idea to do this while locked? */
	sc->req = NULL;
	sc->curcmd = NULL;
	req->done(req);
}



/**
 *	omap3_mmc_request - entry point for all read/write/cmd requests
 *	@brdev: mmc bridge device handle
 *	@reqdev: the device doing the requesting ?
 *	@req: the action requested
 *
 *	LOCKING:
 *	None, internally takes the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	0 on success
 *	EBUSY if the driver is already performing a request
 */
static int
omap3_mmc_request(device_t brdev, device_t reqdev, struct mmc_request *req)
{
	struct omap3_mmc_softc *sc = device_get_softc(brdev);
	
	
	OMAP3_MMC_LOCK(sc);

	// XXX do we want to be able to queue up multiple commands?
	// XXX sounds like a good idea, but all protocols are sync, so
	// XXX maybe the idea is naive...
	if (sc->req != NULL) {
		OMAP3_MMC_UNLOCK(sc);
		return (EBUSY);
	}
	
	/* Store the request and start the command */
	sc->req = req;
	sc->flags = 0;
	omap3_mmc_start(sc);

	OMAP3_MMC_UNLOCK(sc);
	
	return (0);
}


/**
 *	omap3_mmc_get_ro - returns the status of the read-only setting
 *	@brdev: mmc bridge device handle
 *	@reqdev: device doing the request
 *
 *	This function is currently not implemented.
 *
 *	LOCKING:
 *	-
 *
 *	RETURNS:
 *	0 if not read-only
 *	1 if read only
 */
static int
omap3_mmc_get_ro(device_t brdev, device_t reqdev)
{
	printf("[BRG] %s : %d\n", __func__, __LINE__);	
	
	/* TODO: Implement this correctly */
	return (0);
}



/**
 *	omap3_mmc_update_ios - sets bus/controller settings
 *	@brdev: mmc bridge device handle
 *	@reqdev: device doing the request
 *
 *	Called to set the bus and controller settings that need to be applied to
 *	the actual HW.  Currently this function just sets the bus width and the
 *	clock speed.
 *
 *	LOCKING:
 *	
 *
 *	RETURNS:
 *	0 if function succeeded
 */
static int
omap3_mmc_update_ios(device_t brdev, device_t reqdev)
{
	struct omap3_mmc_softc *sc;
	struct mmc_host *host;
	struct mmc_ios *ios;
	uint32_t clkdiv;
	uint32_t hctl_reg;
	uint32_t sysctl_reg;
	
	sc = device_get_softc(brdev);
	host = &sc->host;
	ios = &host->ios;
	
	printf("[BRG] %s : %d : clock %d : width %d\n", __func__, __LINE__, ios->clock, ios->bus_width);
	
	/* need the MMCHS_SYSCTL register */
	sysctl_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_SYSCTL);

	/* Just in case this hasn't been setup before, set the timeout to the default */
	sysctl_reg &= MMCHS_SYSCTL_DTO_MASK;
	sysctl_reg |= MMCHS_SYSCTL_DTO(0xe);
	
	/* Disable the clock output while configuring the new clock */
	sysctl_reg &= ~(MMCHS_SYSCTL_ICE | MMCHS_SYSCTL_CEN);
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, sysctl_reg);
	
	/* bus mode? */
	if (ios->clock == 0) {
		clkdiv = 0;
	} else {
		clkdiv = OMAP3_MMC_REF_CLK / ios->clock;
		if (clkdiv < 1)
			clkdiv = 1;
		if ((OMAP3_MMC_REF_CLK / clkdiv) > ios->clock)
			clkdiv += 1;
		if (clkdiv > 250)
			clkdiv = 250;
	}

	/* Set the new clock divider */
	sysctl_reg &= ~MMCHS_SYSCTL_CLKD_MASK;
	sysctl_reg |= MMCHS_SYSCTL_CLKD(clkdiv);
	
	/* Write the new settings ... */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, sysctl_reg);
	/* ... write the internal clock enable bit ... */
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, sysctl_reg | MMCHS_SYSCTL_ICE);
	/* ... wait for the clock to stablise ... */
	while (((sysctl_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_SYSCTL)) &
			MMCHS_SYSCTL_ICS) == 0) {
		continue;
	}
	/* ... then enable */
	sysctl_reg |= MMCHS_SYSCTL_CEN;
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_SYSCTL, sysctl_reg);
	
	
	/* Set the bus width */
	hctl_reg = omap3_mmc_readl(sc, OMAP35XX_MMCHS_HCTL);
	if (ios->bus_width == bus_width_4)
		hctl_reg |= MMCHS_HCTL_DTW;
	else
		hctl_reg &= ~MMCHS_HCTL_DTW;
	omap3_mmc_writel(sc, OMAP35XX_MMCHS_HCTL, hctl_reg);

	
	return (0);
}


/**
 *	omap3_mmc_acquire_host - 
 *	@brdev: mmc bridge device handle
 *	@reqdev: device doing the request
 *
 *	TODO: Is this function needed ?
 *
 *	LOCKING:
 *	none
 *
 *	RETURNS:
 *	0 function succeeded
 *	
 */
static int
omap3_mmc_acquire_host(device_t brdev, device_t reqdev)
{
	struct omap3_mmc_softc *sc = device_get_softc(brdev);
	int err = 0;
	
	printf("[BRG] %s : %d\n", __func__, __LINE__);	
	
	OMAP3_MMC_LOCK(sc);
	while (sc->bus_busy)
		msleep(sc, &sc->sc_mtx, PZERO, "mmc", hz / 5);
	sc->bus_busy++;
	OMAP3_MMC_UNLOCK(sc);
	return (err);
}

/**
 *	omap3_mmc_release_host - 
 *	@brdev: mmc bridge device handle
 *	@reqdev: device doing the request
 *
 *	TODO: Is this function needed ?
 *
 *	LOCKING:
 *	none
 *
 *	RETURNS:
 *	0 function succeeded
 *	
 */
static int
omap3_mmc_release_host(device_t brdev, device_t reqdev)
{
	struct omap3_mmc_softc *sc = device_get_softc(brdev);
	
	printf("[BRG] %s : %d\n", __func__, __LINE__);	

	OMAP3_MMC_LOCK(sc);
	sc->bus_busy--;
	wakeup(sc);
	OMAP3_MMC_UNLOCK(sc);
	return (0);
}




/**
 *	omap3_mmc_read_ivar - returns driver conf variables
 *	@bus: 
 *	@child: 
 *	@which: The variable to get the result for
 *	@result: Upon return will store the variable value
 *
 *	
 *
 *	LOCKING:
 *	None, caller must hold locks
 *
 *	RETURNS:
 *	0 on success
 *	EINVAL if the variable requested is invalid
 */
static int
omap3_mmc_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct omap3_mmc_softc *sc = device_get_softc(bus);
	
	switch (which) {
		case MMCBR_IVAR_BUS_MODE:
			*(int *)result = sc->host.ios.bus_mode;
			break;
		case MMCBR_IVAR_BUS_WIDTH:
			*(int *)result = sc->host.ios.bus_width;
			break;
		case MMCBR_IVAR_CHIP_SELECT:
			*(int *)result = sc->host.ios.chip_select;
			break;
		case MMCBR_IVAR_CLOCK:
			*(int *)result = sc->host.ios.clock;
			break;
		case MMCBR_IVAR_F_MIN:
			*(int *)result = sc->host.f_min;
			break;
		case MMCBR_IVAR_F_MAX:
			*(int *)result = sc->host.f_max;
			break;
		case MMCBR_IVAR_HOST_OCR:
			*(int *)result = sc->host.host_ocr;
			break;
		case MMCBR_IVAR_MODE:
			*(int *)result = sc->host.mode;
			break;
		case MMCBR_IVAR_OCR:
			*(int *)result = sc->host.ocr;
			break;
		case MMCBR_IVAR_POWER_MODE:
			*(int *)result = sc->host.ios.power_mode;
			break;
		case MMCBR_IVAR_VDD:
			*(int *)result = sc->host.ios.vdd;
			break;
		case MMCBR_IVAR_CAPS:
			*(int *)result = sc->host.caps;
			break;
		case MMCBR_IVAR_MAX_DATA:
			*(int *)result = 1;
			break;
		default:
			return (EINVAL);
	}
	return (0);
}


/**
 *	omap3_mmc_write_ivar - writes a driver conf variables
 *	@bus: 
 *	@child: 
 *	@which: The variable to set
 *	@value: The value to write into the variable
 *
 *	
 *
 *	LOCKING:
 *	None, caller must hold locks
 *
 *	RETURNS:
 *	0 on success
 *	EINVAL if the variable requested is invalid
 */
static int
omap3_mmc_write_ivar(device_t bus, device_t child, int which, uintptr_t value)
{
	struct omap3_mmc_softc *sc = device_get_softc(bus);
	
	switch (which) {
		case MMCBR_IVAR_BUS_MODE:
			sc->host.ios.bus_mode = value;
			break;
		case MMCBR_IVAR_BUS_WIDTH:
			sc->host.ios.bus_width = value;
			break;
		case MMCBR_IVAR_CHIP_SELECT:
			sc->host.ios.chip_select = value;
			break;
		case MMCBR_IVAR_CLOCK:
			sc->host.ios.clock = value;
			break;
		case MMCBR_IVAR_MODE:
			sc->host.mode = value;
			break;
		case MMCBR_IVAR_OCR:
			sc->host.ocr = value;
			break;
		case MMCBR_IVAR_POWER_MODE:
			sc->host.ios.power_mode = value;
			break;
		case MMCBR_IVAR_VDD:
			sc->host.ios.vdd = value;
			break;
			/* These are read-only */
		case MMCBR_IVAR_CAPS:
		case MMCBR_IVAR_HOST_OCR:
		case MMCBR_IVAR_F_MIN:
		case MMCBR_IVAR_F_MAX:
		case MMCBR_IVAR_MAX_DATA:
			return (EINVAL);
		default:
			return (EINVAL);
	}
	return (0);
}




static device_method_t g_omap3_mmc_methods[] = {
	/* device_if */
	DEVMETHOD(device_probe, omap3_mmc_probe),
	DEVMETHOD(device_attach, omap3_mmc_attach),
	DEVMETHOD(device_detach, omap3_mmc_detach),
	
	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	omap3_mmc_read_ivar),
	DEVMETHOD(bus_write_ivar,	omap3_mmc_write_ivar),
	
	/* mmcbr_if - MMC state machine callbacks */
	DEVMETHOD(mmcbr_update_ios, omap3_mmc_update_ios),
	DEVMETHOD(mmcbr_request, omap3_mmc_request),
	DEVMETHOD(mmcbr_get_ro, omap3_mmc_get_ro),
	DEVMETHOD(mmcbr_acquire_host, omap3_mmc_acquire_host),
	DEVMETHOD(mmcbr_release_host, omap3_mmc_release_host),
	
	{0, 0},
};

static driver_t g_omap3_mmc_driver = {
	"omap3_mmc",
	g_omap3_mmc_methods,
	sizeof(struct omap3_mmc_softc),
};
static devclass_t g_omap3_mmc_devclass;


DRIVER_MODULE(omap3_mmc, omap3, g_omap3_mmc_driver, g_omap3_mmc_devclass, 0, 0);
MODULE_DEPEND(omap3_mmc, omap3_prcm, 1, 1, 1);

