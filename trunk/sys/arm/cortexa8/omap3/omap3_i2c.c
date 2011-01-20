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
 * Driver for the I2C module on the TI OMAP3530 SoC.
 *
 * This driver is heavily based on the TWI driver for the AT91 (at91_twi.c).
 *
 * The OMAP3530 has 3 I2C controllers, hints are used to indicate which devices
 * to enable.
 *
 * CAUTION: The I2Ci registers are limited to 16 bit and 8 bit data accesses,
 * 32 bit data access is not allowed and can corrupt register content.
 *
 * This driver currently doesn't use DMA for the transfer, although I hope to
 * incorporate that sometime in the future.  The idea being that for transaction
 * larger than a certain size the DMA engine is used, for anything less the
 * normal interrupt/fifo driven option is used.
 *
 *
 * WARNING: This driver uses mtx_sleep and interrupts to perform transactions,
 * which means you can't do a transaction during startup before the interrupts
 * have been enabled.  Hint - the freebsd function config_intrhook_establish().
 */



#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_prcm.h>
#include <arm/cortexa8/omap3/omap3_i2c.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"




/**
 *	I2C device driver context, a pointer to this is stored in the device
 *	driver structure.
 */
struct omap3_i2c_softc
{
	device_t			sc_dev;
	unsigned int        sc_controller;  /* the controller number; 1, 2 or 3 */
	bus_space_tag_t		sc_iotag;
	bus_space_handle_t	sc_ioh;
	
	device_t			sc_iicbus;

	void*				sc_irq_h;
	struct resource*	sc_irq_res;	    /* IRQ resource */

	struct mtx			sc_mtx;
	
	volatile uint16_t	sc_stat_flags;	/* contains the status flags last IRQ */

	uint16_t			sc_i2c_addr;
};


/**
 *	omap3_i2c_readw - reads a 16-bit value from one of the I2C registers
 *	@sc: I2C device context
 *	@off: the byte offset within the register bank to read from.
 *
 *
 *	LOCKING:
 *	No locking required
 *
 *	RETURNS:
 *	16-bit value read from the register.
 */
static inline uint16_t
omap3_i2c_readw(struct omap3_i2c_softc *sc, bus_size_t off)
{
	return bus_space_read_2(sc->sc_iotag, sc->sc_ioh, off);
}

/**
 *	omap3_i2c_writew - writes a 16-bit value to one of the I2C registers
 *	@sc: I2C device context
 *	@off: the byte offset within the register bank to read from.
 *	@val: the value to write into the register
 *
 *	LOCKING:
 *	No locking required
 *
 *	RETURNS:
 *	16-bit value read from the register.
 */
static inline void
omap3_i2c_writew(struct omap3_i2c_softc *sc, bus_size_t off, uint16_t val)
{
	bus_space_write_2(sc->sc_iotag, sc->sc_ioh, off, val);
}


/**
 *	Locking macros used throughout the driver
 */
#define OMAP3_I2C_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define	OMAP3_I2C_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define OMAP3_I2C_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	         "omap3_i2c", MTX_DEF)
#define OMAP3_I2C_LOCK_DESTROY(_sc)		mtx_destroy(&_sc->sc_mtx);
#define OMAP3_I2C_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define OMAP3_I2C_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);


static devclass_t omap3_i2c_devclass;

/* bus entry points */

static int omap3_i2c_probe(device_t dev);
static int omap3_i2c_attach(device_t dev);
static int omap3_i2c_detach(device_t dev);
static void omap3_i2c_intr(void *);

/* helper routines */
static int omap3_i2c_activate(device_t dev);
static void omap3_i2c_deactivate(device_t dev);






/**
 *	omap3_i2c_probe - probe function for the driver
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	
 *
 *	RETURNS:
 *	Always returns 0
 */
static int
omap3_i2c_probe(device_t dev)
{
	device_set_desc(dev, "TI OMAP3530 I2C Controller");
	return (0);
}


/**
 *	omap3_i2c_attach - attach function for the driver
 *	@dev: i2c device handle
 *
 *	Initialised driver data structures and activates the I2C controller. 
 *
 *	LOCKING:
 *	
 *
 *	RETURNS:
 *	
 */
static int
omap3_i2c_attach(device_t dev)
{
	struct omap3_i2c_softc *sc = device_get_softc(dev);
	int err;
	
	sc->sc_dev = dev;
	sc->sc_iotag = &omap3_bs_tag;
	
	/* TODO: figure out how to have multiple I2C bus controllers */
	
	/* for now we only use controller 1 */
	sc->sc_controller = 1;

	err = omap3_i2c_activate(dev);
	if (err)
		goto out;
	
	OMAP3_I2C_LOCK_INIT(sc);
	
	/* activate the interrupt */
	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
						 NULL, omap3_i2c_intr, sc, &sc->sc_irq_h);
	if (err) {
		OMAP3_I2C_LOCK_DESTROY(sc);
		goto out;
	}

	if ((sc->sc_iicbus = device_add_child(dev, "iicbus", -1)) == NULL)
		device_printf(dev, "could not allocate iicbus instance\n");

	/* probe and attach the iicbus */
	bus_generic_attach(dev);

	
out:
	if (err)
		omap3_i2c_deactivate(dev);
	return (err);
}



/**
 *	omap3_i2c_detach - detach function for the driver
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	
 *
 *	RETURNS:
 *	Always returns 0
 */
static int
omap3_i2c_detach(device_t dev)
{
	struct omap3_i2c_softc *sc = device_get_softc(dev);
	int rv;
	
	omap3_i2c_deactivate(dev);
	
	if (sc->sc_iicbus && (rv = device_delete_child(dev, sc->sc_iicbus)) != 0)
		return (rv);
	
	OMAP3_I2C_LOCK_DESTROY(sc);
	
	return (0);
}


/**
 *	omap3_i2c_activate - initialises and activates an I2C bus
 *	@dev: i2c device handle
 *	@num: the number of the I2C controller to activate; 1, 2 or 3
 *	
 *
 *	LOCKING:
 *	Assumed called in an atomic context.
 *
 *	RETURNS:
 *	nothing
 */
static int
omap3_i2c_activate(device_t dev)
{
	struct omap3_i2c_softc *sc = (struct omap3_i2c_softc*) device_get_softc(dev);
	unsigned int timeout = 0;
	int err = ENOMEM;
	int rid = 0;
	uint32_t membase;
	uint32_t memsize;
	uint32_t irqnum;
	uint32_t iclk;
	uint32_t fclk;
	uint16_t con_reg;
	
	
	/* Sort out the IRQ, memory addresses and clocks for the controller */
	switch (sc->sc_controller) {
		case 1:
			membase = OMAP35XX_I2C1_HWBASE;
			memsize = OMAP35XX_I2C1_SIZE;
			irqnum  = OMAP35XX_IRQ_I2C1;
			iclk    = OMAP3_MODULE_I2C1_ICLK;
			fclk    = OMAP3_MODULE_I2C1_FCLK;
			break;
		case 2:
			membase = OMAP35XX_I2C2_HWBASE;
			memsize = OMAP35XX_I2C2_SIZE;
			irqnum  = OMAP35XX_IRQ_I2C2;
			iclk    = OMAP3_MODULE_I2C2_ICLK;
			fclk    = OMAP3_MODULE_I2C2_FCLK;
			break;
		case 3:
			membase = OMAP35XX_I2C3_HWBASE;
			memsize = OMAP35XX_I2C3_SIZE;
			irqnum  = OMAP35XX_IRQ_I2C3;
			iclk    = OMAP3_MODULE_I2C3_ICLK;
			fclk    = OMAP3_MODULE_I2C3_FCLK;
			break;
		default:
			return(-EINVAL);
	}
	
	/* Map the register set for the I2C controller */
	if (bus_space_map(sc->sc_iotag, membase, memsize, 0, &sc->sc_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
		sc->sc_ioh = 0;
	}
	
	
	/* Allocate an IRQ resource for the I2C controller */
	sc->sc_irq_res = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, irqnum, irqnum,
							            1, RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		err = -ENOMEM;
		goto errout;
	}
	
	/*
	 * The following sequence is taken from the OMAP3530 technical reference
	 * 
	 * 1. Enable the functional and interface clocks (see Section 18.3.1.1.1).
	 */

	/* Set the EN_I2C1 bit (I2C 1 interface clock control) */
	omap3_prcm_enable_clk(iclk);
	
	/* Set the EN_I2C1 bit (I2C 1 functional clock control) */
	omap3_prcm_enable_clk(fclk);

	/* There seems to be a bug in the I2C reset mechanism, for some reason you
	 * need to disable the I2C module before issuing the reset and then enable
	 * it again after to detect the reset done.
	 *
	 * I found this out by looking at the Linux driver implementation, thanks
	 * linux guys!
	 */
	
	/* Disable the I2C controller */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, 0x0000);

	/* Issue a softreset to the controller */
	omap3_i2c_writew(sc, OMAP35XX_I2C_SYSC, 0x0002);

	/* Re-enable the module and then check for the reset done */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, I2C_CON_I2C_EN);

	while ((omap3_i2c_readw(sc, OMAP35XX_I2C_SYSS) & 0x01) == 0x00) {
		if (timeout++ > 100) {
			err = -EBUSY;
			goto errout;
		}
		DELAY(100);
	}

	/* Disable the I2C controller once again, now that the reset has finished */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, 0x0000);
	
	
	
	/* 2. Program the prescaler to obtain an approximately 12-MHz internal
	 *    sampling clock (I2Ci_INTERNAL_CLK) by programming the corresponding
	 *    value in the I2Ci.I2C_PSC[3:0] PSC field.
	 *    This value depends on the frequency of the functional clock (I2Ci_FCLK).
	 *    Because this frequency is 96MHz, the I2Ci.I2C_PSC[7:0] PSC field value
	 *    is 0x7.
	 */
	
	/* Program the prescaler to obtain an approximately 12-MHz internal
	 * sampling clock.
	 */
	omap3_i2c_writew(sc, OMAP35XX_I2C_PSC, 0x0017);

	
	/* 3. Program the I2Ci.I2C_SCLL[7:0] SCLL and I2Ci.I2C_SCLH[7:0] SCLH fields
	 *    to obtain a bit rate of 100K bps or 400K bps. These values depend on
	 *    the internal sampling clock frequency (see Table 18-12).
	 */
	 
	/* Set the bitrate to 100kbps */
	omap3_i2c_writew(sc, OMAP35XX_I2C_SCLL, 0x000d);
	omap3_i2c_writew(sc, OMAP35XX_I2C_SCLH, 0x000f);


	/* 4. (Optional) Program the I2Ci.I2C_SCLL[15:8] HSSCLL and
	 *    I2Ci.I2C_SCLH[15:8] HSSCLH fields to obtain a bit rate of 400K bps or
	 *    3.4M bps (for the second phase of HS mode). These values depend on the
	 *    internal sampling clock frequency (see Table 18-12).
	 *
	 * 5. (Optional) If a bit rate of 3.4M bps is used and the bus line
	 *    capacitance exceeds 45 pF, program the CONTROL.CONTROL_DEVCONF1[12]
	 *    I2C1HSMASTER bit for I2C1, the CONTROL.CONTROL_DEVCONF1[13]
	 *    I2C2HSMASTER bit for I2C2, or the CONTROL.CONTROL_DEVCONF1[14]
	 *    I2C3HSMASTER bit for I2C3.
	 */
	 

	/* 6. Configure the Own Address of the I2C controller by storing it in the
	 *    I2Ci.I2C_OA0 register. Up to four Own Addresses can be programmed in
	 *    the I2Ci.I2C_OAi registers (with I = 0, 1, 2, 3) for each I2C
	 *    controller.
	 *
	 * Note: For a 10-bit address, set the corresponding expand Own Address bit
	 * in the I2Ci.I2C_CON register.
	 */
	
	/* Driver currently always in single master mode so ignore this step */
	
	 
	 
	/* 7. Set the TX threshold (in transmitter mode) and the RX threshold (in
	 *    receiver mode) by setting the I2Ci.I2C_BUF[5:0]XTRSH field to (TX
	 *    threshold - 1) and the I2Ci.I2C_BUF[13:8]RTRSH field to (RX threshold
	 *    - 1), where the TX and RX thresholds are greater than or equal to 1.
	 */
	 
	/* Set the FIFO buffer threshold, note I2C1 & I2C2 have 8 byte FIFO, whereas
	 * I2C3 has 64 bytes.  Threshold set to 5 for now.
	 */
	omap3_i2c_writew(sc, OMAP35XX_I2C_BUF, 0x0404);
	
	
	/*
	 * 8. Take the I2C controller out of reset by setting the I2Ci.I2C_CON[15] 
	 *    I2C_EN bit to 1.
	 */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, I2C_CON_I2C_EN | I2C_CON_OPMODE_STD);



	/*
	 * To initialize the I2C controller, perform the following steps:
	 *
	 * 1. Configure the I2Ci.I2C_CON register:
	 *    · For master or slave mode, set the I2Ci.I2C_CON[10] MST bit (0: slave,
	 *      1: master).
	 *    · For transmitter or receiver mode, set the I2Ci.I2C_CON[9] TRX bit
	 *      (0: receiver, 1: transmitter).
	 */
	con_reg = omap3_i2c_readw(sc, OMAP35XX_I2C_CON);
	con_reg |= I2C_CON_MST;
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, con_reg);
	
	
	/* 2. If using an interrupt to transmit/receive data, set to 1 the
	 *    corresponding bit in the I2Ci.I2C_IE register (the I2Ci.I2C_IE[4]
	 *    XRDY_IE bit for the transmit interrupt, the I2Ci.I2C_IE[3] RRDY bit
	 *    for the receive interrupt).
	 */
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, I2C_IE_XRDY | I2C_IE_RRDY);
	
	  
	/* 3. If using DMA to receive/transmit data, set to 1 the corresponding bit
	 *    in the I2Ci.I2C_BUF register (the I2Ci.I2C_BUF[15] RDMA_EN bit for the
	 *    receive DMA channel, the I2Ci.I2C_BUF[7] XDMA_EN bit for the transmit
	 *    DMA channel).
	 */	
	
	/* not using DMA for now, so ignore this */
	
	
	return(0);
	
errout:
	return(err);
}


/**
 *	omap3_i2c_deactivate - deactivates the controller and releases resources
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	Assumed called in an atomic context.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_i2c_deactivate(device_t dev)
{
	struct omap3_i2c_softc *sc = device_get_softc(dev);
	uint32_t memsize;
	uint32_t iclk;
	uint32_t fclk;

	/* Sort out the IRQ, memory addresses and clocks for the controller */
	switch (sc->sc_controller) {
		case 1:
			memsize = OMAP35XX_I2C1_SIZE;
			iclk    = OMAP3_MODULE_I2C1_ICLK;
			fclk    = OMAP3_MODULE_I2C1_FCLK;
			break;
		case 2:
			memsize = OMAP35XX_I2C2_SIZE;
			iclk    = OMAP3_MODULE_I2C2_ICLK;
			fclk    = OMAP3_MODULE_I2C2_FCLK;
			break;
		case 3:
			memsize = OMAP35XX_I2C3_SIZE;
			iclk    = OMAP3_MODULE_I2C3_ICLK;
			fclk    = OMAP3_MODULE_I2C3_FCLK;
			break;
		default:
			return;
	}

	/* Disable the controller - cancel all transactions */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, 0x0000);
	
	/* Release the interrupt handler */
	if (sc->sc_irq_h) {
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_irq_h);
		sc->sc_irq_h = 0;
	}
	
	bus_generic_detach(sc->sc_dev);
	
	/* Unmap the I2C controller registers */
	if (sc->sc_ioh != 0) {
		bus_space_unmap(sc->sc_iotag, sc->sc_ioh, memsize);
		sc->sc_ioh = 0;
	}
	
	/* Release the IRQ resource */
	if (sc->sc_irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->sc_irq_res),
							 sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}

	
	/* Finally disable the functional and interface clocks */
	omap3_prcm_disable_clk(iclk);
	omap3_prcm_disable_clk(fclk);

	
	return;
}



/**
 *	omap3_i2c_reset - attach function for the driver
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	Called from timer context
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
omap3_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct omap3_i2c_softc *sc = device_get_softc(dev);
	uint16_t psc_reg, scll_reg, sclh_reg, con_reg;
	

	OMAP3_I2C_LOCK(sc);
	
	if (oldaddr)
		*oldaddr = sc->sc_i2c_addr;
	sc->sc_i2c_addr = addr;
	
	
	/* The header file doesn't actual tell you what speeds should be used for
	 * the 3 possible settings, so I'm going to go with the usual:
	 *
	 *    IIC_SLOW    => 100kbps
	 *    IIC_FAST    => 400kbps
	 *    IIC_FASTEST => 3.4Mbps
	 *
	 * I2Ci_INTERNAL_CLK = I2Ci_FCLK / (PSC + 1)
	 * I2Ci_INTERNAL_CLK = 96MHZ / (PSC + 1)
	 */
	switch (speed) {
		case IIC_FASTEST:
			psc_reg = 0x0004;
			scll_reg = 0x0811;
			sclh_reg = 0x0a13;
			break;

		case IIC_FAST:
			psc_reg = 0x0009;
			scll_reg = 0x0005;
			sclh_reg = 0x0007;
			break;
			
		case IIC_SLOW:
		case IIC_UNKNOWN:
		default:
			psc_reg = 0x0017;   
			scll_reg = 0x000D;
			sclh_reg = 0x000F;
			break;
	}

	/* First disable the controller while changing the clocks */
	con_reg = omap3_i2c_readw(sc, OMAP35XX_I2C_CON);
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, 0x0000);
	
	/* Program the prescaler */
	omap3_i2c_writew(sc, OMAP35XX_I2C_PSC, psc_reg);
	
	/* Set the bitrate */
	omap3_i2c_writew(sc, OMAP35XX_I2C_SCLL, scll_reg);
	omap3_i2c_writew(sc, OMAP35XX_I2C_SCLH, sclh_reg);
	
	/* Set the remote slave address */
	omap3_i2c_writew(sc, OMAP35XX_I2C_SA, addr);
	
	
	/* Enable the I2C module again */
	con_reg  = I2C_CON_I2C_EN;
	con_reg |= (speed == IIC_FASTEST) ? I2C_CON_OPMODE_HS : I2C_CON_OPMODE_STD;
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, con_reg);

	
	OMAP3_I2C_UNLOCK(sc);
	
	return 0;
}



/**
 *	omap3_i2c_intr - interrupt handler for the I2C module
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	Called from timer context
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static void
omap3_i2c_intr(void *arg)
{
	struct omap3_i2c_softc *sc = (struct omap3_i2c_softc*) arg;
	uint16_t status;
	
	status = omap3_i2c_readw(sc, OMAP35XX_I2C_STAT);
	if (status == 0)
		return;
		
	OMAP3_I2C_LOCK(sc);
	
	/* save the flags */
	sc->sc_stat_flags |= status;
		
	/* clear the status flags */
	omap3_i2c_writew(sc, OMAP35XX_I2C_STAT, status);
	
	/* wakeup the process the started the transaction */
	wakeup(sc);
	
	OMAP3_I2C_UNLOCK(sc);
	
	return;
}


/**
 *	omap3_i2c_wait - waits for the specific event to occur
 *	@sc: i2c driver context
 *	@flags: the event(s) to wait on, this is a bitmask of the I2C_STAT_??? flags
 *	@statp: if not null will contain the status flags upon return
 *	@timo: the number of ticks to wait
 *
 *	
 *
 *	LOCKING:
 *	The driver context must be locked before calling this function. Internally
 *	the function sleeps, releasing the lock as it does so, however the lock is
 *	always retaken before this function returns.
 *
 *	RETURNS:
 *	0 if the event(s) were tripped within timeout period
 *	-EBUSY if timedout waiting for the events
 *	-ENXIO if a NACK event was received
 */
static int
omap3_i2c_wait(struct omap3_i2c_softc *sc, uint16_t flags, uint16_t *statp, int timo)
{
	int waittime = timo;
	int start_ticks = ticks;
	int rc;
	

	OMAP3_I2C_ASSERT_LOCKED(sc);

	/* check if the condition has already occured, the interrupt routine will
	 * clear the status flags.
	 */
	if ((sc->sc_stat_flags & flags) == 0) {
	
		/* condition(s) haven't occured so sleep on the IRQ */
		while (waittime > 0) {
		
			rc = mtx_sleep(sc, &sc->sc_mtx, 0, "I2Cwait", waittime);
			if (rc == EWOULDBLOCK) {
				/* timed-out, simply break out of the loop */
				break;
			} else {
				/* IRQ has been tripped, but need to sanity check we have the
				 * right events in the status flag.
				 */
				if ((sc->sc_stat_flags & flags) != 0)
					break;
				
				/* event hasn't been tripped so wait some more */
				waittime -= (ticks - start_ticks);
				start_ticks = ticks;
			}
		}
	}

	/* copy the actual status bits */
	if (statp != NULL)
		*statp = sc->sc_stat_flags;
	
	/* return the status found */
	if ((sc->sc_stat_flags & flags) != 0)
		rc = 0;
	else
		rc = -EBUSY;
		
	/* clear the flags set by the interrupt handler */
	sc->sc_stat_flags = 0;
	
	return(rc);
}


/**
 *	omap3_i2c_wait_for_free_bus - waits for the bus to become free
 *	@sc: i2c driver context
 *	@timo: the time to wait for the bus to become free
 *
 *	
 *
 *	LOCKING:
 *	The driver context must be locked before calling this function. Internally
 *	the function sleeps, releasing the lock as it does so, however the lock is
 *	always taken before this function returns.
 *
 *	RETURNS:
 *	0 if the event(s) were tripped within timeout period
 *	-EBUSY if timedout waiting for the events
 *	-ENXIO if a NACK event was received
 */
static int
omap3_i2c_wait_for_free_bus(struct omap3_i2c_softc *sc, int timo)
{
	/* check if the bus is free, BB bit = 0 */
	if ((omap3_i2c_readw(sc, OMAP35XX_I2C_STAT) & I2C_STAT_BB) == 0)
		return 0;
	
	/* enable bus free interrupts */
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, I2C_IE_BF);

	/* wait for the bus free interrupt to be tripped */
	return omap3_i2c_wait(sc, I2C_STAT_BF, NULL, timo);
}


/**
 *	omap3_i2c_read_bytes - attempts to perform a read operation
 *	@sc: i2c driver context
 *	@buf: buffer to hold the received bytes
 *	@len: the number of bytes to read
 *
 *	This function assumes the slave address is already set
 *
 *	LOCKING:
 *	The context lock should be held before calling this function
 *
 *	RETURNS:
 *	0 on function succeeded
 *	-EINVAL if invalid message is passed as an arg
 */
static int
omap3_i2c_read_bytes(struct omap3_i2c_softc *sc, uint8_t *buf, uint16_t len)
{
	int      timo = (hz / 4);
	int      err = 0;
	uint16_t con_reg;
	uint16_t events;
	uint16_t status;
	uint32_t amount;
	uint32_t sofar = 0;
	uint32_t i;

	/* wait for the bus to become free */
	err = omap3_i2c_wait_for_free_bus(sc, timo);
	if (err != 0)
		return(err);

	/* set the events to wait for */
	events = I2C_IE_RDR |   /* Receive draining interrupt */
	         I2C_IE_RRDY |  /* Receive Data Ready interrupt */
	         I2C_IE_ARDY |  /* Register Access Ready interrupt */
	         I2C_IE_NACK |  /* No Acknowledgment interrupt */
	         I2C_IE_AL;

	/* enable interrupts for the events we want*/
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, events);

	/* write the number of bytes to read */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CNT, len);

	/* clear the write bit and initiate the read transaction. Setting the STT
	 * (start) bit initiates the transfer.
	 */
	con_reg = omap3_i2c_readw(sc, OMAP35XX_I2C_CON);
	con_reg &= ~I2C_CON_TRX;
	con_reg |=  I2C_CON_MST | I2C_CON_STT | I2C_CON_STP;
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, con_reg);
	
	
	/* reading loop */
	while (1) {
	
		/* wait for an event */
		err = omap3_i2c_wait(sc, events, &status, timo);
		if (err != 0)
			break;
		
		/* check for the error conditions */
		if (status & I2C_STAT_NACK) {
			/* no ACK from slave */
			printf("[BRG] %s : %d : NACK\n", __func__, __LINE__);
			err = -ENXIO;
			break;
		}
		if (status & I2C_STAT_AL) {
			/* arbitration lost */
			printf("[BRG] %s : %d : Arbitration lost\n", __func__, __LINE__);
			err = -ENXIO;
			break;
		}

		/* check if we have finished */
		if (status & I2C_STAT_ARDY) {
			/* register access ready - transaction complete basically */
			printf("[BRG] %s : %d : ARDY transaction complete\n", __func__, __LINE__);
			err = 0;
			break;
		}



		/* read some data */
		if (status & I2C_STAT_RDR) {
			/* Receive draining interrupt - last data received */
			printf("[BRG] %s : %d : Receive draining interrupt\n", __func__, __LINE__);
			
			/* get the number of bytes in the FIFO */
			amount = omap3_i2c_readw(sc, OMAP35XX_I2C_BUFSTAT);
			amount >>= 8;
			amount &= 0x3f;
		}
		else if (status & I2C_STAT_RRDY) {
			/* Receive data ready interrupt - enough data received */
			printf("[BRG] %s : %d : Receive data ready interrupt\n", __func__, __LINE__);
			
			/* get the number of bytes in the FIFO */
			amount = omap3_i2c_readw(sc, OMAP35XX_I2C_BUF);
			amount >>= 8;
			amount &= 0x3f;
			amount += 1;
		}
		
		/* sanity check we haven't overwritten the array */
		if ((sofar + amount) > len) {
			printf("[BRG] %s : %d : to many bytes to read\n", __func__, __LINE__);
			amount = (len - sofar);
		}
			
		/* read the bytes from the fifo */
		for (i = 0; i < amount; i++) {
			buf[sofar++] = (uint8_t)(omap3_i2c_readw(sc, OMAP35XX_I2C_DATA) & 0xff);
		}
		
		/* attempt to clear the receive ready bits */
		omap3_i2c_writew(sc, OMAP35XX_I2C_STAT, I2C_STAT_RDR | I2C_STAT_RRDY);
	}
	
	/* reset the registers regardless if there was an error or not */
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, 0x0000);
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, I2C_CON_I2C_EN | I2C_CON_MST | I2C_CON_STP);
	
	return(err);
}


/**
 *	omap3_i2c_write_bytes - attempts to perform a read operation
 *	@sc: i2c driver context
 *	@buf: buffer containing the bytes to write
 *	@len: the number of bytes to write
 *
 *	This function assumes the slave address is already set
 *
 *	LOCKING:
 *	The context lock should be held before calling this function
 *
 *	RETURNS:
 *	0 on function succeeded
 *	-EINVAL if invalid message is passed as an arg
 */
static int
omap3_i2c_write_bytes(struct omap3_i2c_softc *sc, const uint8_t *buf, uint16_t len)
{
	int      timo = (hz / 4);
	int      err = 0;
	uint16_t con_reg;
	uint16_t events;
	uint16_t status;
	uint32_t amount;
	uint32_t sofar = 0;
	uint32_t i;


	/* wait for the bus to become free */
	err = omap3_i2c_wait_for_free_bus(sc, timo);
	if (err != 0)
		return(err);


	/* set the events to wait for */
	events = I2C_IE_XDR |   /* Transmit draining interrupt */
	         I2C_IE_XRDY |  /* Transmit Data Ready interrupt */
	         I2C_IE_ARDY |  /* Register Access Ready interrupt */
	         I2C_IE_NACK |  /* No Acknowledgment interrupt */
	         I2C_IE_AL;

	/* enable interrupts for the events we want*/
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, events);

	/* write the number of bytes to write */
	omap3_i2c_writew(sc, OMAP35XX_I2C_CNT, len);

	/* set the write bit and initiate the write transaction. Setting the STT
	 * (start) bit initiates the transfer.
	 */
	con_reg = omap3_i2c_readw(sc, OMAP35XX_I2C_CON);
	con_reg |= I2C_CON_TRX | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP;
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, con_reg);


	/* writing loop */
	while (1) {
	
		/* wait for an event */
		err = omap3_i2c_wait(sc, events, &status, timo);
		if (err != 0)
			break;
		

		/* check for the error conditions */
		if (status & I2C_STAT_NACK) {
			/* no ACK from slave */
			printf("[BRG] %s : %d : NACK\n", __func__, __LINE__);
			err = -ENXIO;
			break;
		}
		if (status & I2C_STAT_AL) {
			/* arbitration lost */
			printf("[BRG] %s : %d : Arbitration lost\n", __func__, __LINE__);
			err = -ENXIO;
			break;
		}

		/* check if we have finished */
		if (status & I2C_STAT_ARDY) {
			/* register access ready - transaction complete basically */
			printf("[BRG] %s : %d : ARDY transaction complete\n", __func__, __LINE__);
			err = 0;
			break;
		}



		/* read some data */
		if (status & I2C_STAT_XDR) {
			/* Receive draining interrupt - last data received */
			printf("[BRG] %s : %d : Transmit draining interrupt\n", __func__, __LINE__);
			
			/* get the number of bytes in the FIFO */
			amount = omap3_i2c_readw(sc, OMAP35XX_I2C_BUFSTAT);
			amount &= 0x3f;
		}
		else if (status & I2C_STAT_XRDY) {
			/* Receive data ready interrupt - enough data received */
			printf("[BRG] %s : %d : Transmit data ready interrupt\n", __func__, __LINE__);
			
			/* get the number of bytes in the FIFO */
			amount = omap3_i2c_readw(sc, OMAP35XX_I2C_BUF);
			amount &= 0x3f;
			amount += 1;
		}
		
		/* sanity check we haven't overwritten the array */
		if ((sofar + amount) > len) {
			printf("[BRG] %s : %d : to many bytes to write\n", __func__, __LINE__);
			amount = (len - sofar);
		}
			
		/* write the bytes from the fifo */
		for (i = 0; i < amount; i++) {
			omap3_i2c_writew(sc, OMAP35XX_I2C_DATA, buf[sofar++]);
		}

		/* attempt to clear the transmit ready bits */
		omap3_i2c_writew(sc, OMAP35XX_I2C_STAT, I2C_STAT_XDR | I2C_STAT_XRDY);
	}
	
	/* reset the registers regardless if there was an error or not */
	omap3_i2c_writew(sc, OMAP35XX_I2C_IE, 0x0000);
	omap3_i2c_writew(sc, OMAP35XX_I2C_CON, I2C_CON_I2C_EN | I2C_CON_MST | I2C_CON_STP);

	return(err);
}


/**
 *	omap3_i2c_transfer - called to perform the transfer
 *	@dev: i2c device handle
 *	@msgs: the messages to send/receive
 *	@nmsgs: the number of messages in the msgs array
 *
 *
 *	LOCKING:
 *	Internally locked
 *
 *	RETURNS:
 *	0 on function succeeded
 *	-EINVAL if invalid message is passed as an arg
 */
static int
omap3_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct omap3_i2c_softc *sc = device_get_softc(dev);
	int err = 0;
	uint32_t i;
	uint16_t len;
	uint8_t *buf;
	
	OMAP3_I2C_LOCK(sc);
	
	for (i = 0; i < nmsgs; i++) {

		len = msgs[i].len;
		buf = msgs[i].buf;
		
		/* zero byte transfers aren't allowed */
		if (len == 0 || buf == NULL) {
			err = -EINVAL;
			goto out;
		}
		
		/* set the slave address */
		omap3_i2c_writew(sc, OMAP35XX_I2C_SA, msgs[i].slave);
		

		/* perform the read or write */
		if (msgs[i].flags & IIC_M_RD) {
			err = omap3_i2c_read_bytes(sc, buf, len);
		} else {
			err = omap3_i2c_write_bytes(sc, buf, len);
		}

	}
	
out:;
	OMAP3_I2C_UNLOCK(sc);

	return(err);
}


/**
 *	omap3_i2c_callback - not sure about this one
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	Called from timer context
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
omap3_i2c_callback(device_t dev, int index, caddr_t data)
{
	int error = 0;
	
	switch (index) {
		case IIC_REQUEST_BUS:
			break;
			
		case IIC_RELEASE_BUS:
			break;
			
		default:
			error = EINVAL;
	}
	
	return(error);
}





static device_method_t omap3_i2c_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		omap3_i2c_probe),
	DEVMETHOD(device_attach,	omap3_i2c_attach),
	DEVMETHOD(device_detach,	omap3_i2c_detach),
	
	/* iicbus interface */
	DEVMETHOD(iicbus_callback,	omap3_i2c_callback),
	DEVMETHOD(iicbus_reset,		omap3_i2c_reset),
	DEVMETHOD(iicbus_transfer,	omap3_i2c_transfer),
	{ 0, 0 }
};

static driver_t omap3_i2c_driver = {
	"omap3_i2c",
	omap3_i2c_methods,
	sizeof(struct omap3_i2c_softc),
};

DRIVER_MODULE(omap3_i2c, omap3, omap3_i2c_driver, omap3_i2c_devclass, 0, 0);
DRIVER_MODULE(iicbus, omap3_i2c, iicbus_driver, iicbus_devclass, 0, 0);
MODULE_DEPEND(omap3_i2c, omap3_prcm, 1, 1, 1);
MODULE_DEPEND(omap3_i2c, omap3_clk, 1, 1, 1);
MODULE_DEPEND(omap3_i2c, iicbus, 1, 1, 1);
