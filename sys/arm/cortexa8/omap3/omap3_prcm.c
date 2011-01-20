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
 * Power, Reset and Clock Managment Module
 *
 * This is a very simple driver wrapper around the PRCM set of registers in
 * the OMAP3 chip. It allows you to turn on and off things like the functional
 * and interface clocks to the various on-chip modules.
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_prcm.h>


/**
 *	Structure that stores the driver context.
 *
 *	This structure is allocated during driver attach, it is not designed to be
 *	deallocated and a pointer to it is stored globally (g_omap3_prcm_softc).
 */
struct omap3_prcm_softc {
	device_t			sc_dev;
	
	/* the bus handle/tag for the clock management (CM) part of the reg set */
	bus_space_tag_t		sc_cm_iot;
	bus_space_handle_t	sc_cm_ioh;

	/* the bus handle/tag for the power & resource management (PRM) part of the
	 * register set */
	bus_space_tag_t		sc_prm_iot;
	bus_space_handle_t	sc_prm_ioh;
	
	struct mtx			sc_mtx;
};

static struct omap3_prcm_softc *g_omap3_prcm_softc = NULL;

/**
 *	Macros for driver mutex locking
 */
#define OMAP3_PRCM_LOCK(_sc)			mtx_lock(&(_sc)->sc_mtx)
#define	OMAP3_PRCM_UNLOCK(_sc)			mtx_unlock(&(_sc)->sc_mtx)
#define OMAP3_PRCM_LOCK_INIT(_sc) \
	mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), \
	         "omap3_prcm", MTX_SPIN)
#define OMAP3_PRCM_LOCK_DESTROY(_sc)	mtx_destroy(&_sc->sc_mtx);
#define OMAP3_PRCM_ASSERT_LOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_OWNED);
#define OMAP3_PRCM_ASSERT_UNLOCKED(_sc)	mtx_assert(&_sc->sc_mtx, MA_NOTOWNED);



/**
 *	omap3_prcm_setup_dpll5 - probe function for the driver
 *	@dev: prcm device handle
 *
 *	Simply sets the name of the driver module.
 *
 *	LOCKING:
 *	None
 *
 *	RETURNS:
 *	Always returns 0
 */
static int
omap3_prcm_setup_dpll5(struct omap3_prcm_softc *sc, uint32_t mul, uint32_t div)
{
	uint32_t val;
	
	/* DPPL5 uses DPLL5_ALWON_FCLK as it's reference clock, this is just SYS_CLK
	 * which on the beagleboard is 13MHz.
	 */
	
	/* Set the multipler and divider values for the PLL.  We want 120Mhz so take
	 * the system clock (13Mhz) divide by that then multiple by 120.
	 */
	val = ((120 & 0x7ff) << 8) | ((13 - 1) & 0x7f);
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, OMAP35XX_CM_CLKSEL4_PLL, val);
	
	/* This is the clock divider from the PLL into the 120Mhz clock supplied to
	 * the USB module. */
	val = 0x01;
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, OMAP35XX_CM_CLKSEL5_PLL, val);

	/* PERIPH2_DPLL_FREQSEL = 0x7   (1.75 MHz—2.1 MHz)
	 * EN_PERIPH2_DPLL = 0x7        (Enables the DPLL5 in lock mode)
	 */
	val = (7 << 4) | (7 << 0);
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, OMAP35XX_CM_CLKEN2_PLL, val);


	/* Disable auto-idle */
	val = 0x0;
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, OMAP35XX_CM_AUTOIDLE2_PLL, val);



	/* Wait until the DPLL5 is locked and there is clock activity */
	while (((val = bus_space_read_4(sc->sc_cm_iot, sc->sc_cm_ioh,
	       OMAP35XX_CM_IDLEST2_CKGEN)) & 0x01) == 0x00) {
		printf("OMAP35XX_CM_IDLEST2_PLL = 0x%08x\n", val);
	}

	return 0;
}


/**
 *	omap3_prcm_probe - probe function for the driver
 *	@dev: prcm device handle
 *
 *	Simply sets the name of the driver module.
 *
 *	LOCKING:
 *	None
 *
 *	RETURNS:
 *	Always returns 0
 */
static int
omap3_prcm_probe(device_t dev)
{
	device_set_desc(dev, "TI OMAP3530 Power, Reset and Clock Management");
	return (0);
}

/**
 *	omap3_prcm_attach - attach function for the driver
 *	@dev: prcm device handle
 *
 *	Allocates and sets up the driver context, this simply entails creating a
 *	bus mappings for the PRCM register set.
 *
 *	LOCKING:
 *	None
 *
 *	RETURNS:
 *	Always returns 0
 */
static int
omap3_prcm_attach(device_t dev)
{
	struct omap3_prcm_softc *sc = device_get_softc(dev);
	
	g_omap3_prcm_softc = sc;
	
	sc->sc_dev = dev;
	sc->sc_cm_iot = sc->sc_prm_iot = &omap3_bs_tag;
	
	OMAP3_PRCM_LOCK_INIT(sc);
	
	/* Map in the clock management register set */
	if (bus_space_map(sc->sc_cm_iot, OMAP35XX_CM_HWBASE, OMAP35XX_CM_SIZE,
					  0, &sc->sc_cm_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}
	
	/* Map in the power and reset register set */
	if (bus_space_map(sc->sc_prm_iot, OMAP35XX_PRM_HWBASE, OMAP35XX_PRM_SIZE,
					  0, &sc->sc_prm_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}

	printf("[BRG] %s : %d : sc->sc_cm_iot = %p : sc->sc_cm_ioh = 0x%08x\n",
		__func__, __LINE__, sc->sc_cm_iot, (uint32_t)sc->sc_cm_ioh);


	/* Setup DPLL5, this is the 120MHz clock used by the USB module */
	omap3_prcm_setup_dpll5(sc, 120, 12);
	
	return (0);
}

static device_method_t g_omap3_prcm_methods[] = {
	DEVMETHOD(device_probe, omap3_prcm_probe),
	DEVMETHOD(device_attach, omap3_prcm_attach),
	{0, 0},
};

static driver_t g_omap3_prcm_driver = {
	"omap3_prcm",
	g_omap3_prcm_methods,
	sizeof(struct omap3_prcm_softc),
};
static devclass_t g_omap3_prcm_devclass;

DRIVER_MODULE(omap3_prcm, omap3, g_omap3_prcm_driver, g_omap3_prcm_devclass, 0, 0);
MODULE_VERSION(omap3_prcm, 1);




/**
 *	omap3_prcm_disable_clk - disables a clock for a particular module
 *	@module: identifier for the module to disable, see omap3_prcm.h for a list
 *	         of possible modules.
 *	         Example: OMAP3_MODULE_MMC1_ICLK or OMAP3_MODULE_GPTIMER10_FCLK.
 *	
 *	This function can enable either a functional or interface clock, the module
 *	param defines the clock that is going to be enabled.
 *
 *	LOCKING:
 *	Internally locks the driver context
 *
 *	RETURNS:
 *	Always returns 0
 */
int
omap3_prcm_disable_clk(unsigned int module)
{
	struct omap3_prcm_softc *sc = g_omap3_prcm_softc;
	uint32_t val;
	uint32_t off;
	uint32_t mask;
	
	if (sc == NULL) {
		panic("%s: PRCM module not setup", __func__);
	}
	
	off = OMAP3_MODULE_REG_OFFSET(module);
	mask = 1UL << OMAP3_MODULE_REG_BIT(module);
	
	OMAP3_PRCM_LOCK(sc);
	
	val = bus_space_read_4(sc->sc_cm_iot, sc->sc_cm_ioh, off);
	val &= ~mask;
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, off, val);
	
	OMAP3_PRCM_UNLOCK(sc);
	return (0);
}


/**
 *	omap3_prcm_enable_clk - enables a clock for a particular module
 *	@module: identifier for the module to enable, see omap3_prcm.h for a list
 *	         of possible modules.
 *	         Example: OMAP3_MODULE_MMC1_ICLK or OMAP3_MODULE_GPTIMER10_FCLK.
 *	
 *	This function can enable either a functional or interface clock, the module
 *	param defines the clock that is going to be enabled.
 *
 *	LOCKING:
 *	Internally locks the driver context
 *
 *	RETURNS:
 *	Always returns 0
 */
int
omap3_prcm_enable_clk(unsigned int module)
{
	struct omap3_prcm_softc *sc = g_omap3_prcm_softc;
	uint32_t val;
	uint32_t off;
	uint32_t mask;

	if (sc == NULL) {
		panic("%s: PRCM module not setup", __func__);
	}

	off = OMAP3_MODULE_REG_OFFSET(module);
	mask = 1UL << OMAP3_MODULE_REG_BIT(module);
	
	OMAP3_PRCM_LOCK(sc);
	
	val = bus_space_read_4(sc->sc_cm_iot, sc->sc_cm_ioh, off);
	val |= mask;
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, off, val);
	
	OMAP3_PRCM_UNLOCK(sc);

	return (0);
}



/**
 *	omap3_prcm_accessible - checks if a module is accessible
 *	@module: identifier for the module to check, see omap3_prcm.h for a list
 *	         of possible modules.
 *	         Example: OMAP3_MODULE_MMC1
 *	
 *	
 *
 *	LOCKING:
 *	Internally locks the driver context
 *
 *	RETURNS:
 *	Always returns 0
 */
int
omap3_prcm_accessible(unsigned int module)
{
	struct omap3_prcm_softc *sc = g_omap3_prcm_softc;
	uint32_t val;
	uint32_t off;
	uint32_t mask;

	if (sc == NULL) {
		panic("%s: PRCM module not setup", __func__);
	}

	off = OMAP3_MODULE_REG_OFFSET(module);
	mask = 1UL << OMAP3_MODULE_REG_BIT(module);

	val = bus_space_read_4(sc->sc_cm_iot, sc->sc_cm_ioh, off);

	if (val & mask) {
		return 1;
	} else {
		return 0;
	}

}


/**
 *	omap3_prcm_set_gptimer_clksrc - sets the clock source for a GPTIMER
 *	@timer: the number of the timer to set the reference clock for
 *	@sys_clk: if non-zero use the SYSCLK
 *	
 *	Currently you can only change the clock source for GPTIMER10 and 
 *	GPTIMER11.
 *
 *	LOCKING:
 *	Internally locks the driver context
 *
 *	RETURNS:
 *	0 on success
 *	-EINVAL if timer is not 10 or 11
 */
int
omap3_prcm_set_gptimer_clksrc(int timer, int sys_clk)
{
	struct omap3_prcm_softc *sc = g_omap3_prcm_softc;
	uint32_t val;
	uint32_t off;
	uint32_t mask;
	
	if (sc == NULL) {
		panic("%s: PRCM module not setup", __func__);
	}
	
	switch (timer) {
		case 10:
			off = OMAP35XX_CM_CLKSEL_CORE;
			mask = (1UL << 6);
			break;
		case 11:
			off = OMAP35XX_CM_CLKSEL_CORE;
			mask = (1UL << 7);
			break;
		default:
			return (-EINVAL);
	}
	
	OMAP3_PRCM_LOCK(sc);
	
	val = bus_space_read_4(sc->sc_cm_iot, sc->sc_cm_ioh, off);
	if (sys_clk)
		val |= mask;
	else
		val &= ~mask;
	bus_space_write_4(sc->sc_cm_iot, sc->sc_cm_ioh, off, val);
	
	OMAP3_PRCM_UNLOCK(sc);
	return (0);
}



