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
#include <sys/reboot.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/interrupt.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/kdb.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>

#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/bus.h>
#include <machine/bus.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_gpio.h>


struct omap3_softc {
	device_t           sc_dev;
	bus_space_tag_t    sc_iot;
	
	bus_space_handle_t sc_intr_ioh;		/* bus space handle, i.e. address of irq regs */
	
	struct rman        sc_irq_rman;
	struct rman        sc_mem_rman;
	bus_dma_tag_t      sc_dmat;
};

struct	omap3_softc *g_omap3_softc = NULL;

static int	omap3_probe(device_t);
static void	omap3_identify(driver_t *, device_t);
static int	omap3_attach(device_t);





/**
 * cpu_reset
 *
 * Called by the system to reset the CPU - obvious really
 *
 */
void
cpu_reset()
{
	panic("%s: unimplemented", __func__);
	
	cpu_halt();
	while (1);
}



/**
 * omap3_install_gpio_kdb
 *
 * 
 *
 */
static void
omap3_intr_gpio_kdb(unsigned int pin, unsigned int datain, void *data)
{
#if defined(KDB)
	kdb_enter(KDB_WHY_BREAK, "KDB Button Pressed");
#endif
}


/**
 * omap3_install_gpio_kdb
 *
 * 
 *
 */
static void
omap3_install_gpio_kdb(void)
{
	int rc;
	
	if (omap3_gpio_request(7, "USER") == 0) {
	
		/* Set the pin as an input */
		rc = omap3_gpio_direction_input(7);
		if (rc != 0)
			printf("[BRG] failed to set GPIO pin as an input\n");
		
		/* Enable the debounce circuit */
		rc = omap3_gpio_pin_debounce(7, 1, 500);
		if (rc != 0)
			printf("[BRG] failed to set GPIO debounce\n");
		
		/* Install an intterupt on GPIO for triggering the KDB */
		rc = omap3_gpio_pin_intr(7, GPIO_TRIGGER_FALLING, omap3_intr_gpio_kdb,
		                         NULL);
		if (rc != 0)
			printf("[BRG] failed to set GPIO interrupt trigger\n");
			
		printf("[BRG] installed GPIO trigger for KDB\n");
	}
}


/**
 * omap3_identify
 *
 * 
 *
 */
static void
omap3_identify(driver_t *driver, device_t parent)
{
	BUS_ADD_CHILD(parent, 0, "omap3", 0);
}

/**
 *	omap3_probe - driver probe function
 *	@dev: the root device
 * 
 *	Simply sets the name of this base driver
 *
 */
static int
omap3_probe(device_t dev)
{
	device_set_desc(dev, "TI OMAP3530");
	return (0);
}

/**
 * omap3_attach
 *
 * 
 *
 */
static int
omap3_attach(device_t dev)
{
	struct omap3_softc *sc;
	
	//device_printf(dev, "%b\n", omap3_read_feature_bits(), EXP_FCTRL_BITS);
	

	sc = device_get_softc(dev);
	sc->sc_iot = &omap3_bs_tag;

	KASSERT(g_omap3_softc == NULL, ("%s called twice?", __func__));
	g_omap3_softc = sc;
	
	/* Disable interrupts and config the mask and steering */
	// intr_enabled = 0;
	// omap3_set_intrmask();
	// omap3_set_intrsteer();
	
	/* install an interrupt post filter */
	arm_post_filter = omap3_post_filter_intr;
	
/*	
	if (bus_space_map(sc->sc_iot, IXP425_GPIO_HWBASE, IXP425_GPIO_SIZE,
					  0, &sc->sc_gpio_ioh))
		panic("%s: unable to map GPIO registers", __func__);
	if (bus_space_map(sc->sc_iot, IXP425_EXP_HWBASE, IXP425_EXP_SIZE,
					  0, &sc->sc_exp_ioh))
		panic("%s: unable to map Expansion Bus registers", __func__);
*/
	
	/* Ensure the IRQ control registers are mapped in, we use direct pointers
	 * for accessing those for speed ... 
	 */
	if (bus_space_map(sc->sc_iot, OMAP35XX_INTCPS_HWBASE, OMAP35XX_INTCPS_SIZE,
					  0, &sc->sc_intr_ioh))
		panic("%s: unable to map IRQ registers", __func__);
	
	printf("[BRG] %s : line %d\n", __func__, __LINE__);
	
	
	/* TODO: Investigate */
	if (bus_dma_tag_create(NULL, 1, 0, BUS_SPACE_MAXADDR_32BIT,
						   BUS_SPACE_MAXADDR, NULL, NULL,  0xffffffff, 0xff, 0xffffffff, 0, 
						   NULL, NULL, &sc->sc_dmat))
		panic("%s: failed to create dma tag", __func__);
	
	
	/* Setup the resource manager for IRQs (nb: 96 possible IRQs) */
	sc->sc_irq_rman.rm_type = RMAN_ARRAY;
	sc->sc_irq_rman.rm_descr = "OMAP3 IRQs";
	if (rman_init(&sc->sc_irq_rman) != 0 ||
	    rman_manage_region(&sc->sc_irq_rman, 0, 96) != 0)
		panic("%s: failed to set up IRQ rman", __func__);
	
	/* Setup the memory resource manager */
	sc->sc_mem_rman.rm_type = RMAN_ARRAY;
	sc->sc_mem_rman.rm_descr = "OMAP3 Memory";
	if (rman_init(&sc->sc_mem_rman) != 0 ||
	    rman_manage_region(&sc->sc_mem_rman, 0, ~0) != 0)
		panic("%s: failed to set up memory rman", __func__);

	
	/* Add the PRCM driver to the list of modules */
	BUS_ADD_CHILD(dev, 0, "omap3_prcm", 0);

	/* Add the SCM driver to the list of modules */
	BUS_ADD_CHILD(dev, 0, "omap3_scm", 0);

	/* Add the tick clock to the list of modules initialised */
	BUS_ADD_CHILD(dev, 0, "omap3_clk", 0);
	
	/* Add the GPIO module */
	BUS_ADD_CHILD(dev, 0, "omap3_gpio", 0);

	/* Add the DMA driver to the list of modules initialised */
	BUS_ADD_CHILD(dev, 0, "omap3_dma", 0);
	
	/* Add the MMC/SD/SDIO driver to the list of modules */
	BUS_ADD_CHILD(dev, 0, "omap3_mmc", 0);
	
	/* Add the I2C controller driver to the list of modules */
	BUS_ADD_CHILD(dev, 0, "omap3_i2c", 0);
	
	/* Add the USB EHCI driver to the list of modules */
	BUS_ADD_CHILD(dev, 0, "ehci", 0);

		
	/* attach wired devices via hints */
	bus_enumerate_hinted_children(dev);
	
	bus_generic_probe(dev);
	bus_generic_attach(dev);
	enable_interrupts(I32_bit | F32_bit);
	
	/* install a GPIO interrupt handler so when the user button
	 * is pressed the kernel debugger is started - temporary debug
	 */
	omap3_install_gpio_kdb();
	
	return (0);
}


/**
 * omap3_hinted_child
 *
 * 
 *
 */
static void
omap3_hinted_child(device_t bus, const char *dname, int dunit)
{
	device_t child;
	struct omap3_ivar *ivar;
	
	child = BUS_ADD_CHILD(bus, 0, dname, dunit);
	ivar = OMAP3_IVAR(child);
	resource_int_value(dname, dunit, "addr", &ivar->addr);
	resource_int_value(dname, dunit, "irq", &ivar->irq);
}

/**
 * omap3_add_child
 *
 * 
 *
 */
static device_t
omap3_add_child(device_t dev, int order, const char *name, int unit)
{
	device_t child;
	struct omap3_ivar *ivar;
	
	child = device_add_child_ordered(dev, order, name, unit);
	if (child == NULL)
		return NULL;
	
	ivar = malloc(sizeof(struct omap3_ivar), M_DEVBUF, M_NOWAIT);
	if (ivar == NULL) {
		device_delete_child(dev, child);
		return NULL;
	}
	
	ivar->addr = 0;
	ivar->irq = -1;
	device_set_ivars(child, ivar);

	return child;
}

/**
 * omap3_read_ivar
 *
 * 
 *
 */
static int
omap3_read_ivar(device_t bus, device_t child, int which, uintptr_t *result)
{
	struct omap3_ivar *ivar = OMAP3_IVAR(child);
	
	switch (which) {
		case OMAP3_IVAR_ADDR:
			if (ivar->addr != 0) {
				*(uint32_t *)result = ivar->addr;
				return 0;
			}
			break;
		case OMAP3_IVAR_IRQ:
			if (ivar->irq != -1) {
				*(int *)result = ivar->irq;
				return 0;
			}
			break;
	}
	return EINVAL;
}



/*
 * NB: This table handles P->V translations for regions setup with
 * static mappings in initarm.  This is used solely for calls to
 * bus_alloc_resource_any; anything done with bus_space_map is
 * handled elsewhere and does not require an entry here.
 *
 * XXX this table is also used by uart_cpu_getdev via getvbase
 *    (hence the public api)
 */
struct hwvtrans {
	uint32_t	hwbase;
	uint32_t	size;
	uint32_t	vbase;
};

static const struct hwvtrans *
omap3_gethwvtrans(uint32_t hwbase, uint32_t size)
{
	static const struct hwvtrans hwvtrans[] = {
	    /* The L3 mapping for some IO devices */
	    { .hwbase	= OMAP35XX_L3_HWBASE,
			.size 	= OMAP35XX_L3_SIZE,
			.vbase	= OMAP35XX_L3_VBASE },
		/* The three main L4 mappings for IO devices */
	    { .hwbase	= OMAP35XX_L4_CORE_HWBASE,
			.size 	= OMAP35XX_L4_CORE_SIZE,
			.vbase	= OMAP35XX_L4_CORE_VBASE },
	    { .hwbase	= OMAP35XX_L4_WAKEUP_HWBASE,
			.size 	= OMAP35XX_L4_WAKEUP_SIZE,
			.vbase	= OMAP35XX_L4_WAKEUP_VBASE },
	    { .hwbase	= OMAP35XX_L4_PERIPH_HWBASE,
			.size 	= OMAP35XX_L4_PERIPH_SIZE,
			.vbase	= OMAP35XX_L4_PERIPH_VBASE },
	};
	int i;
	
	for (i = 0; i < sizeof hwvtrans / sizeof *hwvtrans; i++) {
		if (hwbase >= hwvtrans[i].hwbase &&
		    hwbase + size <= hwvtrans[i].hwbase + hwvtrans[i].size)
			return &hwvtrans[i];
	}
	return NULL;
}

/**
 * omap3_getvbase
 *
 * For a given hardware base address returns the virtual address, ideally 
 * we try to ensure that virtual address match actual hardware address, it's
 * much easier that way.
 */
#if 0
static int
omap3_getvbase(uint32_t hwbase, uint32_t size, uint32_t *vbase)
{
	const struct hwvtrans *hw;
	
	hw = omap3_gethwvtrans(hwbase, size);
	if (hw == NULL)
		return (ENOENT);
	*vbase = hwbase - hw->hwbase + hw->vbase;
	return (0);
}
#endif

/**
 * omap3_release_resource
 *
 * 
 *
 */
static struct resource *
omap3_alloc_resource(device_t dev, device_t child, int type, int *rid,
                     u_long start, u_long end, u_long count, u_int flags)
{
	struct omap3_softc *sc = device_get_softc(dev);
	const struct hwvtrans *vtrans;
	struct resource *rv;
	int needactivate = flags & RF_ACTIVE;
	
	flags &= ~RF_ACTIVE;
	switch (type) {
		case SYS_RES_IRQ:
			printf("[BRG] omap3_alloc_resource : irq : %d %d\n", (int)start, (int)end);
			rv = rman_reserve_resource(&sc->sc_irq_rman, start, end, count,
									   flags, child);
			if (rv != NULL)
				rman_set_rid(rv, *rid);
			break;
			
		case SYS_RES_MEMORY:
			printf("[BRG] omap3_alloc_resource : mem : 0x%08X 0x%08X\n", start, end);
			vtrans = omap3_gethwvtrans(start, end - start);
			if (vtrans == NULL) {
				/* likely means above table needs to be updated */
				device_printf(child, "%s: no mapping for 0x%lx:0x%lx\n",
							  __func__, start, end - start);
				return NULL;
			}
			rv = rman_reserve_resource(&sc->sc_mem_rman, start, end,
									   end - start, flags, child);
			if (rv == NULL) {
				device_printf(child, "%s: cannot reserve 0x%lx:0x%lx\n",
							  __func__, start, end - start);
				return NULL;
			}
			rman_set_rid(rv, *rid);
			break;
			
		default:
			rv = NULL;
			break;
	}
	
	if (rv != NULL && needactivate) {
		if (bus_activate_resource(child, type, *rid, rv)) {
			rman_release_resource(rv);
			return (NULL);
		}
	}
	
	return (rv);
}


/**
 * omap3_release_resource
 *
 * 
 *
 */
static int
omap3_release_resource(device_t bus, device_t child, int type, int rid,
						struct resource *r)
{
	/* NB: no private resources, just release */
	return rman_release_resource(r);
}


/**
 * omap3_activate_resource
 *
 * 
 *
 */
static int
omap3_activate_resource(device_t dev, device_t child, int type, int rid,
						 struct resource *r)
{
	struct omap3_softc *sc = device_get_softc(dev);
	const struct hwvtrans *vtrans;
	uint32_t start = rman_get_start(r);
	uint32_t size = rman_get_size(r);
	
	if (type == SYS_RES_MEMORY) {
		vtrans = omap3_gethwvtrans(start, size);
		if (vtrans == NULL) {		/* NB: should not happen */
			device_printf(child, "%s: no mapping for 0x%lx:0x%lx\n",
						  __func__, start, size);
			return (ENOENT);
		}
		rman_set_bustag(r, sc->sc_iot);
		rman_set_bushandle(r, vtrans->vbase + (start - vtrans->hwbase));
	}
	return (rman_activate_resource(r));
}


/**
 * omap3_deactivate_resource
 *
 * 
 *
 */
static int
omap3_deactivate_resource(device_t bus, device_t child, int type, int rid,
						   struct resource *r) 
{
	/* NB: no private resources, just deactive */
	return (rman_deactivate_resource(r));
}




static device_method_t g_omap3_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			omap3_probe),
	DEVMETHOD(device_attach,		omap3_attach),
	DEVMETHOD(device_identify,		omap3_identify),
	
	/* Bus interface */
	DEVMETHOD(bus_add_child,		omap3_add_child),
	DEVMETHOD(bus_hinted_child,		omap3_hinted_child),
	DEVMETHOD(bus_read_ivar,		omap3_read_ivar),
	
	DEVMETHOD(bus_alloc_resource,		omap3_alloc_resource),
	DEVMETHOD(bus_release_resource,		omap3_release_resource),
	DEVMETHOD(bus_activate_resource,	omap3_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	omap3_deactivate_resource),
	DEVMETHOD(bus_setup_intr,			omap3_setup_intr),
	DEVMETHOD(bus_teardown_intr,		omap3_teardown_intr),
	
	{0, 0},
};

static driver_t g_omap3_driver = {
	"omap3",
	g_omap3_methods,
	sizeof(struct omap3_softc),
};
static devclass_t g_omap3_devclass;

DRIVER_MODULE(omap3, nexus, g_omap3_driver, g_omap3_devclass, 0, 0);






