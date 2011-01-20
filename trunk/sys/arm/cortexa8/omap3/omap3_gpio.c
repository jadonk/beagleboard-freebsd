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
 *	Very simple GPIO (general purpose IO) driver module for the TI OMAP3530 SoC.
 *
 *	Currently this driver only does the basics, get a value on a pin & set a
 *	value on a pin. Hopefully over time I'll expand this to be a bit more generic
 *	and support interrupts and other various bits on the SoC can do ... in the
 *	meantime this is all you get.
 *
 *	Also beware, the OMAP3530 datasheet lists GPIO banks 1-6, whereas I've used
 *	0-5 here in the code.
 *
 *	External Usage
 *	~~~~~~~~~~~~~~
 *	- Call omap3_gpio_request() to get 'exclusive' access to the GPIO pin, only
 *	  one person can call omap3_gpio_request() on the same pin. However currently
 *	  this module works on the honor system, there is nothing stopping another
 *	  module from changing the state of the pin using the pin number.
 *	- Use omap3_gpio_direction_output or omap3_gpio_direction_input to change
 *	  the mode of the pin, from an output to an input and vice verse.
 *	- Use omap3_gpio_pin_get() to get the level of an input pin.
 *	- Use omap3_gpio_pin_set() to set the level of an output GPIO pin.
 *	- Call omap3_gpio_free() to release 'exclusive' access to the GPIO pin.
 *
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
#include <sys/timetc.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_gpio.h>
#include <arm/cortexa8/omap3/omap3_prcm.h>


/**
 * The TI OMAP3530 has a maximum of 192 IO pins, seperated accross 6 register
 * sets.
 */
#define OMAP3_GPIO_MAX_PINS		192


/**
 * Macros for converting a pin value (0-195) to a register bank (0-5) and a
 * bitmask (1,2,4,8, etc).
 */
#define OMAP3_GPIO_PIN2MASK(p)	(1UL << ((p) & 0x1F))
#define OMAP3_GPIO_PIN2BANK(p)	((p) >> 5)


/**
 *	GPIO device driver context, a pointer to this is stored globally as the
 *	driver is never intented to be unloaded (see g_omap3_gpio_sc).
 */
struct omap3_gpio_softc {
	device_t			sc_dev;
	/* Annoyingly GPIO1 is in the L4-WAKEUP memory space, whereas all the others
	 * are in the L4-PERIPH space ... so we need seperate mappings for GPIO1 and
	 * GPIO2-6.
	 */
	bus_space_tag_t		sc_iotag;
	bus_space_handle_t	sc_ioh_gpio1;
	bus_space_handle_t	sc_ioh_gpio2_6;

	struct resource*	sc_irq;
	
	uint32_t			sc_gpio_setup[(OMAP3_GPIO_MAX_PINS / 32)];
	
	void (*sc_callbacks[OMAP3_GPIO_MAX_PINS])(unsigned int, unsigned int, void*);
	void*               sc_callback_data[OMAP3_GPIO_MAX_PINS];
	
	uint32_t			sc_debounce[(OMAP3_GPIO_MAX_PINS / 32)];

	struct mtx			sc_mtx;

	
};

static struct omap3_gpio_softc *g_omap3_gpio_sc = NULL;


/**
 *	Function prototypes
 *
 */
static void omap3_gpio_intr(void *);


/**
 *	omap3_gpio_readl - reads a 32-bit value from one of the GPIO registers
 *	@sc: gpio device context
 *	@bank: the bank to read from, values 0 to 5.
 *	@off: the byte offset within the register bank to read from.
 *
 *	
 *
 *	LOCKING:
 *	Read only, no locking required
 *
 *	RETURNS:
 *	32-bit value read from the register.
 */
static uint32_t
omap3_gpio_readl(struct omap3_gpio_softc *sc, uint32_t bank, bus_size_t off)
{
	switch (bank) {
		case 0:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio1, off);
		case 1:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio2_6, off);
		case 2:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
									((OMAP35XX_GPIO3_HWBASE - OMAP35XX_GPIO2_HWBASE)
									+ off));
		case 3:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
									((OMAP35XX_GPIO4_HWBASE - OMAP35XX_GPIO2_HWBASE)
									 + off));
		case 4:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
									((OMAP35XX_GPIO5_HWBASE - OMAP35XX_GPIO2_HWBASE)
									 + off));
		case 5:
			return bus_space_read_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
									((OMAP35XX_GPIO6_HWBASE - OMAP35XX_GPIO2_HWBASE)
									 + off));
		default:
			panic("%s: Invalid gpio module", device_get_name(sc->sc_dev));
			return(0);
	}
			
}

/**
 *	omap3_gpio_writel - writes a 32-bit value to one of the GPIO registers
 *	@sc: gpio device context
 *	@bank: the bank to write to, values 0 to 5.
 *	@off: the byte offset within the register bank to write to.
 *	
 *
 *	LOCKING:
 *	Read only, no locking required
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_gpio_writel(struct omap3_gpio_softc *sc, uint32_t bank, bus_size_t off,
				  uint32_t val)
{
	switch (bank) {
		case 0:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio1, off, val);
			break;
		case 1:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio2_6, off, val);
			break;
		case 2:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
							  ((OMAP35XX_GPIO3_HWBASE - OMAP35XX_GPIO2_HWBASE)
							   + off), val);
			break;
		case 3:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
							  ((OMAP35XX_GPIO4_HWBASE - OMAP35XX_GPIO2_HWBASE)
							   + off), val);
			break;
		case 4:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
							  ((OMAP35XX_GPIO5_HWBASE - OMAP35XX_GPIO2_HWBASE)
							   + off), val);
			break;
		case 5:
			bus_space_write_4(sc->sc_iotag, sc->sc_ioh_gpio2_6,
							  ((OMAP35XX_GPIO6_HWBASE - OMAP35XX_GPIO2_HWBASE)
							   + off), val);
			break;
		default:
			panic("%s: Invalid gpio module", device_get_name(sc->sc_dev));
	}
}




/**
 *	omap3_gpio_probe - driver probe function
 *	@dev: gpio device handle
 *
 *	Simply sets the name of the driver
 *
 *
 *	RETURNS:
 *	Returns 0 on sucess, a negative error code on failure.
 */
static int
omap3_gpio_probe(device_t dev)
{
	
	device_set_desc(dev, "TI OMAP3 GPIO Controller");
	return (0);
}

/**
 *	omap3_gpio_attach - driver attach function
 *	@dev: gpio device handle
 *
 *	Sets up the driver data structure and initialises all the fields.
 *
 *	RETURNS:
 *	Returns 0 on sucess, a negative error code on failure.
 */
static int
omap3_gpio_attach(device_t dev)
{
	struct omap3_gpio_softc *sc = device_get_softc(dev);
	unsigned int i = 0;
	int          rid = 0;
	void        *ihl;
	int          err;

	/* Setup the basics */
	sc->sc_dev = dev;
	sc->sc_iotag = &omap3_bs_tag;
	
	/* No gpios requested at startup */
	for (i=0; i<(OMAP3_GPIO_MAX_PINS/32); i++)
		sc->sc_gpio_setup[i] = 0x00000000;
	
	
	/* Mutex to protect the shared data structures */
	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "omap3_gpio", MTX_SPIN);
	
	/* Map in the GPIO1 register set */
	if (bus_space_map(sc->sc_iotag, OMAP35XX_GPIO1_HWBASE, OMAP35XX_GPIO1_SIZE,
					  0, &sc->sc_ioh_gpio1)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}

	/* Map in the GPIO2 - GPIO6 register set */
	if (bus_space_map(sc->sc_iotag, OMAP35XX_GPIO2_HWBASE,
					  ((OMAP35XX_GPIO6_HWBASE + OMAP35XX_GPIO6_SIZE) -
					   OMAP35XX_GPIO2_HWBASE),
					  0, &sc->sc_ioh_gpio2_6)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}
	

	/* Install interrupt handlers for all the possible interrupts */
	sc->sc_irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, OMAP35XX_IRQ_GPIO1_MPU,
									OMAP35XX_IRQ_GPIO6_MPU, 6, RF_ACTIVE);
	if (sc->sc_irq == NULL)
		panic("Unable to setup the GPIO irq handler.\n");
	
	err = bus_setup_intr(dev, sc->sc_irq, INTR_TYPE_MISC | INTR_MPSAFE, NULL,
				         omap3_gpio_intr, NULL, &ihl);
	if (err) {
		panic("%s: Cannot register IRQ", device_get_name(dev));
	}



	
	/* TODO: Reset all the GPIO modules ... is this needed ? is it a good idea ? */

	
	/* Store the GPIO structure globally, this driver should never be unloaded */
	g_omap3_gpio_sc = sc;
	
	return (0);
}


static device_method_t g_omap3_gpio_methods[] = {
	DEVMETHOD(device_probe,		omap3_gpio_probe),
	DEVMETHOD(device_attach,	omap3_gpio_attach),
	{ 0, 0 }
};

static driver_t g_omap3_gpio_driver = {
	"omap3_gpio",
	g_omap3_gpio_methods,
	sizeof(g_omap3_gpio_methods),
};

static devclass_t g_omap3_gpio_devclass;

DRIVER_MODULE(omap3_gpio, omap3, g_omap3_gpio_driver, g_omap3_gpio_devclass, 0, 0);
MODULE_DEPEND(omap3_gpio, omap3_prcm, 1, 1, 1);



/**
 *	omap3_gpio_request - requests 'exclusive' access to the GPIO pin.
 *	@pin: the pin number (0-195)
 *	@name: the name to give the pin (currently not used).
 *
 *	
 *
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success.
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin requested is outside valid range or already in use.
 */
int
omap3_gpio_request(unsigned int pin, const char *name)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS) {
		device_printf(sc->sc_dev, "Error - pin number is too large (%u)\n", pin);
		return(-EINVAL);
	}
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & mask) {
		ret = -EINVAL;
	} else {
		/* enable the interface clock for the bank, just because there is no
		 * guarantee it is already enabled.
		 */
		switch (bank) {
			case 0:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO1_ICLK);
				break;
			case 1:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO2_ICLK);
				break;
			case 2:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO3_ICLK);
				break;
			case 3:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO4_ICLK);
				break;
			case 4:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO5_ICLK);
				break;
			case 5:
				omap3_prcm_enable_clk(OMAP3_MODULE_GPIO6_ICLK);
				break;
		}
	
		/* set the flag to say it is in use */
		sc->sc_gpio_setup[bank] |= mask;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}

/**
 *	omap3_gpio_free - releases 'exclusive' access to the GPIO pin.
 *	@pin: the pin number (0-195)
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success.
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin released is outside valid range or not requested in use.
 */
int
omap3_gpio_free(unsigned int pin)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS) {
		device_printf(sc->sc_dev, "Error - pin number is too large (%u)\n", pin);
		return(-EINVAL);
	}
	
	mtx_lock(&sc->sc_mtx);
	
	if (!(sc->sc_gpio_setup[bank] & mask)) {
		ret = -EINVAL;
	} else {
		sc->sc_gpio_setup[bank] &= ~mask;
	
		/* check if none of the pins are no in use and if so disable the
		 * interface clock.
		 */
		if (sc->sc_gpio_setup[bank] == 0x00000000) {
			switch (bank) {
				case 0:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO1_ICLK);
					break;
				case 1:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO2_ICLK);
					break;
				case 2:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO3_ICLK);
					break;
				case 3:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO4_ICLK);
					break;
				case 4:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO5_ICLK);
					break;
				case 5:
					omap3_prcm_disable_clk(OMAP3_MODULE_GPIO6_ICLK);
					break;
			}
		}
	}

	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}


/**
 *	omap3_gpio_direction_output - sets a GPIO pin to be an output.
 *	@pin: the pin number (0-195)
 *	@val: the value to drive on the output; val==0 low, val!=0 high
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success.
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_direction_output(unsigned int pin, int val)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t dir;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS) {
		device_printf(sc->sc_dev, "Error - pin number is too large (%u)\n", pin);
		return(-EINVAL);
	}
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & mask) {
		
		/* set the data value first */
		if (val)
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_SETDATAOUT, mask);
		else
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_CLEARDATAOUT, mask);

		
		/* set the direction of the pin as an output */
		dir = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_OE);
		dir &= ~mask;
		omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_OE, dir);
	}
	else {
		ret = -EINVAL;
	}

	mtx_unlock(&sc->sc_mtx);

	return(ret);
}	

/**
 *	omap3_gpio_direction_input - sets a GPIO pin to be an input.
 *	@pin: the pin number (0-195)
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success.
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_direction_input(unsigned int pin)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t dir;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS)
		return(-EINVAL);
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & (1UL << (pin & 0x1F))) {
		
		/* set the direction of the pin as an input */
		dir = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_OE);
		dir |= mask;
		omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_OE, dir);
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}

/**
 *	omap3_gpio_pin_get - reads the state on a GPIO pin.
 *	@pin: the pin number (0-195)
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on pin state low
 *	1 on pin state high
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_pin_get(unsigned int pin)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS)
		return(-EINVAL);
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & mask) {
		
		/* get the data value */
		if (omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_DATAIN) & mask)
			ret = 1;
		else
			ret = 0;
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}

/**
 *	omap3_gpio_pin_set - sets the state on a GPIO pin.
 *	@pin: the pin number (0-195)
 *	@val: the value to drive on the output; val==0 low, val!=0 high
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_pin_set(unsigned int pin, int val)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if ((pin < 0) || (pin >= OMAP3_GPIO_MAX_PINS))
		return(-EINVAL);
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & mask) {
		
		/* set the data value */
		if (val)
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_SETDATAOUT, mask);
		else
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_CLEARDATAOUT, mask);
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}



/**
 *	omap3_gpio_pin_toggle - toggles the level on an output GPIO pin.
 *	@pin: the pin number (0-195)
 *
 *	
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_pin_toggle(unsigned int pin)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t val;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS)
		return(-EINVAL);
	
	mtx_lock(&sc->sc_mtx);
	
	if (sc->sc_gpio_setup[bank] & mask) {
		
		/* get the data out value */
		val = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_DATAOUT);
		
		/* set the data value */
		if (val & mask)
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_CLEARDATAOUT, mask);
		else
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_SETDATAOUT, mask);
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}



/**
 *	omap3_gpio_intr - interrupt handler for all 6 GPIO IRQs
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
omap3_gpio_intr(void *arg)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	uint32_t bank;
	uint32_t pin;
	uint32_t i;
	uint32_t intr;
	uint32_t din;
	
	void (*callback)(unsigned int, unsigned int, void*);
	void *callback_data;

	mtx_lock(&sc->sc_mtx);

	for (bank = 0; bank < (OMAP3_GPIO_MAX_PINS / 32); bank++) {
	
		/* read the bank interrupt status */
		intr = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_IRQSTATUS1);
		intr &= omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_IRQENABLE1);
		
		/* read the current level */
		din = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_DATAIN);
		
		/* call any callbacks if the interrupt matches */
		if (intr != 0) {
			for (i = 0; i< 32; i++) {
				pin = (i + (bank * 32));
				if ((intr & (0x1 << i)) && (sc->sc_callbacks[pin] != NULL)) {
				
					callback = sc->sc_callbacks[pin];
					callback_data = sc->sc_callback_data[pin];

					mtx_unlock(&sc->sc_mtx);
					
					callback(pin, ((din >> i) & 0x1), callback_data);
										  
					mtx_lock(&sc->sc_mtx);
				}
			}
		}
	
		/* clear the interrupt status */
		omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_IRQSTATUS1, intr);
	}

	mtx_unlock(&sc->sc_mtx);
}


/**
 *	omap3_gpio_pin_intr - enables interrupt generation on a pin change
 *	@pin: the pin number (0-195)
 *	@trigger: the pin trigger to use to generate interrupts, can OR multiple
 *	          multiple triggers. Possible values are; GPIO_TRIGGER_LOWLEVEL,
 *	          GPIO_TRIGGER_HIGHLEVEL, GPIO_TRIGGER_RISING and
 *	          GPIO_TRIGGER_FAILING.
 *	@callback: callback function
 *	@data: user defined data that is passed in the callbaack
 *
 *	This function automatically turns the pin into a input and enables interrupt
 *	generation on the change. When an interrupt does occur the callback is
 *	called.
 *
 *	If a callback is called, it is done with the GPIO lock held, therefore you
 *	cannot call any of the omap3_gpio_* functions from the callback.
 *
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_pin_intr(unsigned int pin, unsigned int trigger,
                    void (*callback)(unsigned int pin, unsigned int datain, void *data),
					void *data)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	uint32_t reg;
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS)
		return(-EINVAL);

	trigger &= 0xf;
	if (trigger == 0)
		return(-EINVAL);

	mtx_lock(&sc->sc_mtx);

	if (sc->sc_gpio_setup[bank] & mask) {

		/* set the callback details */
		sc->sc_callbacks[pin] = callback;
		sc->sc_callback_data[pin] = data;
		
		/* disable interrupts on this particular pin while changing the trig */
		omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_CLEARIRQENABLE1, mask);
		
		/* enable the interrupts */
		if (trigger & GPIO_TRIGGER_LOWLEVEL) {
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_LEVELDETECT0);
			reg |= mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_LEVELDETECT0, reg);
		}
		if (trigger & GPIO_TRIGGER_HIGHLEVEL) {
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_LEVELDETECT1);
			reg |= mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_LEVELDETECT1, reg);
		}
		if (trigger & GPIO_TRIGGER_RISING) {
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_RISINGDETECT);
			reg |= mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_RISINGDETECT, reg);
		}
		if (trigger & GPIO_TRIGGER_FALLING) {
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_FALLINGDETECT);
			reg |= mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_FALLINGDETECT, reg);
		}
		
		/* enable the interrupt again */
		omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_SETIRQENABLE1, mask);
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}



/**
 *	omap3_gpio_pin_debounce - enables/disables debouncing on a pin
 *	@pin: the pin number (0-195)
 *	@enable: non-zero value to enable, 0 to disable
 *	@interval: the debounce interval in microseconds, used for all pins on the bank
 *
 *	
 *
 *	LOCKING:
 *	Internally locks it's own context.
 *
 *	RETURNS:
 *	0 on success
 *	-ENOMEM on driver not initialised.
 *	-EINVAL if pin is outside valid range or not reserved for use.
 */
int
omap3_gpio_pin_debounce(unsigned int pin, int enable, unsigned int interval)
{
	struct omap3_gpio_softc *sc = g_omap3_gpio_sc;
	int ret = 0;
	uint32_t bank = OMAP3_GPIO_PIN2BANK(pin);
	uint32_t mask = OMAP3_GPIO_PIN2MASK(pin);
	uint32_t reg;
	
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= OMAP3_GPIO_MAX_PINS)
		return(-EINVAL);

	mtx_lock(&sc->sc_mtx);

	if (sc->sc_gpio_setup[bank] & mask) {
	
		if (enable) {
		
			/* set the debounce flag */
			sc->sc_debounce[bank] |= mask;
			
			/* if enabling we need to check that the functional clock is enabled */
			switch (bank) {
				case 0:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO1_FCLK);
					break;
				case 1:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO2_FCLK);
					break;
				case 2:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO3_FCLK);
					break;
				case 3:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO4_FCLK);
					break;
				case 4:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO5_FCLK);
					break;
				case 5:
					omap3_prcm_enable_clk(OMAP3_MODULE_GPIO6_FCLK);
					break;
			}
	
			/* enable debounce in the register */
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_DEBOUNCENABLE);
			reg |= mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_DEBOUNCENABLE, reg);
			
			/* finally set the debounce time, this affects all pins on the bank,
			 * but hopefully the caller knows that.
			 */
			interval /= 31;
			if (interval != 0)
				interval -= 1;
			if (interval > 0xff)
				interval = 0xff;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_DEBOUNCINGTIME, interval);
			
		} else {
		
			/* clear the debounce flag */
			sc->sc_debounce[bank] &= ~mask;
			
			/* if debouncing is no longer needed on the bank disable the clock */
			if (sc->sc_debounce[bank] == 0x00000000) {
				switch (bank) {
					case 0:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO1_FCLK);
						break;
					case 1:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO2_FCLK);
						break;
					case 2:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO3_FCLK);
						break;
					case 3:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO4_FCLK);
						break;
					case 4:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO5_FCLK);
						break;
					case 5:
						omap3_prcm_disable_clk(OMAP3_MODULE_GPIO6_FCLK);
						break;
				}
			}
			
			/* disable debounce in the register */
			reg = omap3_gpio_readl(sc, bank, OMAP35XX_GPIO_DEBOUNCENABLE);
			reg &= ~mask;
			omap3_gpio_writel(sc, bank, OMAP35XX_GPIO_DEBOUNCENABLE, reg);
		}
		
	}
	else {
		ret = -EINVAL;
	}
	
	mtx_unlock(&sc->sc_mtx);
	
	return(ret);
}


