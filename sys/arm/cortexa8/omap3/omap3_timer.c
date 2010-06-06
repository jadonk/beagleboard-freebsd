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
 * To be confusing, there are two timers setup here, one is the system ticks
 * that is suppose to go off 'hz' number of times a second. The other is a
 * general purpose counter that is just used to provide a count to the system.
 *
 * For the system tick timer we use GPTIMER10, this has an acurate 1ms mode
 * which is designed for system tick generation. Provided hz never gets over
 * a 1000 this is perfect.
 *
 * For the other timer we use GPTIMER11, for no special reason other than it
 * comes after 10 and they are both in the core power domain. It's set to a
 * clock frequency of 12Mhz, again for no special reason except it is a nice
 * round number.
 *
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/timetc.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/frame.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_prcm.h>
#include <arm/cortexa8/omap3/omap3_gpio.h>


/**
 * The pin to use for the heartbeat, the pin will be toggled every second. If
 * you don't want a heartbeat, don't define the following.
 * TODO: Move this to make options
 */
#define OMAP3_HEARTBEAT_GPIO	150


/**
 * The interrupt/status bits used in the timer registers.
 *
 */
#define TCAR_IT_FLAG	0x04
#define OVF_IT_FLAG		0x02
#define MAT_IT_FLAG		0x01

#define TCAR_IT_ENA		0x04
#define OVF_IT_ENA		0x02
#define MAT_IT_ENA		0x01

#define TCAR_WUP_ENA	0x04
#define OVF_WUP_ENA		0x02
#define MAT_WUP_ENA		0x01


/**
 * Timer module data structure, persists forever as this driver module is
 * never intented to be dynamically unloaded, see g_omap3_clk_sc.
 *
 */
struct omap3_clk_softc {
	device_t			sc_dev;
	bus_space_tag_t		sc_iot;

	bus_addr_t			sc_gptimer10_baseaddr;
	bus_size_t			sc_gptimer10_size;
	bus_space_handle_t	sc_gptimer10_ioh;
	
	bus_addr_t			sc_gptimer11_baseaddr;
	bus_size_t			sc_gptimer11_size;
	bus_space_handle_t	sc_gptimer11_ioh;
};

static struct omap3_clk_softc *g_omap3_clk_sc = NULL;


/* The input oscillator freq, calculated from the boot-strap registers */
uint32_t g_omap3_oscclk_freq = 0;

/* The system clock (SYS_CLK) freq, calculated from the boot-strap registers */
uint32_t g_omap3_sysclk_freq = 0;



/**
 * Function prototypes.
 */
int	omap3_clk_intr(void *);
static unsigned omap3_timer_get_timecount(struct timecounter *tc);




static struct timecounter g_omap3_clk_timecounter = {
	.tc_get_timecount  = omap3_timer_get_timecount,	/* get_timecount */
	.tc_poll_pps       = NULL,						/* no poll_pps */
	.tc_counter_mask   = ~0u,						/* counter_mask */
	.tc_frequency      = 0,							/* frequency */
	.tc_name           = "OMAP3 Timer",				/* name */
	.tc_quality        = 1000,						/* quality */
};


/**
 *	device_set_desc - simply sets the name of the module/device
 *	@dev: timer device handle
 *
 *	
 *
 *	RETURNS:
 *	0 on success, a negative error code on failure
 */
static int
omap3_clk_probe(device_t dev)
{
	device_set_desc(dev, "TI OMAP3 Timer");
	return (0);
}

/**
 *	omap3_clk_attach - module entry point
 *	@dev: timer device handle
 *
 *	
 *
 *	RETURNS:
 *	0 on success, a negative error code on failure
 */
static int
omap3_clk_attach(device_t dev)
{
	struct omap3_clk_softc *sc = device_get_softc(dev);
		
	sc->sc_dev = dev;
	sc->sc_iot = &omap3_bs_tag;
	
	
	/* Map in the general purpose timer registers for timer 10 */
	sc->sc_gptimer10_baseaddr = OMAP35XX_GPTIMER10_HWBASE;
	sc->sc_gptimer10_size = OMAP35XX_GPTIMER_SIZE;
	
	if (bus_space_map(sc->sc_iot, sc->sc_gptimer10_baseaddr, sc->sc_gptimer10_size, 0, &sc->sc_gptimer10_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}
	

	/* Map in the general purpose timer registers for timer 11 */
	sc->sc_gptimer11_baseaddr = OMAP35XX_GPTIMER11_HWBASE;
	sc->sc_gptimer11_size = OMAP35XX_GPTIMER_SIZE;
	
	if (bus_space_map(sc->sc_iot, sc->sc_gptimer11_baseaddr, sc->sc_gptimer11_size, 0, &sc->sc_gptimer11_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}
	
	

	/* Reserve a GPIO line for the heartbeat, also set it as an output and high */
#if defined(OMAP3_HEARTBEAT_GPIO)
	if (omap3_gpio_request(OMAP3_HEARTBEAT_GPIO, "heartbeat") == 0) {
		omap3_gpio_direction_output(OMAP3_HEARTBEAT_GPIO, 1);
	}
#endif
	
	/* Save globally, the timer module never gets unloaded */
	g_omap3_clk_sc = sc;

	return (0);
}

static device_method_t g_omap3_clk_methods[] = {
	DEVMETHOD(device_probe, omap3_clk_probe),
	DEVMETHOD(device_attach, omap3_clk_attach),
	{0, 0},
};

static driver_t g_omap3_clk_driver = {
	"omap3_clk",
	g_omap3_clk_methods,
	sizeof(struct omap3_clk_softc),
};
static devclass_t g_omap3_clk_devclass;

DRIVER_MODULE(omap3_clk, omap3, g_omap3_clk_driver, g_omap3_clk_devclass, 0, 0);







/**
 *	omap3_timer_get_timecount - returns the count in GPTIMER11, the system counter
 *	@tc: pointer to the timecounter structure used to register the callback
 *
 *	
 *
 *	RETURNS:
 *	the value in the counter
 */
static unsigned
omap3_timer_get_timecount(struct timecounter *tc)
{
	struct omap3_clk_softc* sc = g_omap3_clk_sc;
	uint32_t ret;
	
	ret = bus_space_read_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TCRR);
	
	return (ret);
}



/**
 *	omap3_get_system_clks - reads the clocking values from the boot-strap regs
 *	@sc: pointer to the clk module/device context
 *
 *	Read the clocking information from the power-control/boot-strap registers,
 *  and stored in two global variables.
 *
 *	RETURNS:
 *	nothing, values are saved in global variables
 */
static void
omap3_get_system_clks(struct omap3_clk_softc *sc)
{
	uint32_t val;
	bus_space_handle_t prm_ioh;
	
	
	/* Create a temporary mapping for the PRCM registers */
	if (bus_space_map(sc->sc_iot, OMAP35XX_PRM_HWBASE, OMAP35XX_PRM_SIZE, 0, &prm_ioh)) {
		panic("%s: Cannot map registers", __func__);
	}

	
	/* Read the input clock freq from the configuration register */
	val = bus_space_read_4(sc->sc_iot, prm_ioh, OMAP35XX_PRM_CLKSEL);
	switch (val & 0x7) {
		case 0x0:
			/* 12Mhz */
			g_omap3_oscclk_freq = 12000000;
			break;
		case 0x1:
			/* 13Mhz */
			g_omap3_oscclk_freq = 13000000;
			break;
		case 0x2:
			/* 19.2Mhz */
			g_omap3_oscclk_freq = 19200000;
			break;
		case 0x3:
			/* 26Mhz */
			g_omap3_oscclk_freq = 26000000;
			break;
		case 0x4:
			/* 38.4Mhz */
			g_omap3_oscclk_freq = 38400000;
			break;
		case 0x5:
			/* 16.8Mhz */
			g_omap3_oscclk_freq = 16800000;
			break;
		default:
			panic("%s: Invalid clock freq", __func__);
	}

	
	/* Read the value of the clock divider used for the system clock */
	val = bus_space_read_4(sc->sc_iot, prm_ioh, OMAP35XX_PRM_CLKSRC_CTRL);
	switch (val & 0xC0) {
		case 0x40:
			g_omap3_sysclk_freq = g_omap3_oscclk_freq;
			break;
		case 0x80:
			g_omap3_sysclk_freq = g_omap3_oscclk_freq / 2;
			break;
		default:
			panic("%s: Invalid system clock divider", __func__);
	}

	/* TODO: Remove debugging */
	printf("OMAP3530 Clocking frequencies, OSCCLK=%uKHz, SYSCLK=%uKHz\n",
		   g_omap3_oscclk_freq/1000, g_omap3_sysclk_freq/1000);
	
	
	/* No longer need the PRM registers */
	bus_space_unmap(sc->sc_iot, prm_ioh, OMAP35XX_PRM_SIZE);
}


/**
 *	omap3_setup_timecount_timer - sets up the timecount clock (GPTIMER11)
 *	@sc: pointer to the clk module/device context
 *
 *	The timer is setup to run with no prescaler off the SYSCLK, which is fixed
 *	by bootstrap values. It can range from 12MHz through to 38.4MHz, the
 *	omap3_get_system_clks() function reads the freq.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_setup_timecount_timer(struct omap3_clk_softc *sc)
{
	/* Set up system clock information */
	omap3_prcm_enable_clk(OMAP3_MODULE_GPTIMER11_FCLK);	
	omap3_prcm_enable_clk(OMAP3_MODULE_GPTIMER11_ICLK);
	omap3_prcm_set_gptimer_clksrc(11, PRCM_USE_SYSCLK);
	
	
	/* Set up the new clock parameters. */
	
	/* reset the timer and poll on the reset complete flag */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TIOCP_CFG,
					  0x002);
	/* TODO: add a timeout */
	while ((bus_space_read_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TISTAT)
		    & 0x01) == 0x00)
		continue;
	
	
	/* disable all interrupts */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TIER,
					  0x00);
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TWER,
					  0x00);
	
	/* set the reload to start back at zero */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TLDR,
					  0x00000000);
	
	/* set the initial counter value to the reload value */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TCRR,
	                  0x00000000);
	
	
	/* enable autoreload mode and start the timer */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TCLR,
	                  0x00000003);
}



/**
 *	omap3_setup_timecount_timer - sets up the tick timer (GPTIMER10)
 *	@sc: pointer to the clk module/device context
 *
 *	The timer is setup to run hz times a second, where hz is a kernel wide
 *	global variable.
 *
 *	RETURNS:
 *	nothing
 */
static void
omap3_setup_tick_timer(struct omap3_clk_softc *sc)
{
	struct resource *irq;
	device_t dev = sc->sc_dev;
	u_int    oldirqstate;
	int      rid = 0;
	void    *ihl;
	
	
	oldirqstate = disable_interrupts(I32_bit);

	/* Register an interrupt handler for general purpose timer 10 */
	irq = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, OMAP35XX_IRQ_GPT10,
							 OMAP35XX_IRQ_GPT10, 1, RF_ACTIVE);
	if (!irq)
		panic("Unable to setup the clock irq handler.\n");
	else
		bus_setup_intr(dev, irq, INTR_TYPE_CLK, omap3_clk_intr, NULL,
					   NULL, &ihl);
	

	/* Set up system clock information */
	omap3_prcm_enable_clk(OMAP3_MODULE_GPTIMER10_FCLK);	
	omap3_prcm_enable_clk(OMAP3_MODULE_GPTIMER10_ICLK);
	omap3_prcm_set_gptimer_clksrc(10, PRCM_USE_32KCLK);

	
	
	/* Set up the new clock parameters. */
	
	/* reset the timer and poll on the reset complete flag */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TIOCP_CFG,
					  0x002);
	/* TODO: add a timeout */
	while ((bus_space_read_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TISTAT)
		    & 0x01) == 0x00)
		continue;
	
	
	/* clear interrupt */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TISR,
					  TCAR_IT_FLAG | OVF_IT_FLAG | MAT_IT_FLAG);
	
	/* enable overflow interrupt */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TIER,
					  OVF_IT_ENA);
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TWER,
					  OVF_WUP_ENA);
	
	/* setup the timer for 1ms ticks */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TPIR,
					  232000);
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TNIR,
					  -768000);
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TLDR,
					  0xFFFFFFE0);
	
	/* setup the number of overflows before irq */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TOWR,
	                  (1000 / hz));
	
	/* set the initial counter value to the reload value */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TCRR,
	                  0xFFFFFFE0);
	
	
	/* enable autoload mode and start the timer */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TCLR,
	                  0x00000003);
	
	restore_interrupts(oldirqstate);
	
	rid = 0;
}



/**
 *	cpu_initclocks - function called by the system in init the tick clock/timer
 *
 *	This is where both the timercount and system ticks timer are started.
 *
 *	RETURNS:
 *	nothing
 */
void
cpu_initclocks(void)
{
	struct omap3_clk_softc* sc = g_omap3_clk_sc;
	
	/* number of microseconds between interrupts */
	tick = 1000000 / hz;

	omap3_get_system_clks(sc);
	
	omap3_setup_tick_timer(sc);
	
	omap3_setup_timecount_timer(sc);

	g_omap3_clk_timecounter.tc_frequency = g_omap3_sysclk_freq;
	tc_init(&g_omap3_clk_timecounter);
}


/**
 *	DELAY - Delay for at least N microseconds.
 *	@n: number of microseconds to delay by
 *
 *	This function is called all over the kernel and is suppose to provide a
 *	consistent delay.
 *
 *	RETURNS:
 *	nothing
 */
void
DELAY(int n)
{
	struct omap3_clk_softc* sc = g_omap3_clk_sc;
	int32_t counts_per_usec;
	int32_t counts;
	uint32_t first, last;
	
	
	if (n == 0)
		return;
	
	/* Check the timers are setup, if not just use a for loop for the meantime */
	if (g_omap3_clk_timecounter.tc_frequency == 0) {
		
		/* TODO: Added a more acurate loop */
		while (n--)
			continue;
		
		return;
	}
	
	/* Get the number of times to count */
	counts_per_usec = ((g_omap3_clk_timecounter.tc_frequency / 1000000) + 1);
	
	/*
	 * Clamp the timeout at a maximum value (about 32 seconds with
	 * a 66MHz clock). *Nobody* should be delay()ing for anywhere
	 * near that length of time and if they are, they should be hung
	 * out to dry.
	 */
	if (n >= (0x80000000U / counts_per_usec))
		counts = (0x80000000U / counts_per_usec) - 1;
	else
		counts = n * counts_per_usec;
	
	
	first = bus_space_read_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TCRR);
	
	while (counts > 0) {
		last = bus_space_read_4(sc->sc_iot, sc->sc_gptimer11_ioh, OMAP35XX_GPTIMER_TCRR);
		counts -= (int32_t)(last - first);
		first = last;
	}
}

/**
 *	omap3_clk_intr - interrupt handler for the tick timer (GPTIMER10)
 *	@arg: the trapframe, needed for the hardclock system function.
 *
 *	This interrupt is triggered every hz times a second.  It's role is basically
 *	to just clear the interrupt status and set it up for triggering again, plus
 *	tell the system a tick timer has gone off by calling the hardclock()
 *	function from the kernel API.
 *
 *	RETURNS:
 *	Always returns FILTER_HANDLED. 
 */
int
omap3_clk_intr(void *arg)
{
	struct omap3_clk_softc* sc = g_omap3_clk_sc;
	struct trapframe *frame = arg;
	uint32_t val;
#if defined(OMAP3_HEARTBEAT_GPIO)
	static int heartbeat_cnt = 0;
#endif	
	
	/* read the interrupt status flag */
	val = bus_space_read_4(sc->sc_iot, sc->sc_gptimer10_ioh,
						   OMAP35XX_GPTIMER_TISR);

	/* clear the interrupt flag */
	bus_space_write_4(sc->sc_iot, sc->sc_gptimer10_ioh, OMAP35XX_GPTIMER_TISR,
					  val);

	/* check if an actual overflow interrupt */
	if (!(val & OVF_IT_FLAG))
		return (FILTER_HANDLED);

	/* heartbeat */
#if defined(OMAP3_HEARTBEAT_GPIO)
	if (heartbeat_cnt++ >= (hz/2)) {
		//		printf("[BRG] ** tick **\n");
		omap3_gpio_pin_toggle(OMAP3_HEARTBEAT_GPIO);
		heartbeat_cnt = 0;
	}
#endif
	
	/* tell the system a timer tick has gone off */
	hardclock(TRAPF_USERMODE(frame), TRAPF_PC(frame));
	
	/* indicate we've handed the interrupt */
	return (FILTER_HANDLED);
}


/**
 *	cpu_startprofclock - Starts the profile clock
 *
 *
 *	RETURNS:
 *	nothing
 */
void
cpu_startprofclock(void)
{
	/* TODO: implement */
}


/**
 *	cpu_startprofclock - Stop the profile clock
 *
 *
 *	RETURNS:
 *	nothing
 */
void
cpu_stopprofclock(void)
{
	/* TODO: implement */
}
