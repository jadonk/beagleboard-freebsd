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
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/ic/ns16550.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include "uart_if.h"

static int uart_omap3_probe(device_t dev);


/* --------------------------------------------------------------- */
/* uart_omap3_methods
 *
 * Methods implemented for the UART device, note the only unique one
 * is the 'probe', the others are done by the default 8250 UART
 * driver.
 */
static device_method_t uart_omap3_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		uart_omap3_probe),
	DEVMETHOD(device_attach,	uart_bus_attach),
	DEVMETHOD(device_detach,	uart_bus_detach),
	{ 0, 0 }
};


/* --------------------------------------------------------------- */
/* uart_omap3_driver
 *
 * The driver device, which is just a standard 8250 type device.
 *
 */
static driver_t uart_omap3_driver = {
	uart_driver_name,
	uart_omap3_methods,
	sizeof(struct uart_softc),
};
DRIVER_MODULE(uart, omap, uart_omap3_driver, uart_devclass, 0, 0);



/* --------------------------------------------------------------- */
/*
 * uart_omap3_probe
 *
 * Called to check for the presence of the serial port and also
 * performs initialisation of the UART device.
 *
 */
static int uart_omap3_probe(device_t dev)
{
	struct uart_softc *sc;
	int unit = device_get_unit(dev);
	u_int rclk;

	sc = device_get_softc(dev);
	sc->sc_class = &uart_ns8250_class;
	if (resource_int_value("uart", unit, "rclk", &rclk))
		rclk = OMAP35XX_UART_FREQ;
	
	if (bootverbose)
		device_printf(dev, "rclk %u\n", rclk);

	return uart_bus_probe(dev, 0, rclk, 0, 0);
}
