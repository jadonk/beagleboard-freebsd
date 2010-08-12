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

/*
 * Texas Instruments OMAP Power Management and System Companion Device
 *
 * 
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/clock.h>
#include <sys/time.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>

#include <dev/iicbus/iiconf.h>

#include "iicbus_if.h"

//#define	IIC_M_WR	0	/* write operation */

#define	TPS65950_ADDR	0x4A	/* slave address */

#define	MAX_IIC_DATA_SIZE	4

struct tps65950_softc {
	device_t                sc_dev;
	
	/* the following is the init function hook so it is run after interupts
	 * and the clocks have been enabled.
	 */
	struct intr_config_hook sc_ich;

};

/**
 *	tps65950_probe - probe function for the driver
 *	@dev: i2c device handle
 *
 *	
 *
 *	LOCKING:
 *	Called from timer context
 *
 *	RETURNS:
 *	BUS_PROBE_NOWILDCARD
 */
static int
tps65950_probe(device_t dev)
{
	device_set_desc(dev, "TI OMAP Power Management and System Companion Device");
	return (BUS_PROBE_NOWILDCARD);
}

/**
 *	tps65950_read - reads one or more bytes from the remote IC
 *	@dev: i2c device handle
 *	@addr: the address to read from
 *	@data: buffer to put the received bytes in (must be at least as big as size)
 *	@size: the number of bytes to read
 *	
 *
 *	LOCKING:
 *	Called from timer context
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
tps65950_read(device_t dev, uint8_t addr, uint8_t *data, uint8_t size)
{
	struct iic_msg msgs[2] = {
	     { TPS65950_ADDR, IIC_M_WR, 1, &addr },
	     { TPS65950_ADDR, IIC_M_RD, size, data }
	};

	return (iicbus_transfer(dev, msgs, 2));
}

/**
 *	tps65950_write - attach function for the driver
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
tps65950_write(device_t dev, uint8_t addr, uint8_t *data, uint8_t size)
{
	uint8_t buffer[MAX_IIC_DATA_SIZE + 1];
	struct iic_msg msgs[1] = {
	     { TPS65950_ADDR, IIC_M_WR, size + 1, buffer },
	};
	
	if (size > MAX_IIC_DATA_SIZE)
		return (ENOMEM);

	buffer[0] = addr;
	memcpy(buffer + 1, data, size);

	return (iicbus_transfer(dev, msgs, 1));
}

/**
 *	tps65950_init - attach function for the driver
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
tps65950_init(void *arg)
{
	device_t dev = (device_t)arg;
	struct tps65950_softc *sc = device_get_softc(dev);
	uint8_t rsp = 0x00;
	int error = 0;
	
	error = tps65950_read(dev, 0xEE, &rsp, 1);
	printf("[BRG] Read 0x%02x from the tps65950 (error = %d)\n", rsp, error);

	rsp = 0x00;
	error = tps65950_write(dev, 0xEE, &rsp, 1);
	printf("[BRG] Read 0x%02x from the tps65950 (error = %d)\n", rsp, error);

	error = tps65950_read(dev, 0xEE, &rsp, 1);
	printf("[BRG] Read 0x%02x from the tps65950 (error = %d)\n", rsp, error);

	/* this seems a bit weird, but we appear to have to remove this intrhook
	 * function manually.
	 */
	config_intrhook_disestablish(&sc->sc_ich);

	return;
}

/**
 *	tps65950_attach - attach function for the driver
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
tps65950_attach(device_t dev)
{
	struct tps65950_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	/* install a function to initialised */
	sc->sc_ich.ich_func = tps65950_init;
	sc->sc_ich.ich_arg = dev;
	config_intrhook_establish(&sc->sc_ich);
	
	return (0);
}

static device_method_t tps65950_methods[] = {
	DEVMETHOD(device_probe,		tps65950_probe),
	DEVMETHOD(device_attach,	tps65950_attach),

	{0, 0},
};

static driver_t tps65950_driver = {
	"tps65950",
	tps65950_methods,
	sizeof(struct tps65950_softc),
};
static devclass_t tps65950_devclass;

DRIVER_MODULE(tps65950, iicbus, tps65950_driver, tps65950_devclass, 0, 0);
MODULE_VERSION(tps65950, 1);
MODULE_DEPEND(tps65950, iicbus, 1, 1, 1);

