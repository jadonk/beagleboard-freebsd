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
 *	SCM - System Control Module
 *
 *	Hopefully in the end this module will contain a bunch of utility functions
 *	for configuring and querying the general system control registers, but for
 *	now it only does pin(pad) multiplexing.
 *
 *	This is different from the GPIO module in that it is used to configure the
 *	pins between modules not just GPIO input output.
 *
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_scm.h>



/**
 *	The following array maps GPIO numbers to their corresponding PADCONF 
 *	registers.
 */
static uint16_t omap3_scm_gpio_map[192] = {
	[0] = CONTROL_PADCONF_SYS_NIRQ,
	[1] = CONTROL_PADCONF_SYS_CLKREQ,
	[2] = CONTROL_PADCONF_SYS_BOOT0,
	[3] = CONTROL_PADCONF_SYS_BOOT1,
	[4] = CONTROL_PADCONF_SYS_BOOT2,
	[5] = CONTROL_PADCONF_SYS_BOOT3,
	[6] = CONTROL_PADCONF_SYS_BOOT3,
	[7] = CONTROL_PADCONF_SYS_BOOT5,
	[8] = CONTROL_PADCONF_SYS_BOOT6,
	[9] = CONTROL_PADCONF_SYS_OFF_MODE,
	[10] = CONTROL_PADCONF_SYS_CLKOUT1,
	[11] = CONTROL_PADCONF_JTAG_EMU0,
	[12] = CONTROL_PADCONF_ETK_CLK,
	[13] = CONTROL_PADCONF_ETK_CTL,
	[14] = CONTROL_PADCONF_ETK_D0,
	[15] = CONTROL_PADCONF_ETK_D1,
	[16] = CONTROL_PADCONF_ETK_D2,
	[17] = CONTROL_PADCONF_ETK_D3,
	[18] = CONTROL_PADCONF_ETK_D4,
	[19] = CONTROL_PADCONF_ETK_D5,
	[20] = CONTROL_PADCONF_ETK_D6,
	[21] = CONTROL_PADCONF_ETK_D7,
	[22] = CONTROL_PADCONF_ETK_D8,
	[23] = CONTROL_PADCONF_ETK_D9,
	[24] = CONTROL_PADCONF_ETK_D10,
	[25] = CONTROL_PADCONF_ETK_D11,
	[26] = CONTROL_PADCONF_ETK_D12,
	[27] = CONTROL_PADCONF_ETK_D13,
	[28] = CONTROL_PADCONF_ETK_D14,
	[29] = CONTROL_PADCONF_ETK_D15,
	[30] = CONTROL_PADCONF_SYS_NRESWARM,
	[31] = CONTROL_PADCONF_JTAG_EMU1,
	[32] = CONTROL_PADCONF_SAD2D_FRINT,
	[33] = 0,  /* no padconf for gpio_33 */
	[34] = CONTROL_PADCONF_GPMC_A1,
	[35] = CONTROL_PADCONF_GPMC_A2,
	[36] = CONTROL_PADCONF_GPMC_A3,
	[37] = CONTROL_PADCONF_GPMC_A4,
	[38] = CONTROL_PADCONF_GPMC_A5,
	[39] = CONTROL_PADCONF_GPMC_A6,
	[40] = CONTROL_PADCONF_GPMC_A7,
	[41] = CONTROL_PADCONF_GPMC_A8,
	[42] = CONTROL_PADCONF_GPMC_A9,
	[43] = CONTROL_PADCONF_GPMC_A10,
	[44] = CONTROL_PADCONF_GPMC_D8,
	[45] = CONTROL_PADCONF_GPMC_D9,
	[46] = CONTROL_PADCONF_GPMC_D10,
	[47] = CONTROL_PADCONF_GPMC_D11,
	[48] = CONTROL_PADCONF_GPMC_D12,
	[49] = CONTROL_PADCONF_GPMC_D13,
	[50] = CONTROL_PADCONF_GPMC_D14,
	[51] = CONTROL_PADCONF_GPMC_D15,
	[52] = CONTROL_PADCONF_GPMC_NCS1,
	[53] = CONTROL_PADCONF_GPMC_NCS2,
	[54] = CONTROL_PADCONF_GPMC_NCS3,
	[55] = CONTROL_PADCONF_GPMC_NCS4,
	[56] = CONTROL_PADCONF_GPMC_NCS5,
	[57] = CONTROL_PADCONF_GPMC_NCS6,
	[58] = CONTROL_PADCONF_GPMC_NCS7,
	[59] = CONTROL_PADCONF_GPMC_CLK,
	[60] = CONTROL_PADCONF_GPMC_NBE0_CLE,
	[61] = CONTROL_PADCONF_GPMC_NBE1,
	[62] = CONTROL_PADCONF_GPMC_NWP,
	[63] = CONTROL_PADCONF_GPMC_WAIT1,
	[64] = CONTROL_PADCONF_GPMC_WAIT2,
	[65] = CONTROL_PADCONF_GPMC_WAIT3,
	[66] = CONTROL_PADCONF_DSS_PCLK,
	[67] = CONTROL_PADCONF_DSS_HSYNC,
	[68] = CONTROL_PADCONF_DSS_VSYNC,
	[69] = CONTROL_PADCONF_DSS_ACBIAS,
	[70] = CONTROL_PADCONF_DSS_DATA0,
	[71] = CONTROL_PADCONF_DSS_DATA1,
	[72] = CONTROL_PADCONF_DSS_DATA2,
	[73] = CONTROL_PADCONF_DSS_DATA3,
	[74] = CONTROL_PADCONF_DSS_DATA4,
	[75] = CONTROL_PADCONF_DSS_DATA5,
	[76] = CONTROL_PADCONF_DSS_DATA6,
	[77] = CONTROL_PADCONF_DSS_DATA7,
	[78] = CONTROL_PADCONF_DSS_DATA8,
	[79] = CONTROL_PADCONF_DSS_DATA9,
	[80] = CONTROL_PADCONF_DSS_DATA10,
	[81] = CONTROL_PADCONF_DSS_DATA11,
	[82] = CONTROL_PADCONF_DSS_DATA12,
	[83] = CONTROL_PADCONF_DSS_DATA13,
	[84] = CONTROL_PADCONF_DSS_DATA14,
	[85] = CONTROL_PADCONF_DSS_DATA15,
	[86] = CONTROL_PADCONF_DSS_DATA16,
	[87] = CONTROL_PADCONF_DSS_DATA17,
	[88] = CONTROL_PADCONF_DSS_DATA18,
	[89] = CONTROL_PADCONF_DSS_DATA19,
	[90] = CONTROL_PADCONF_DSS_DATA20,
	[91] = CONTROL_PADCONF_DSS_DATA21,
	[92] = CONTROL_PADCONF_DSS_DATA22,
	[93] = CONTROL_PADCONF_DSS_DATA23,
	[94] = CONTROL_PADCONF_CAM_HS,
	[95] = CONTROL_PADCONF_CAM_VS,
	[96] = CONTROL_PADCONF_CAM_XCLKA,
	[97] = CONTROL_PADCONF_CAM_PCLK,
	[98] = CONTROL_PADCONF_CAM_FLD,
	[99] = CONTROL_PADCONF_CAM_D0,
	[100] = CONTROL_PADCONF_CAM_D1,
	[101] = CONTROL_PADCONF_CAM_D2,
	[102] = CONTROL_PADCONF_CAM_D3,
	[103] = CONTROL_PADCONF_CAM_D4,
	[104] = CONTROL_PADCONF_CAM_D5,
	[105] = CONTROL_PADCONF_CAM_D6,
	[106] = CONTROL_PADCONF_CAM_D7,
	[107] = CONTROL_PADCONF_CAM_D8,
	[108] = CONTROL_PADCONF_CAM_D9,
	[109] = CONTROL_PADCONF_CAM_D10,
	[110] = CONTROL_PADCONF_CAM_D11,
	[111] = CONTROL_PADCONF_CAM_XCLKB,
	[112] = CONTROL_PADCONF_CSI2_DX0,
	[113] = CONTROL_PADCONF_CSI2_DY0,
	[114] = CONTROL_PADCONF_CSI2_DX1,
	[115] = CONTROL_PADCONF_CSI2_DY1,
	[116] = CONTROL_PADCONF_MCBSP2_FSX,
	[117] = CONTROL_PADCONF_MCBSP2_CLKX,
	[118] = CONTROL_PADCONF_MCBSP2_DR,
	[119] = CONTROL_PADCONF_MCBSP2_DX,
	[120] = CONTROL_PADCONF_MMC1_CLK,
	[121] = CONTROL_PADCONF_MMC1_CMD,
	[122] = CONTROL_PADCONF_MMC1_DAT0,
	[123] = CONTROL_PADCONF_MMC1_DAT1,
	[124] = CONTROL_PADCONF_MMC1_DAT2,
	[125] = CONTROL_PADCONF_MMC1_DAT3,
	[126] = CONTROL_PADCONF_MMC1_DAT4,  /* also CONTROL_PADCONF_CAM_STROBE */
	[127] = CONTROL_PADCONF_MMC1_DAT5,
	[128] = CONTROL_PADCONF_MMC1_DAT6,
	[129] = CONTROL_PADCONF_MMC1_DAT7,
	[130] = CONTROL_PADCONF_MMC2_CLK,
	[131] = CONTROL_PADCONF_MMC2_CMD,
	[132] = CONTROL_PADCONF_MMC2_DAT0,
	[133] = CONTROL_PADCONF_MMC2_DAT1,
	[134] = CONTROL_PADCONF_MMC2_DAT2,
	[135] = CONTROL_PADCONF_MMC2_DAT3,
	[136] = CONTROL_PADCONF_MMC2_DAT4,
	[137] = CONTROL_PADCONF_MMC2_DAT5,
	[138] = CONTROL_PADCONF_MMC2_DAT6,
	[139] = CONTROL_PADCONF_MMC2_DAT7,
	[140] = CONTROL_PADCONF_MCBSP3_DX,
	[141] = CONTROL_PADCONF_MCBSP3_DR,
	[142] = CONTROL_PADCONF_MCBSP3_CLKX,
	[143] = CONTROL_PADCONF_MCBSP3_FSX,
	[144] = CONTROL_PADCONF_UART2_CTS,
	[145] = CONTROL_PADCONF_UART2_RTS,
	[146] = CONTROL_PADCONF_UART2_TX,
	[147] = CONTROL_PADCONF_UART2_RX,
	[148] = CONTROL_PADCONF_UART1_TX,
	[149] = CONTROL_PADCONF_UART1_RTS,
	[150] = CONTROL_PADCONF_UART1_CTS,
	[151] = CONTROL_PADCONF_UART1_RX,
	[152] = CONTROL_PADCONF_MCBSP4_CLKX,
	[153] = CONTROL_PADCONF_MCBSP4_DR,
	[154] = CONTROL_PADCONF_MCBSP4_DX,
	[155] = CONTROL_PADCONF_MCBSP4_FSX,
	[156] = CONTROL_PADCONF_MCBSP1_CLKR,
	[157] = CONTROL_PADCONF_MCBSP1_FSR,
	[158] = CONTROL_PADCONF_MCBSP1_DX,
	[159] = CONTROL_PADCONF_MCBSP1_DR,
	[160] = CONTROL_PADCONF_MCBSP_CLKS,
	[161] = CONTROL_PADCONF_MCBSP1_FSX,
	[162] = CONTROL_PADCONF_MCBSP1_CLKX,
	[163] = CONTROL_PADCONF_UART3_CTS_RCTX,
	[164] = CONTROL_PADCONF_UART3_RTS_SD,
	[165] = CONTROL_PADCONF_UART3_RX_IRRX,
	[166] = CONTROL_PADCONF_UART3_TX_IRTX,
	[167] = CONTROL_PADCONF_CAM_WEN,
	[168] = CONTROL_PADCONF_I2C2_SCL,
	[169] = CONTROL_PADCONF_HSUSB0_DATA3,
	[170] = CONTROL_PADCONF_HDQ_SIO,
	[171] = CONTROL_PADCONF_MCSPI1_CLK,
	[172] = CONTROL_PADCONF_MCSPI1_SIMO,
	[173] = CONTROL_PADCONF_MCSPI1_SOMI,
	[174] = CONTROL_PADCONF_MCSPI1_CS0,
	[175] = CONTROL_PADCONF_MCSPI1_CS1,
	[176] = CONTROL_PADCONF_MCSPI1_CS2,
	[177] = CONTROL_PADCONF_MCSPI1_CS3,
	[178] = CONTROL_PADCONF_MCSPI2_CLK,
	[179] = CONTROL_PADCONF_MCSPI2_SIMO,
	[180] = CONTROL_PADCONF_MCSPI2_SOMI,
	[181] = CONTROL_PADCONF_MCSPI2_CS0,
	[182] = CONTROL_PADCONF_MCSPI2_CS1,
	[183] = CONTROL_PADCONF_I2C2_SDA,
	[184] = CONTROL_PADCONF_I2C3_SCL,
	[185] = CONTROL_PADCONF_I2C3_SDA,
	[186] = CONTROL_PADCONF_SYS_CLKOUT2,
	[187] = CONTROL_PADCONF_SAD2D_SPINT,
	[188] = CONTROL_PADCONF_HSUSB0_DATA4,
	[189] = CONTROL_PADCONF_HSUSB0_DATA5,
	[190] = CONTROL_PADCONF_HSUSB0_DATA6,
	[191] = CONTROL_PADCONF_HSUSB0_DATA7,
};



/**
 *	
 */
struct omap3_scm_softc {
	device_t            sc_dev;
	
	bus_space_tag_t     sc_iotag;
	bus_space_handle_t  sc_ioh;

};

static struct omap3_scm_softc *g_omap3_scm_sc = NULL;



/**
 *	omap3_scm_reads - reads a 16-bit value from one of the PADCONFS registers
 *	@sc: PinMux device context
 *	@off: The offset of a register from the SCM register address range
 *
 *
 *	RETURNS:
 *	16-bit value read from the register.
 */
static inline uint16_t
omap3_scm_reads(struct omap3_scm_softc *sc, bus_size_t off)
{
	return bus_space_read_2(sc->sc_iotag, sc->sc_ioh, off);
}

/**
 *	omap3_scm_writes - writes a 16-bit value to one of the PADCONFS registers
 *	@sc: PinMux device context
 *	@off: The offset of a register from the SCM register address range
 *
 *
 *	RETURNS:
 *	nothing
 */
static inline void
omap3_scm_writes(struct omap3_scm_softc *sc, bus_size_t off, uint16_t val)
{
	bus_space_write_2(sc->sc_iotag, sc->sc_ioh, off, val);
}


/**
 *	omap3_scm_readl - reads a 32-bit value from one of the PADCONFS registers
 *	@sc: SCM device context
 *	@off: The offset of a register from the SCM register address range
 *
 *
 *	RETURNS:
 *	32-bit value read from the register.
 */
static inline uint32_t
omap3_scm_readl(struct omap3_scm_softc *sc, bus_size_t off)
{
	return bus_space_read_4(sc->sc_iotag, sc->sc_ioh, off);
}

/**
 *	omap3_scm_writel - writes a 32-bit value to one of the PADCONFS registers
 *	@sc: SCM device context
 *	@off: The offset of a register from the SCM register address range
 *
 *
 *	RETURNS:
 *	nothing
 */
static inline void
omap3_scm_writel(struct omap3_scm_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_iotag, sc->sc_ioh, off, val);
}






/**
 *	omap3_scm_probe - driver probe function
 *	@dev: pinumx device handle
 *
 *	Simply sets the name of the driver
 *
 *
 *	RETURNS:
 *	Returns 0 on sucess, a negative error code on failure.
 */
static int
omap3_scm_probe(device_t dev)
{
	
	device_set_desc(dev, "TI OMAP3 System Control Module");
	return (0);
}




/**
 *	omap3_scm_attach - driver attach function
 *	@dev: scm device handle
 *
 *	Sets up the driver data structure and initialises all the fields.
 *
 *	RETURNS:
 *	Returns 0 on sucess, a negative error code on failure.
 */
static int
omap3_scm_attach(device_t dev)
{
	struct omap3_scm_softc *sc = device_get_softc(dev);

	/* Setup the basics */
	sc->sc_dev = dev;
	sc->sc_iotag = &omap3_bs_tag;
	

	/* Map in the SCM register set, contains the PADCONF registers */
	if (bus_space_map(sc->sc_iotag, OMAP35XX_SCM_HWBASE, OMAP35XX_SCM_SIZE,
					  0, &sc->sc_ioh)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}

	/* Store the scm structure globally, this driver should never be unloaded */
	g_omap3_scm_sc = sc;
	
	return (0);
}


static device_method_t g_omap3_scm_methods[] = {
	DEVMETHOD(device_probe,		omap3_scm_probe),
	DEVMETHOD(device_attach,	omap3_scm_attach),
	{ 0, 0 }
};

static driver_t g_omap3_scm_driver = {
	"omap3_scm",
	g_omap3_scm_methods,
	sizeof(struct omap3_scm_softc),
};

static devclass_t g_omap3_scm_devclass;

DRIVER_MODULE(omap3_scm, omap3, g_omap3_scm_driver, g_omap3_scm_devclass, 0, 0);






/**
 *	omap3_scm_padconf_set - requests 'exclusive' access to the GPIO pin.
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
omap3_scm_padconf_set(uint32_t padconf, unsigned int mode, unsigned int state)
{
	struct omap3_scm_softc *sc = g_omap3_scm_sc;
	uint16_t val;

	/* sanity checking */
	if (sc == NULL)
		return(-ENOMEM);

	/* populate the new value for the PADCONF register */
	val = (uint16_t)(state & 0xFF18) | (uint16_t)(mode & 0x7);
	
	/* write the register value (16-bit writes) */
	omap3_scm_writes(sc, padconf, val);
	
	printf("[BRG] %s : %d : val = 0x%04x : 0x%04x[0x%04x] => 0x%08x\n", __func__, __LINE__,
		val, padconf, (padconf & ~0x3), omap3_scm_readl(sc, (padconf & ~0x3)));

	return 0;
}

/**
 *	omap3_scm_padconf_set_gpiomode - converts a pad to GPIO mode.
 *	@pin: the pin number (0-191)
 *	@state: the input/output, pull-up/pull-down state, can be one
 *	        of the following values:
 *	          - PADCONF_PIN_OUTPUT
 *	          - PADCONF_PIN_INPUT
 *	          - PADCONF_PIN_INPUT_PULLUP
 *	          - PADCONF_PIN_INPUT_PULLDOWN
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
omap3_scm_padconf_set_gpiomode(uint32_t pin, unsigned int state)
{
	struct omap3_scm_softc *sc = g_omap3_scm_sc;
	uint16_t val;
	uint16_t mode;
	uint32_t padconf;
	uint32_t max_gpio = sizeof(omap3_scm_gpio_map) / sizeof(omap3_scm_gpio_map[0]);
	
	/* sanity checking */
	if (sc == NULL)
		return(-ENOMEM);
	if (pin >= max_gpio)
		return(-EINVAL);
	
	/* get the corresponding PADCONF register for the GPIO */
	padconf = omap3_scm_gpio_map[pin];
	if (padconf != 0) {
	
		/* mode is always 4 for GPIO pins */
		mode = 4;

		/* populate the new value for the PADCONF register */
		val = (uint16_t)(state & 0xFF18) | (uint16_t)(mode & 0x7);
		
		/* write the register value (16-bit writes) */
		omap3_scm_writes(sc, padconf, val);
		
		printf("[BRG] %s : %d : val = 0x%04x : 0x%04x[0x%04x] => 0x%08x\n", __func__, __LINE__,
			val, padconf, (padconf & ~0x3), omap3_scm_readl(sc, (padconf & ~0x3)));
	}
	
	return 0;
}


/**
 *	omap3_scm_padconf_get - requests 'exclusive' access to the GPIO pin.
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
omap3_scm_padconf_get(uint32_t padconf, unsigned int *mode, unsigned int *state)
{
	struct omap3_scm_softc *sc = g_omap3_scm_sc;
	uint16_t val;

	/* sanity checking */
	if (sc == NULL)
		return(-ENOMEM);

	val = omap3_scm_reads(sc, padconf);
	
	if (mode != NULL)
		*mode = (val & 0x7);
	if (state != NULL)
		*state = (val & 0xFF18);

	return 0;
}



