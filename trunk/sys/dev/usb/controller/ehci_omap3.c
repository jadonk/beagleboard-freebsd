/*-
 * Copyright (c) 2008 Sam Leffler.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * 
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_bus.h"

#include <sys/stdint.h>
#include <sys/stddef.h>
#include <sys/param.h>
#include <sys/queue.h>
#include <sys/types.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/linker_set.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/sysctl.h>
#include <sys/sx.h>
#include <sys/unistd.h>
#include <sys/callout.h>
#include <sys/malloc.h>
#include <sys/priv.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_util.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/ehci.h>
#include <dev/usb/controller/ehcireg.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>

#include <arm/cortexa8/omap3/omap3_prcm.h>
#include <arm/cortexa8/omap3/omap3_gpio.h>
#include <arm/cortexa8/omap3/omap3_scm.h>


#define TLL_SHARED_CONF_USB_90D_DDR_EN			(1UL << 6)
#define TLL_SHARED_CONF_USB_180D_SDR_EN			(1UL << 5)
#define TLL_SHARED_CONF_USB_DIVRATIO_MASK		(7UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_128		(7UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_64			(6UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_32			(5UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_16			(4UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_8			(3UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_4			(2UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_2			(1UL << 2)
#define TLL_SHARED_CONF_USB_DIVRATIO_1			(0UL << 2)
#define TLL_SHARED_CONF_FCLK_REQ				(1UL << 1)
#define TLL_SHARED_CONF_FCLK_IS_ON				(1UL << 0)

#define TLL_CHANNEL_CONF_DRVVBUS				(1UL << 16)
#define TLL_CHANNEL_CONF_CHRGVBUS				(1UL << 15)
#define TLL_CHANNEL_CONF_ULPINOBITSTUFF			(1UL << 11)
#define TLL_CHANNEL_CONF_ULPIAUTOIDLE			(1UL << 10)
#define TLL_CHANNEL_CONF_UTMIAUTOIDLE			(1UL << 9)
#define TLL_CHANNEL_CONF_ULPIDDRMODE			(1UL << 8)
#define TLL_CHANNEL_CONF_ULPIOUTCLKMODE			(1UL << 7)
#define TLL_CHANNEL_CONF_TLLFULLSPEED			(1UL << 6)
#define TLL_CHANNEL_CONF_TLLCONNECT				(1UL << 5)
#define TLL_CHANNEL_CONF_TLLATTACH				(1UL << 4)
#define TLL_CHANNEL_CONF_UTMIISADEV				(1UL << 3)
#define TLL_CHANNEL_CONF_CHANEN					(1UL << 0)



#define UHH_SYSCONFIG_MIDLEMODE_MASK			(3UL << 12)
#define UHH_SYSCONFIG_MIDLEMODE_SMARTSTANDBY	(2UL << 12)
#define UHH_SYSCONFIG_MIDLEMODE_NOSTANDBY		(1UL << 12)
#define UHH_SYSCONFIG_MIDLEMODE_FORCESTANDBY	(0UL << 12)
#define UHH_SYSCONFIG_CLOCKACTIVITY				(1UL << 8)
#define UHH_SYSCONFIG_SIDLEMODE_MASK			(3UL << 3)
#define UHH_SYSCONFIG_SIDLEMODE_SMARTIDLE		(2UL << 3)
#define UHH_SYSCONFIG_SIDLEMODE_NOIDLE			(1UL << 3)
#define UHH_SYSCONFIG_SIDLEMODE_FORCEIDLE		(0UL << 3)
#define UHH_SYSCONFIG_ENAWAKEUP					(1UL << 2)
#define UHH_SYSCONFIG_SOFTRESET					(1UL << 1)
#define UHH_SYSCONFIG_AUTOIDLE					(1UL << 0)

#define UHH_HOSTCONFIG_P3_CONNECT_STATUS		(1UL << 10)
#define UHH_HOSTCONFIG_P2_CONNECT_STATUS		(1UL << 9)
#define UHH_HOSTCONFIG_P1_CONNECT_STATUS		(1UL << 8)
#define UHH_HOSTCONFIG_ENA_INCR_ALIGN			(1UL << 5)
#define UHH_HOSTCONFIG_ENA_INCR16				(1UL << 4)
#define UHH_HOSTCONFIG_ENA_INCR8				(1UL << 3)
#define UHH_HOSTCONFIG_ENA_INCR4				(1UL << 2)
#define UHH_HOSTCONFIG_AUTOPPD_ON_OVERCUR_EN	(1UL << 1)
#define UHH_HOSTCONFIG_P1_ULPI_BYPASS			(1UL << 0)




#define EHCI_VENDORID_OMAP3		0x42fa05
#define OMAP3_EHCI_HC_DEVSTR	"Texas Instruments OMAP USB 2.0 controller"

#define EHCI_HCD_OMAP3_MODE_UNKNOWN	0
#define EHCI_HCD_OMAP3_MODE_PHY		1
#define EHCI_HCD_OMAP3_MODE_TLL		2



struct omap3_ehci_softc {
	ehci_softc_t		base;	/* storage for EHCI code */

	device_t			sc_dev;

	bus_space_tag_t		sc_iotag_host;
	bus_space_handle_t	sc_ioh_host;

	struct resource*    sc_res_tll;
	bus_space_tag_t		sc_iotag_tll;
	bus_space_handle_t	sc_ioh_tll;
	
	int					port_mode[3];
	
	int					phy_reset;
	int					reset_gpio_pin[3];
};

static device_attach_t ehci_omap3_attach;
static device_detach_t ehci_omap3_detach;
static device_shutdown_t ehci_omap3_shutdown;
static device_suspend_t ehci_omap3_suspend;
static device_resume_t ehci_omap3_resume;

static inline uint32_t
omap3_tll_readl(struct omap3_ehci_softc *sc, bus_size_t off)
{
	return bus_space_read_4(sc->sc_iotag_tll, sc->sc_ioh_tll, off);
}
static inline void
omap3_tll_writel(struct omap3_ehci_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_iotag_tll, sc->sc_ioh_tll, off, val);
}
static inline uint8_t
omap3_tll_readb(struct omap3_ehci_softc *sc, bus_size_t off)
{
	return bus_space_read_1(sc->sc_iotag_tll, sc->sc_ioh_tll, off);
}
static inline void
omap3_tll_writeb(struct omap3_ehci_softc *sc, bus_size_t off, uint8_t val)
{
	bus_space_write_1(sc->sc_iotag_tll, sc->sc_ioh_tll, off, val);
}


static inline uint32_t
omap3_ehci_readl(struct omap3_ehci_softc *sc, bus_size_t off)
{
	return bus_space_read_4(sc->sc_iotag_host, sc->sc_ioh_host, off);
}
static inline void
omap3_ehci_writel(struct omap3_ehci_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->sc_iotag_host, sc->sc_ioh_host, off, val);
}





/**
 *	ehci_omap3_utmi_init - initialises the MMC/SD/SIO controller
 *	@dev: mmc device handle
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
ehci_omap3_utmi_init(struct omap3_ehci_softc *isc, unsigned int en_mask)
{
	unsigned int i;
	uint32_t reg;
	
	/* There are 3 TLL channels, one per USB controller so set them all up the
	 * same, SDR mode, bit stuffing and no autoidle.
	 */
	for (i=0; i<3; i++) {
		reg = omap3_tll_readl(isc, OMAP35XX_USBTLL_TLL_CHANNEL_CONF(i));
		
		reg &= ~(TLL_CHANNEL_CONF_UTMIAUTOIDLE
				 | TLL_CHANNEL_CONF_ULPINOBITSTUFF
				 | TLL_CHANNEL_CONF_ULPIDDRMODE);
		
		omap3_tll_writel(isc, OMAP35XX_USBTLL_TLL_CHANNEL_CONF(i), reg);
	}
	
	/* Program the common TLL register */
	reg = omap3_tll_readl(isc, OMAP35XX_USBTLL_TLL_SHARED_CONF);

	reg &= ~( TLL_SHARED_CONF_USB_90D_DDR_EN
			| TLL_SHARED_CONF_USB_DIVRATIO_MASK);
	reg |=  ( TLL_SHARED_CONF_FCLK_IS_ON
			| TLL_SHARED_CONF_USB_DIVRATIO_2
			| TLL_SHARED_CONF_USB_180D_SDR_EN);
	
	omap3_tll_writel(isc, OMAP35XX_USBTLL_TLL_SHARED_CONF, reg);
	
	/* Enable channels now */
	for (i = 0; i < 3; i++) {
		reg = omap3_tll_readl(isc, OMAP35XX_USBTLL_TLL_CHANNEL_CONF(i));
		
		/* Enable only the reg that is needed */
		if ((en_mask & (1 << i)) == 0)
			continue;
		
		reg |= TLL_CHANNEL_CONF_CHANEN;
		omap3_tll_writel(isc, OMAP35XX_USBTLL_TLL_CHANNEL_CONF(i), reg);
		
		omap3_tll_writeb(isc, OMAP35XX_USBTLL_ULPI_SCRATCH_REGISTER(i), 0xbe);
		device_printf(isc->sc_dev, "ULPI_SCRATCH_REG[ch=%d]= 0x%02x\n",
				i+1, omap3_tll_readb(isc, OMAP35XX_USBTLL_ULPI_SCRATCH_REGISTER(i)));
	}
}




/**
 *	ehci_omap3_init - initialises the MMC/SD/SIO controller
 *	@dev: mmc device handle
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
ehci_omap3_init(struct omap3_ehci_softc *isc)
{
//	unsigned long timeout;
	uint8_t tll_ch_mask = 0;
	unsigned reg = 0;
	int ret = 0;
	
	device_printf(isc->sc_dev, "starting TI EHCI USB Controller\n");
	
	
	/* Enable Clocks for USBHOST */
	omap3_prcm_enable_clk(OMAP3_MODULE_USBHOST_ICLK);
	omap3_prcm_enable_clk(OMAP3_MODULE_USBHOST_120M_FCLK);
	omap3_prcm_enable_clk(OMAP3_MODULE_USBHOST_48M_FCLK);
	


	/* Hold the PHY in reset while configuring */
	if (isc->phy_reset) {

		if (isc->reset_gpio_pin[0] != -1) {
			/* Set the pad as a GPIO pin and output */
			omap3_scm_padconf_set_gpiomode(isc->reset_gpio_pin[0],
			                               PADCONF_PIN_OUTPUT);
			/* Configure the GPIO to drive low (hold in reset) */
			omap3_gpio_request(isc->reset_gpio_pin[0], "USB1 PHY reset");
			omap3_gpio_direction_output(isc->reset_gpio_pin[0], 0);
		}
		
		if (isc->reset_gpio_pin[1] != -1) {
			/* Set the pad as a GPIO pin and output */
			omap3_scm_padconf_set_gpiomode(isc->reset_gpio_pin[1],
			                               PADCONF_PIN_OUTPUT);
			/* Configure the GPIO to drive low (hold in reset) */
			omap3_gpio_request(isc->reset_gpio_pin[1], "USB2 PHY reset");
			omap3_gpio_direction_output(isc->reset_gpio_pin[1], 0);
		}
		
		/* Hold the PHY in RESET for enough time till DIR is high */
		DELAY(10);
	}

	/* Configure TLL for 60Mhz clk for ULPI */
	omap3_prcm_enable_clk(OMAP3_MODULE_USBTLL_FCLK);
	omap3_prcm_enable_clk(OMAP3_MODULE_USBTLL_ICLK);
	
	omap3_prcm_accessible(OMAP3_MODULE_USBTLL);
	
	
#if 0	/* AARRGGHH - For some reason the TLL module doesn't come out of reset */

	/* perform TLL soft reset, and wait until reset is complete */
	omap3_tll_writel(isc, OMAP35XX_USBTLL_SYSCONFIG, 0x02);
	
	/* Set the timeout */
	if (hz < 10)
		timeout = 1;
	else
		timeout = (100 * hz) / 1000;

	/* Wait for TLL reset to complete */
	while ((omap3_tll_readl(isc, OMAP35XX_USBTLL_SYSSTATUS) & 0x01) == 0x00) {

		/* Sleep for a tick */
		pause("USBRESET", 1);
		
		if (timeout-- == 0) {
			device_printf(isc->sc_dev, "operation timed out\n");
			ret = -EINVAL;
			goto err_sys_status;
		}
	}
	
	device_printf(isc->sc_dev, "TLL RESET DONE\n");
	
	/* 
	 * CLOCKACTIVITY = 1 : OCP-derived internal clocks ON during idle
	 * SIDLEMODE = 1     : No-idle mode. Sidleack never asserted
	 * ENAWAKEUP = 1     : Wakeup generation enabled
	 */
	omap3_tll_writel(isc, OMAP35XX_USBTLL_SYSCONFIG, 0x010c);
#endif	
	
	/* Put UHH in NoIdle/NoStandby mode */
	reg = omap3_ehci_readl(isc, OMAP35XX_USBHOST_UHH_SYSCONFIG);
	reg &= ~(UHH_SYSCONFIG_SIDLEMODE_MASK |
			 UHH_SYSCONFIG_MIDLEMODE_MASK |
			 UHH_SYSCONFIG_AUTOIDLE);
	reg |= (UHH_SYSCONFIG_ENAWAKEUP |
			UHH_SYSCONFIG_SIDLEMODE_NOIDLE |
			UHH_SYSCONFIG_CLOCKACTIVITY |
			UHH_SYSCONFIG_MIDLEMODE_NOSTANDBY);
	omap3_ehci_writel(isc, OMAP35XX_USBHOST_UHH_SYSCONFIG, reg);

	
	reg = omap3_ehci_readl(isc, OMAP35XX_USBHOST_UHH_HOSTCONFIG);
	
	/* setup ULPI bypass and burst configurations */
	reg |= (UHH_HOSTCONFIG_ENA_INCR4 |
			UHH_HOSTCONFIG_ENA_INCR8 |
			UHH_HOSTCONFIG_ENA_INCR16);
	reg &= ~UHH_HOSTCONFIG_ENA_INCR_ALIGN;
	
	if (isc->port_mode[0] == EHCI_HCD_OMAP3_MODE_UNKNOWN)
		reg &= ~UHH_HOSTCONFIG_P1_CONNECT_STATUS;
	if (isc->port_mode[1] == EHCI_HCD_OMAP3_MODE_UNKNOWN)
		reg &= ~UHH_HOSTCONFIG_P2_CONNECT_STATUS;
	if (isc->port_mode[2] == EHCI_HCD_OMAP3_MODE_UNKNOWN)
		reg &= ~UHH_HOSTCONFIG_P3_CONNECT_STATUS;
	
	/* Bypass the TLL module for PHY mode operation */
	if ((isc->port_mode[0] == EHCI_HCD_OMAP3_MODE_PHY) ||
		(isc->port_mode[1] == EHCI_HCD_OMAP3_MODE_PHY) ||
		(isc->port_mode[2] == EHCI_HCD_OMAP3_MODE_PHY))
		reg &= ~UHH_HOSTCONFIG_P1_ULPI_BYPASS;
	else
		reg |= UHH_HOSTCONFIG_P1_ULPI_BYPASS;

	omap3_ehci_writel(isc, OMAP35XX_USBHOST_UHH_HOSTCONFIG, reg);
	device_printf(isc->sc_dev, "UHH setup done, uhh_hostconfig=%x\n", reg);
	
	
	if ((isc->port_mode[0] == EHCI_HCD_OMAP3_MODE_TLL) ||
		(isc->port_mode[1] == EHCI_HCD_OMAP3_MODE_TLL) ||
		(isc->port_mode[2] == EHCI_HCD_OMAP3_MODE_TLL)) {
		
		if (isc->port_mode[0] == EHCI_HCD_OMAP3_MODE_TLL)
			tll_ch_mask |= 0x1;
		if (isc->port_mode[1] == EHCI_HCD_OMAP3_MODE_TLL)
			tll_ch_mask |= 0x2;
		if (isc->port_mode[2] == EHCI_HCD_OMAP3_MODE_TLL)
			tll_ch_mask |= 0x4;
		
		/* Enable UTMI mode for required TLL channels */
		ehci_omap3_utmi_init(isc, tll_ch_mask);
	}
	
	if (isc->phy_reset) {
		/* Refer ISSUE1:
		 * Hold the PHY in RESET for enough time till
		 * PHY is settled and ready
		 */
		DELAY(10);
		
		if (isc->reset_gpio_pin[0] != -EINVAL)
			omap3_gpio_pin_set(isc->reset_gpio_pin[0], 1);
		
		if (isc->reset_gpio_pin[1] != -EINVAL)
			omap3_gpio_pin_set(isc->reset_gpio_pin[1], 1);
	}

	return(0);

#if 0
err_sys_status:
	
	/* Configure TLL for 60Mhz clk for ULPI */
	omap3_prcm_disable_clk(OMAP3_MODULE_USBTLL_FCLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_USBTLL_ICLK);
	
	/* Free the reset lines GPIO */
	omap3_gpio_free(isc->reset_gpio_pin[0]);
	omap3_gpio_free(isc->reset_gpio_pin[1]);
	
	/* Disable Clocks for USBHOST */
	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_ICLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_120M_FCLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_48M_FCLK);

	return(ret);
#endif
}


/**
 *	omap3_mmc_fini - shutdown the MMC/SD/SIO controller
 *	@dev: mmc device handle
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
ehci_omap3_fini(struct omap3_ehci_softc *isc)
{
	unsigned long timeout;
	
	device_printf(isc->sc_dev, "stopping TI EHCI USB Controller\n");
	
	/* Set the timeout */
	if (hz < 10)
		timeout = 1;
	else
		timeout = (100 * hz) / 1000;

	/* Reset the UHH, OHCI and EHCI modules */
	omap3_ehci_writel(isc, OMAP35XX_USBHOST_UHH_SYSCONFIG, 0x0002);
	while ((omap3_ehci_readl(isc, OMAP35XX_USBHOST_UHH_SYSSTATUS) & 0x07) == 0x00) {
		
		/* Sleep for a tick */
		pause("USBRESET", 1);
		
		if (timeout-- == 0) {
			device_printf(isc->sc_dev, "operation timed out\n");
			break;
		}
	}
	

	/* Set the timeout */
	if (hz < 10)
		timeout = 1;
	else
		timeout = (100 * hz) / 1000;

	/* Reset the TLL module */
	omap3_tll_writel(isc, OMAP35XX_USBTLL_SYSCONFIG, 0x0002);
	while ((omap3_tll_readl(isc, OMAP35XX_USBTLL_SYSSTATUS) & (0x01)) == 0x00) {
		/* Sleep for a tick */
		pause("USBRESET", 1);
		
		if (timeout-- == 0) {
			device_printf(isc->sc_dev, "operation timed out\n");
			break;
		}
	}

	/* Disable functional and interface clocks for the TLL and HOST modules */
	omap3_prcm_disable_clk(OMAP3_MODULE_USBTLL_FCLK);

	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_ICLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_48M_FCLK);
	omap3_prcm_disable_clk(OMAP3_MODULE_USBHOST_120M_FCLK);
	
	omap3_prcm_disable_clk(OMAP3_MODULE_USBTLL_ICLK);

	
	/* Release the GPIO lines */
	if (isc->phy_reset) {
		if (isc->reset_gpio_pin[0] != -EINVAL)
			omap3_gpio_free(isc->reset_gpio_pin[0]);
		
		if (isc->reset_gpio_pin[1] != -EINVAL)
			omap3_gpio_free(isc->reset_gpio_pin[1]);
	}
	
	device_printf(isc->sc_dev, "Clock to USB host has been disabled\n");
	
}


/**
 *	ehci_omap3_conf_pinmux - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI suspend code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_conf_pinmux(struct omap3_ehci_softc *isc)
{
	switch (isc->port_mode[0]) {
	case EHCI_HCD_OMAP3_MODE_PHY:
printf("[BRG] Configuring port 0 in PHY mode\n");
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_CLK, 3, /* hsusb1_stp */
		                      PADCONF_PIN_OUTPUT);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_CTL, 3, /* hsusb1_clk */
		                      PADCONF_PIN_OUTPUT);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D8,  3, /* hsusb1_dir */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D9,  3, /* hsusb1_clk */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D0,  3, /* hsusb1_nxt */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D1,  3, /* hsusb1_data0 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D2,  3, /* hsusb1_data1 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D3,  3, /* hsusb1_data2 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D4,  3, /* hsusb1_data3 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D5,  3, /* hsusb1_data4 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D6,  3, /* hsusb1_data5 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D7,  3, /* hsusb1_data6 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		break;
	case EHCI_HCD_OMAP3_MODE_TLL:
printf("[BRG] Configuring port 0 in TLL mode\n");
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_CLK, 6, /* hsusb1_tll_stp */
		                      PADCONF_PIN_INPUT_PULLUP);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_CTL, 6, /* hsusb1_tll_clk */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D8,  6, /* hsusb1_tll_dir */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D9,  6, /* hsusb1_tll_nxt */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D0,  6, /* hsusb1_tll_data0 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D1,  6, /* hsusb1_tll_data1 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D2,  6, /* hsusb1_tll_data2 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D3,  6, /* hsusb1_tll_data3 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D4,  6, /* hsusb1_tll_data4 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D5,  6, /* hsusb1_tll_data5 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D6,  6, /* hsusb1_tll_data6 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D7,  6, /* hsusb1_tll_data7 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		break;
	case EHCI_HCD_OMAP3_MODE_UNKNOWN:
		/* FALLTHROUGH */
	default:
		break;
	}

	switch (isc->port_mode[1]) {
	case EHCI_HCD_OMAP3_MODE_PHY:
printf("[BRG] Configuring port 1 in PHY mode\n");
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D11,     3, /* hsusb2_stp */
		                      PADCONF_PIN_OUTPUT);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D10,     3, /* hsusb2_clk */
		                      PADCONF_PIN_OUTPUT);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D12,     3, /* hsusb2_dir */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D13,     3, /* hsusb2_nxt */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D14,     3, /* hsusb2_data0 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D15,     3, /* hsusb2_data1 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI1_CS3,  3, /* hsusb2_data2 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CS1,  3, /* hsusb2_data3 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_SIMO, 3, /* hsusb2_data4 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_SOMI, 3, /* hsusb2_data5 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CS0,  3, /* hsusb2_data6 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CLK,  3, /* hsusb2_data7 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		break;
	case EHCI_HCD_OMAP3_MODE_TLL:
printf("[BRG] Configuring port 1 in TLL mode\n");
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D11,     6, /* hsusb2_tll_stp */
		                      PADCONF_PIN_INPUT_PULLUP);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D10,     6, /* hsusb2_tll_clk */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D12,     6, /* hsusb2_tll_dir */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D13,     6, /* hsusb2_tll_nxt */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D14,     6, /* hsusb2_tll_data0 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_ETK_D15,     6, /* hsusb2_tll_data1 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI1_CS3,  2, /* hsusb2_tll_data2 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CS1,  2, /* hsusb2_tll_data3 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_SIMO, 2, /* hsusb2_tll_data4 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_SOMI, 2, /* hsusb2_tll_data5 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CS0,  2, /* hsusb2_tll_data6 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCSPI2_CLK,  2, /* hsusb2_tll_data7 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		break;
	case EHCI_HCD_OMAP3_MODE_UNKNOWN:
		/* FALLTHROUGH */
	default:
		break;
	}

	switch (isc->port_mode[2]) {
	case EHCI_HCD_OMAP3_MODE_PHY:
		device_printf(isc->sc_dev, "Port 3 can't be used in PHY mode\n");
		break;
	case EHCI_HCD_OMAP3_MODE_TLL:
printf("[BRG] Configuring port 3 in TLL mode\n");
		omap3_scm_padconf_set(CONTROL_PADCONF_MMC2_DAT5,   5, /* hsusb3_tll_stp */
		                      PADCONF_PIN_INPUT_PULLUP);
		omap3_scm_padconf_set(CONTROL_PADCONF_UART1_CTS,   5, /* hsusb3_tll_clk */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MMC2_DAT6,   5, /* hsusb3_tll_dir */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MMC2_DAT7,   5, /* hsusb3_tll_nxt */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP4_DR,   5, /* hsusb3_tll_data0 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP4_CLKX, 5, /* hsusb3_tll_data1 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP4_DX,   5, /* hsusb3_tll_data2 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP4_FSX,  5, /* hsusb3_tll_data3 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP3_DX,   5, /* hsusb3_tll_data4 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP3_DR,   5, /* hsusb3_tll_data5 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP3_CLKX, 5, /* hsusb3_tll_data6 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		omap3_scm_padconf_set(CONTROL_PADCONF_MCBSP3_FSX,  5, /* hsusb3_tll_data7 */
		                      PADCONF_PIN_INPUT_PULLDOWN);
		break;
	case EHCI_HCD_OMAP3_MODE_UNKNOWN:
		/* FALLTHROUGH */
	default:
		break;
	}




	return 0;
}


/**
 *	ehci_omap3_suspend - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI suspend code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_suspend(device_t dev)
{
	ehci_softc_t *sc = device_get_softc(dev);
	int err;
	
	err = bus_generic_suspend(dev);
	if (err)
		return (err);
	ehci_suspend(sc);
	return (0);
}


/**
 *	ehci_omap3_resume - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI resume code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_resume(device_t dev)
{
	ehci_softc_t *sc = device_get_softc(dev);
	
	ehci_resume(sc);
	
	bus_generic_resume(dev);
	
	return (0);
}


/**
 *	ehci_omap3_resume - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI resume code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_shutdown(device_t dev)
{
	ehci_softc_t *sc = device_get_softc(dev);
	int err;
	
	err = bus_generic_shutdown(dev);
	if (err)
		return (err);
	ehci_shutdown(sc);
	
	return (0);
}


/**
 *	ehci_omap3_resume - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI resume code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_probe(device_t dev)
{
printf("[BRG] %s : %d \n", __func__, __LINE__);
	
	device_set_desc(dev, OMAP3_EHCI_HC_DEVSTR);
	
	return (BUS_PROBE_DEFAULT);
}

/**
 *	ehci_omap3_attach - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI resume code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_attach(device_t dev)
{
	struct omap3_ehci_softc *isc = device_get_softc(dev);
	ehci_softc_t *sc = &isc->base;
	uint32_t val;
	int err;
	int rid;
	int rval;
   
printf("[BRG] %s : %d \n", __func__, __LINE__);
	
	/* initialise some bus fields */
	sc->sc_bus.parent = dev;
	sc->sc_bus.devices = sc->sc_devices;
	sc->sc_bus.devices_max = EHCI_MAX_DEVICES;
	
	/* save the device */
	isc->sc_dev = dev;
	
	/* get all DMA memory */
	if (usb_bus_mem_alloc_all(&sc->sc_bus, USB_GET_DMA_TAG(dev),
                             &ehci_iterate_hw_softc)) {
		return (ENOMEM);
	}
	
	
	
	/* [BRG] Allocate bus resources for TLL, UHH and USBHOST */
	rid = 0;
	sc->sc_io_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
                                      OMAP35XX_USBHOST_HWBASE,
                                      OMAP35XX_USBHOST_HWBASE + OMAP35XX_USBHOST_SIZE,
                                      OMAP35XX_USBHOST_SIZE,
                                      RF_ACTIVE);
	if (!sc->sc_io_res) {
		device_printf(dev, "Could not map memory\n");
		goto error;
	}
   
	/* Get the bus tag and handle so we can locally access the register set */
	isc->sc_iotag_host = rman_get_bustag(sc->sc_io_res);
	isc->sc_ioh_host = rman_get_bushandle(sc->sc_io_res);

	printf("[BRG] %s : %d : isc->sc_iotag_host = %p : isc->sc_ioh_host = 0x%08x\n",
		__func__, __LINE__, isc->sc_iotag_host, (uint32_t)isc->sc_ioh_host);
	
   
	/* Allocate bus resources for TLL controller */
#if 0	
	rid = 0;
	isc->sc_res_tll = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid,
	                                     OMAP35XX_USBTLL_HWBASE,
										 OMAP35XX_USBTLL_HWBASE + OMAP35XX_USBTLL_SIZE,
										 OMAP35XX_USBTLL_SIZE,
										 RF_ACTIVE);
	if (!isc->sc_res_tll) {
		device_printf(dev, "Could not map memory\n");
		goto error;
	}
	isc->sc_iotag_tll = rman_get_bustag(isc->sc_res_tll);
	isc->sc_ioh_tll = rman_get_bushandle(isc->sc_res_tll);

	printf("[BRG] %s : %d : isc->sc_iotag_tll = %p : isc->sc_ioh_tll = 0x%08x\n",
		__func__, __LINE__, isc->sc_iotag_tll, (uint32_t)isc->sc_ioh_tll);
#endif

	/* Map in the USBTLL register set */
	isc->sc_iotag_tll = &omap3_bs_tag;
	if (bus_space_map(isc->sc_iotag_tll, OMAP35XX_USBTLL_HWBASE, OMAP35XX_USBTLL_SIZE,
					  0, &isc->sc_ioh_tll)) {
		panic("%s: Cannot map registers", device_get_name(dev));
	}

	/* [BRG] Request an interrupt resource */
	rid = 0;
	sc->sc_irq_res = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, OMAP35XX_IRQ_EHCI,
                                       OMAP35XX_IRQ_EHCI, 1, RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "Could not allocate irq\n");
		goto error;
	}


	
	/* [BRG] add this device as a child of the USBus device */
	sc->sc_bus.bdev = device_add_child(dev, "usbus", -1);
	if (!sc->sc_bus.bdev) {
		device_printf(dev, "Could not add USB device\n");
		goto error;
	}
	device_set_ivars(sc->sc_bus.bdev, &sc->sc_bus);
	device_set_desc(sc->sc_bus.bdev, OMAP3_EHCI_HC_DEVSTR);
	
	/* Set the vendor name */
	sprintf(sc->sc_vendor, "Texas Instruments");
	
	
	/* [BRG] Setup the interrupt */
	err = bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
						 NULL, (driver_intr_t *)ehci_interrupt, sc, &sc->sc_intr_hdl);
	if (err) {
		device_printf(dev, "Could not setup irq, %d\n", err);
		sc->sc_intr_hdl = NULL;
		goto error;
	}
	
	
	/* Set the defaults for the hints */
	isc->phy_reset = 0;
	isc->reset_gpio_pin[0] = isc->reset_gpio_pin[1] = isc->reset_gpio_pin[2] = -1;
	isc->port_mode[0] = isc->port_mode[1] = isc->port_mode[2] = -1; 
	
	/* Hints are used to define the PHY interface settings */
	if (resource_int_value("ehci", 0, "phy_reset", &rval) == 0)
		isc->phy_reset = rval;
	printf("[BRG] %s : isc->phy_reset = %d\n", __func__, isc->phy_reset);

	if (isc->phy_reset) {
		if (resource_int_value("ehci", 0, "phy_reset_gpio_0", &rval) == 0)
			isc->reset_gpio_pin[0] = rval;
		if (resource_int_value("ehci", 0, "phy_reset_gpio_1", &rval) == 0)
			isc->reset_gpio_pin[1] = rval;
		if (resource_int_value("ehci", 0, "phy_reset_gpio_2", &rval) == 0)
			isc->reset_gpio_pin[2] = rval;

		printf("[BRG] %s : isc->phy_reset_gpio_0 = %d\n", __func__, isc->reset_gpio_pin[0]);
		printf("[BRG] %s : isc->phy_reset_gpio_1 = %d\n", __func__, isc->reset_gpio_pin[1]);
		printf("[BRG] %s : isc->phy_reset_gpio_2 = %d\n", __func__, isc->reset_gpio_pin[2]);
	}

	resource_int_value("ehci", 0, "phy_mode_0", &isc->port_mode[0]);
	resource_int_value("ehci", 0, "phy_mode_1", &isc->port_mode[1]);
	resource_int_value("ehci", 0, "phy_mode_2", &isc->port_mode[2]);


	/* Configure the pins for USB operations */
	err = ehci_omap3_conf_pinmux(isc);
	if (err) {
		device_printf(dev, "Failed to configure pins for USB, %d\n", err);
		goto error;
	}
	
//	.phy_reset  = true,
//	.reset_gpio_port[0]  = -EINVAL,
//	.reset_gpio_port[1]  = 147,
//	.reset_gpio_port[2]  = -EINVAL
	
	err = ehci_omap3_init(isc);
	if (err) {
		device_printf(dev, "Could not setup OMAP3 EHCI, %d\n", err);
		goto error;
	}
	
	/* SET 1 micro-frame Interrupt interval */
	val = omap3_ehci_readl(isc, OMAP35XX_USBHOST_USBCMD);
	val &= 0xff00ffff;
	val |= (1 << 16);
	omap3_ehci_writel(isc, OMAP35XX_USBHOST_USBCMD, val);
	
	
	/*
	 * OMAP EHCI host controller registers start at certain offset within
	 * the whole USB registers range, so create a subregion for the host
	 * mode configuration purposes.
	 */
	sc->sc_io_tag = rman_get_bustag(sc->sc_io_res);
	sc->sc_io_size = 0x100;

	if (bus_space_subregion(sc->sc_io_tag, rman_get_bushandle(sc->sc_io_res),
	                        OMAP35XX_USBHOST_HCCAPBASE, sc->sc_io_size,
							&sc->sc_io_hdl) != 0) {
		device_printf(dev, "Unable to subregion USB host registers");
		goto error;
	}


	/*
	 * Arrange to force Host mode, select big-endian byte alignment,
	 * and arrange to not terminate reset operations (the adapter
	 * will ignore it if we do but might as well save a reg write).
	 * Also, the controller has an embedded Transaction Translator
	 * which means port speed must be read from the Port Status
	 * register following a port enable.
	 */
	sc->sc_flags |= EHCI_SCFLG_TT
	| EHCI_SCFLG_SETMODE
	| EHCI_SCFLG_BIGEDESC
	| EHCI_SCFLG_BIGEMMIO
	| EHCI_SCFLG_NORESTERM
	;
	
	err = ehci_init(sc);
	if (!err) {
		err = device_probe_and_attach(sc->sc_bus.bdev);
	}
	if (err) {
		device_printf(dev, "USB init failed err=%d\n", err);
		goto error;
	}
	return (0);
	
error:
	ehci_omap3_detach(dev);
	return (ENXIO);
}

/**
 *	ehci_omap3_detach - starts the given command
 *	@dev: 
 *	
 *	Effectively boilerplate EHCI resume code.
 *
 *	LOCKING:
 *	Caller should be holding the OMAP3_MMC lock.
 *
 *	RETURNS:
 *	EH_HANDLED or EH_NOT_HANDLED
 */
static int
ehci_omap3_detach(device_t dev)
{
	struct omap3_ehci_softc *isc = device_get_softc(dev);
	ehci_softc_t *sc = &isc->base;
	device_t bdev;
	int err;
	
 	if (sc->sc_bus.bdev) {
		bdev = sc->sc_bus.bdev;
		device_detach(bdev);
		device_delete_child(dev, bdev);
	}
	/* during module unload there are lots of children leftover */
	device_delete_all_children(dev);
	
	/*
	 * disable interrupts that might have been switched on in ehci_init
	 */
	if (sc->sc_io_res) {
		EWRITE4(sc, EHCI_USBINTR, 0);
	}
	
 	if (sc->sc_irq_res && sc->sc_intr_hdl) {
		/*
		 * only call ehci_detach() after ehci_init()
		 */
		ehci_detach(sc);
		
		err = bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_intr_hdl);
		
		if (err)
		/* XXX or should we panic? */
			device_printf(dev, "Could not tear down irq, %d\n",
						  err);
		sc->sc_intr_hdl = NULL;
	}
	
 	if (sc->sc_irq_res) {
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_irq_res);
		sc->sc_irq_res = NULL;
	}
	if (sc->sc_io_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0,
							 sc->sc_io_res);
		sc->sc_io_res = NULL;
	}
	usb_bus_mem_free_all(&sc->sc_bus, &ehci_iterate_hw_softc);
	
	ehci_omap3_fini(isc);
	
	return (0);
}

static device_method_t ehci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, ehci_omap3_probe),
	DEVMETHOD(device_attach, ehci_omap3_attach),
	DEVMETHOD(device_detach, ehci_omap3_detach),
	DEVMETHOD(device_suspend, ehci_omap3_suspend),
	DEVMETHOD(device_resume, ehci_omap3_resume),
	DEVMETHOD(device_shutdown, ehci_omap3_shutdown),
	
	/* Bus interface */
	DEVMETHOD(bus_print_child, bus_generic_print_child),
	
	{0, 0}
};

static driver_t ehci_driver = {
	"ehci",
	ehci_methods,
	sizeof(struct omap3_ehci_softc),
};

static devclass_t ehci_devclass;

DRIVER_MODULE(ehci, omap3, ehci_driver, ehci_devclass, 0, 0);
MODULE_DEPEND(ehci, usb, 1, 1, 1);

MODULE_DEPEND(ehci, omap3_prcm, 1, 1, 1);
MODULE_DEPEND(ehci, omap3_scm, 1, 1, 1);
MODULE_DEPEND(ehci, omap3_gpio, 1, 1, 1);
