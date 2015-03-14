# Introduction #

This page details the current status of the various SOC components and their drivers on the Omap3530.

I'm attempting to keep this as up to date as possible.


# Components #

  * Power, Reset and Clock Management (PRCM)
  * System Control Module (SCM)
  * DMA Module
  * Interrupt Controller (INTCs)
  * Timers
  * UART/IrDA/CIR
  * I2C
  * Multichannel Buffered Serial Port (McBSP)
  * MMC/SD/SDIO Card Interface
  * High-Speed USB Host Subsystem (HSUSB)
  * General-Purpose Interface (GPIO)




---

### Power, Reset and Clock Management (PRCM) ###

**Status:**

The driver currently does the basics of enabling/disabling the functional and interface clocks of all modules.  It also enables DPLL5 used for the HS USB controller.

There are no know issues with this driver, however it is fairly basic and I've started work on writing a new more powerful version.


**Driver file(s):**
  * `omap3_prcm.c`
  * `omap3_prcm.h`




---

### System Control Module (SCM) ###

**Status:**

Currently this module only performs the pad configuration (pin muxing).  There are no know issues but it is also very basic and more support for the other features of the SCM needs to be added.


**Driver file(s):**
  * `omap3_scm.c`
  * `omap3_scm.h`



---

### DMA Module ###

**Status:**

This driver was written to add support for the MMC/SD card driver, it is currently only been tested with that, it is likely there are bugs in this driver.

Support for packet and normal DMA operations are supported and there are functions that cover a number of the features supported by the module.  In this sense the driver is fairly complete, however not very many of the different DMA features have been tested.

**Driver file(s):**
  * `omap3_dma.c`
  * `omap3_dma.h`





---

### Interrupt Controller ###

**Status:**

Interrupt handling is mostly supported by the kernel, but each platform has to support masking/unmaking IRQ's and getting the next active IRQ.  Functions to do this are all included in this file.

**Driver file(s):**
  * `omap3_intr.c`





---

### Timers ###

**Status:**

The basics of setting up the system timers are present, however the DELAY() function still needs some work.

**Driver file(s):**
  * `omap3_timer.c`






---

### UART/IrDA/CIR ###

**Status:**

The OMAP3530 UART is compatible with 8250 devices, as such the _driver_ simply passes the base address to the ns8250 driver.  This is the first driver I wrote for the BragleBoard so it needs a little bit of a tidy up, but it is working in it's current state.

**Driver file(s):**
  * `uart_bus_omap3.c`
  * `uart_cpu_omap3.c`





---

### I2C ###

**Status:**

I believe this driver is working, however I've only tried it talking to the TPS65950 chip on the BeagleBoard.  Currently the driver doesn't support DMA I2C transactions, I hope to add that sometime in the future.

**Driver file(s):**
  * `omap3_i2c.c`
  * `omap3_i2c.h`




---

### Multichannel Buffered Serial Port (McBSP) ###

**Status:**

I've started work on this driver, but it is no where near a working state.

**Driver file(s):**
  * `omap3_mcbsp.c`
  * `omap3_mcbsp.h`




---

### MMC/SD/SDIO Card Interface ###

**Status:**

This driver is working, however I've only tested it with an SD card so quite possibly the other modes may not work at all.  I've also come across a few issues which I haven't been able to characteristic or reproduce, so again use this driver with caution.

**Driver file(s):**
  * `omap3_mmc.c`
  * `omap3_mmc.h`




---

### High-Speed USB Host Subsystem (HSUSB) ###

**Status:**

The EHCI driver has been written and I've tested it on a few different USB peripherals and so far so good.  But as with all these drivers it needs a lot more testing.

**Driver file(s):**
  * `ehci_omap3.c`




---

### General-Purpose Interface (GPIO) ###

**Status:**

The GPIO driver performs has no know issues, it can set/get the pin status as well as setup de-bouncing and interrupt support.

**Driver file(s):**
  * `omap3_gpio.c`
  * `omap3_gpio.h`
