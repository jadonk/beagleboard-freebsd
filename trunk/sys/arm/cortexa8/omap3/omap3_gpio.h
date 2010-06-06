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
#ifndef _OMAP3_GPIO_H_
#define _OMAP3_GPIO_H_


int
omap3_gpio_request(int pin, const char *name);

int
omap3_gpio_free(int pin);

int
omap3_gpio_direction_output(int pin, int val);

int
omap3_gpio_direction_input(int pin);

int
omap3_gpio_pin_get(int pin);

int
omap3_gpio_pin_set(int pin, int val);

int
omap3_gpio_pin_toggle(int pin);


#endif /* _OMAP3_GPIO_H_ */
