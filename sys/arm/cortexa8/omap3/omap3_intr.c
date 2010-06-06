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
#include <machine/intr.h>
#include <vm/vm.h>
#include <vm/pmap.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>



/*
 * There are a number of ways that interrupt handling is implemented in
 * the various ARM platforms, the PXA has the neatest way, it creates another
 * device driver that handles everything. However IMO this is rather heavy-
 * weight for playing with IRQs which should be quite fast ... so I've
 * gone for something similar to the IXP425, which just directly plays with
 * registers. This assumes that the interrupt control registers are already
 * mapped in virtual memory at a fixed virtual address ... simplies.
 * 
 * The intcps (OMAP3 interrupt controller) has some nice registers, were
 * you write a bit to clear or set the mask register ... I think in theory
 * that means that you don't need to disable interrupts while doing this,
 * because it is an atomic operation.
 *
 * TODO: check this.
 *
 */


/**
 *	omap3_post_filter_intr - called after the IRQ has been filtered
 *	@arg: the IRQ number
 *
 *	Called after the interrupt handler has done it's stuff, can be used to
 * clean up interrupts that haven't been handled properly.
 *
 *
 *	RETURNS:
 *	nothing
 */
void
omap3_post_filter_intr(void *arg)
{
	/* uintptr_t irq = (uintptr_t) arg; */
	
	/* data synchronization barrier */
	cpu_drain_writebuf();
}


/**
 *	omap3_setup_intr - initialises and unmasks the IRQ.
 *	@dev: 
 *	@child: 
 *	@res: 
 *	@flags: 
 *	@filt: 
 *	@intr: 
 *	@arg: 
 *	@cookiep: 
 *
 *	
 *
 *
 *	RETURNS:
 *	0 on success
 */
int
omap3_setup_intr(device_t dev, device_t child,
				  struct resource *res, int flags, driver_filter_t *filt, 
				  driver_intr_t *intr, void *arg, void **cookiep)    
{
	unsigned int i;

	/* Macro .. does something ? */
	BUS_SETUP_INTR(device_get_parent(dev), child, res, flags, filt, intr,
				   arg, cookiep);

	/* Enable all the interrupts in the range ... will probably be only one */
	for (i = rman_get_start(res); (i < 96) && (i <= rman_get_end(res)); i++)
	{
		arm_unmask_irq(i);
	}
	
	return (0);
}

/**
 *	omap3_teardown_intr - 
 *	@dev: 
 *	@child: 
 *	@res: 
 *	@cookie: 
 *
 *	
 *
 *
 *	RETURNS:
 *	0 on success
 */
int
omap3_teardown_intr(device_t dev, device_t child, struct resource *res,
					 void *cookie)
{
	unsigned int i;

	/* Mask (disable) all the interrupts in the range ... will probably be only one */
	for (i = rman_get_start(res); (i < 96) && (i <= rman_get_end(res)); i++)
	{
		arm_mask_irq(i);
	}
	
	return (BUS_TEARDOWN_INTR(device_get_parent(dev), child, res, cookie));
}



/**
 *	arm_mask_irq - masks an IRQ (disables it)
 *	@nb: the number of the IRQ to mask (disable)
 *
 *	Disables the interrupt at the HW level.
 *
 *
 *	RETURNS:
 *	nothing
 */
void
arm_mask_irq(uintptr_t nb)
{
	*((volatile u_int32_t*)OMAP35XX_INTCPS_MIR_SET(nb >> 5))
		= 1 << (nb & 0x1F);
}


/**
 *	arm_unmask_irq - unmasks an IRQ (enables it)
 *	@nb: the number of the IRQ to unmask (enable)
 *
 *	Enables the interrupt at the HW level.
 *
 *
 *	RETURNS:
 *	nothing
 */
void
arm_unmask_irq(uintptr_t nb)
{
	*((volatile u_int32_t*)OMAP35XX_INTCPS_MIR_CLEAR(nb >> 5))
		= 1 << (nb & 0x1F);
}



/**
 *	arm_get_next_irq - gets the next tripped interrupt
 *	@last_irq: the number of the last IRQ processed
 *
 *	Enables the interrupt at the HW level.
 *
 *
 *	RETURNS:
 *	nothing
 */
int
arm_get_next_irq(int last_irq)
{
	volatile u_int32_t* preg;
	uint32_t active_irq;
	
	/* clean-up the last IRQ */
	if (last_irq != -1) {
		
		/* clear the interrupt status flag */
		preg = (volatile u_int32_t*) OMAP35XX_INTCPS_ISR_CLEAR(last_irq >> 5);
		*preg = (0x1UL << (last_irq & 0x1F));
	
		/* tell the interrupt logic we've dealt with the interrupt */
		preg = (volatile u_int32_t*) OMAP35XX_INTCPS_CONTROL;
		*preg = 0x1UL;
	}
	
	/* Get the next active interrupt */
	preg = (volatile u_int32_t*) OMAP35XX_INTCPS_SIR_IRQ;
	active_irq = *preg;
	
	/* Check for spurious interrupt */
	if ((active_irq & 0xffffff80) == 0xffffff80) {
		printf("[BRG] Spurious interrupt detected [0x%08x]\n", active_irq);
		return -1;
	}

	/* Just get the active IRQ part */
	active_irq &= 0x7F;
	
	/* Return the new IRQ if it is different from the previous */
	if (active_irq != last_irq)
		return active_irq;
	else
		return -1;
}
