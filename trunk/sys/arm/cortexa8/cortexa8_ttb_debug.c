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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysproto.h>
#include <sys/signalvar.h>
#include <sys/imgact.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/linker.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/ptrace.h>
#include <sys/cons.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/buf.h>
#include <sys/exec.h>
#include <sys/kdb.h>
#include <sys/msgbuf.h>
#include <machine/reg.h>
#include <machine/cpu.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>
#include <vm/vm_map.h>
#include <vm/vnode_pager.h>
#include <machine/pmap.h>
#include <machine/vmparam.h>
#include <machine/pcb.h>
#include <machine/undefined.h>
#include <machine/machdep.h>
#include <machine/metadata.h>
#include <machine/armreg.h>
#include <machine/bus.h>
#include <sys/reboot.h>

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>


extern void beagle_putchar(unsigned char);
extern void beagle_putstr(unsigned char *);
extern void beagle_printf(const char *fmt, ...);

void beagle_dump_ttb(vm_offset_t va);

#define PHSY2VIRT(a)	(((a) - PHYSADDR) + KERNBASE)




/**
 *	beagle_dump_l2_entry - dumps a single L2 entry
 *	@va: the virtual address of top level PTE.
 *	@pte: the actual PTE entry.
 *
 *
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	nothing
 */
static void
beagle_dump_l2_entry(vm_offset_t va, uint32_t pte)
{
	switch (pte & 0x3) {
		case 0x1:
			beagle_printf("[0x%08x]\t -> L2 L [0x%08x] {0x%08x} XN:%d TEX:%d nG:%d S:%d AP:%d SBZ:%d AP:%d C:%d B:%d\n",
						  va,
						  pte,
						  (pte & 0xFFFF0000),
						  ((pte >> 15) & 0x1),
						  ((pte >> 12) & 0x7),
						  ((pte >> 11) & 0x1),
						  ((pte >> 10) & 0x1),
						  ((pte >> 9) & 0x1),
						  ((pte >> 6) & 0x7),
						  ((pte >> 4) & 0x3),
						  ((pte >> 3) & 0x1),
						  ((pte >> 2) & 0x1));
			break;
		case 0x2:
		case 0x3:
			beagle_printf("[0x%08x]\t -> L2 S [0x%08x] {0x%08x} nG:%d S:%d AP:%d TEX:%d AP:%d C:%d B:%d XN:%d\n",
						  va,
						  pte,
						  (pte & 0xFFFFF000),
						  ((pte >> 11) & 0x1),
						  ((pte >> 10) & 0x1),
						  ((pte >> 9) & 0x1),
						  ((pte >> 6) & 0x7),
						  ((pte >> 4) & 0x3),
						  ((pte >> 3) & 0x1),
						  ((pte >> 2) & 0x1),
						  ((pte >> 0) & 0x1));
			break;
	}
}



/**
 *	beagle_dump_l2_entries - dumps L2 entries
 *	@va: the virtual address of top level PTE.
 *	@off: the offset from the L1 level.
 *
 *	
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	nothing
 */
static void
beagle_dump_l2_entries(vm_offset_t va, vm_offset_t off)
{
	uint32_t i;
	volatile uint32_t *p_pte = (volatile uint32_t*) va;
	
	beagle_printf("\tDumping L2 @ 0x%08x\n", va);
	
	for (i=0; i<256; i++) {
		beagle_dump_l2_entry((off + (i * 4096)), *p_pte++);
	}
}


/**
 *	beagle_dump_l1_entry - dumps an L1 entry
 *	@va: the virtual address of top level PTE.
 *	@pte: the page table entry.
 *
 *	
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	nothing
 */
static void
beagle_dump_l1_entry(vm_offset_t va, uint32_t pte)
{
	switch (pte & 0x3) {
		case 0x1:
			beagle_printf("[0x%08x] -> L1 P [0x%08x] {0x%08x} IMP:%d D:%d SBZ:%d NS:%d SBZ:%d\n",
						  va,
						  pte,
						  (pte & 0xFFFFFC00),
						  ((pte >> 9) & 0x1),
						  ((pte >> 5) & 0xF),
						  ((pte >> 4) & 0x1),
						  ((pte >> 3) & 0x1),
						  ((pte >> 2) & 0x1));
			beagle_dump_l2_entries(PHSY2VIRT(pte & 0xFFFFFC00), va);
			break;
		case 0x2:
			if (((pte >> 18) & 0x1) == 0) {
				beagle_printf("[0x%08x] -> L1 S [0x%08x] {0x%08x} NS:%d 0:%d nG:%d S:%d AP2:%d TEX:%d AP:%d IMP:%d D:%d XN:%d C:%d B:%d\n",
							  va,
							  pte,
							  (pte & 0xFFF00000),
							  ((pte >> 19) & 0x1),
							  ((pte >> 18) & 0x1),
							  ((pte >> 17) & 0x1),
							  ((pte >> 16) & 0x1),
							  ((pte >> 15) & 0x1),
							  ((pte >> 12) & 0x7),
							  ((pte >> 10) & 0x3),
							  ((pte >> 9) & 0x1),
							  ((pte >> 5) & 0xF),
							  ((pte >> 4) & 0x1),
							  ((pte >> 3) & 0x1),
							  ((pte >> 2) & 0x1));
			} else {
				beagle_printf("[0x%08x] -> L1 Ss[0x%08x] {0x%08x} EXT:%d NS:%d 1:%d nG:%d S:%d AP2:%d TEX:%d AP:%d IMP:%d EXT:%d XN:%d C:%d B:%d\n",
							  va,
							  pte,
							  (pte & 0xFF000000),
							  ((pte >> 20) & 0xF),
							  ((pte >> 19) & 0x1),
							  ((pte >> 18) & 0x1),
							  ((pte >> 17) & 0x1),
							  ((pte >> 16) & 0x1),
							  ((pte >> 15) & 0x1),
							  ((pte >> 12) & 0x7),
							  ((pte >> 10) & 0x3),
							  ((pte >> 9) & 0x1),
							  ((pte >> 5) & 0xF),
							  ((pte >> 4) & 0x1),
							  ((pte >> 3) & 0x1),
							  ((pte >> 2) & 0x1));
			}
			break;
		case 0x3:
			beagle_printf("L1 Reservered\n");
			break;
	}
}

/**
 *	beagle_dump_ttb - dump the MMU L1/L2 page tables
 *	@va: the virtual address of top level PTE.
 *
 *	Prints the complete MMU page table out the uart. 
 *
 *	LOCKING:
 *	None required
 *
 *	RETURNS:
 *	nothing
 */
void
beagle_dump_ttb(vm_offset_t va)
{
	uint32_t i;
	volatile uint32_t *p_pte = (volatile uint32_t*) va;

	beagle_printf("Dumping TTB @ 0x%08x\n", va);
	
	for (i=0; i<4096; i++) {
		beagle_dump_l1_entry((i * 1024 * 1024), *p_pte++);
	}
}


