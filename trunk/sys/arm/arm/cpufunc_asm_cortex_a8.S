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
 * This driver is heavily based on the ARM V5 CPU functions (located here
 * cpufunc_asm_amrv5.S). Those assembly functions have the following copyright.
 *
 *
 * Copyright (c) 2002, 2005 ARM Limited
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
 * 3. The name of the company may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * ARMv7 Cortex assembly functions for CPU / MMU / TLB specific operations
 *
 * XXX We make no attempt at present to take advantage of the v7 memory
 * architecture or ASID tagged TLBs. However this is still a work in progress
 * so hopefully soon I'll get around to optimizing them.
 *
 */



#include <machine/asm.h>
__FBSDID("$FreeBSD$");

/*
 * Functions to set the MMU Translation Table Base register
 *
 * We need to clean and flush the cache as it uses virtual
 * addresses that are about to change.
 */
ENTRY(cortexa8_setttb)
	stmfd	sp!, {r0, lr}
	bl	_C_LABEL(cortexa8_idcache_wbinv_all)
	ldmfd	sp!, {r0, lr}

	mcr	p15, 0, r0, c2, c0, 0	/* load new TTB */

	mcr	p15, 0, r0, c8, c7, 0	/* invalidate I+D TLBs */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer - data synchronization barrier */
	mov	pc, lr
	RET

/*
 * TLB functions
 */
ENTRY(cortexa8_tlb_flushID_SE)
	mcr	p15, 0, r0, c8, c6, 1	/* flush D tlb single entry */
	mcr	p15, 0, r0, c8, c5, 1	/* flush I tlb single entry */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer  - data synchronization barrier */
	RET

ENTRY(cortexa8_tlb_flushI_SE)
	mcr	p15, 0, r0, c8, c5, 1	/* flush I tlb single entry */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer -  - data synchronization barrier*/
	RET
	
ENTRY(cortexa8_tlb_flushD_SE)
	mcr	p15, 0, r0, c8, c6, 1	/* flush D tlb single entry */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer  - data synchronization barrier */
	RET



/*
 * Context switch.
 *
 * These is the CPU-specific parts of the context switcher cpu_switch()
 * These functions actually perform the TTB reload.
 *
 * NOTE: Special calling convention
 *	r1, r4-r13 must be preserved
 */
ENTRY(cortexa8_context_switch)
	/*
	 * We can assume that the caches will only contain kernel addresses
	 * at this point.  So no need to flush them again.
	 */
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer - aka "Data Synchronization Barrier" (DSB) */
	mcr	p15, 0, r0, c2, c0, 0	/* set the new TTB */
	mcr	p15, 0, r0, c8, c7, 0	/* and flush the I+D tlbs */

	/* Paranoia -- make sure the pipeline is empty. */
	nop
	nop
	nop
	RET


/*
 * Flush Branch Prediction Array
 *
 * 
 */
ENTRY(cortexa8_flush_brnchtgt_E)
	mcr	p15, 0, r0, c7, c5, 7	/* invalidate VA from branch predictor array */
	mov	pc, lr

ENTRY(cortexa8_flush_brnchtgt_C)
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 6	/* invalidate entire branch predictor array */
	mov	pc, lr
	

/*
 * TLB functions
 */
ENTRY(cortexa8_tlb_flushID)
	mcr	p15, 0, r0, c8, c7, 0	/* flush I+D tlb */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer - data synchronization barrier */
	mov	pc, lr

ENTRY(cortexa8_tlb_flushI)
	mcr	p15, 0, r0, c8, c5, 0	/* flush I tlb */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer - data synchronization barrier */
	mov	pc, lr

ENTRY(cortexa8_tlb_flushD)
	mcr	p15, 0, r0, c8, c6, 0	/* flush D tlb */
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer - data synchronization barrier */
	mov	pc, lr

/*
 * Other functions
 */
ENTRY(cortexa8_drain_writebuf)
	mcr	p15, 0, r0, c7, c10, 4	/* drain write buffer - aka "Data Synchronization Barrier" (DSB) */
	mov	pc, lr					/* see page 3-97 of the Cortex-A8 reference manual */ 

ENTRY(cortexa8_get_dcache_linesize)
	mov	r0, #0x0				
	mcr p15, 2, r0, c0, c0, 0	/* set the cache selection register to 0x0 (L1 dcache) */
	mrc p15, 1, r0, c0, c0, 0	/* read the cache size identification register */
	and r0, r0, #0x7
	add	r0, r0, #2
	mov ip, #1
	mov r0, ip, LSL r0
	RET



#if 0
ENTRY(cortexa8_get_cache_info)
	mov	ip, #0x0
	mcr p15, 2, ip, c0, c0, 0	/* set the cache selection register to 0x0 (L1 dcache) */
	mrc p15, 1, ip, c0, c0, 0	/* read the cache size identification register */
	mov	s_max, ip, LSL#4		/* get the number of sets [bits 27:13] */
	mov s_max, s_max, LSR#17
	mov	s_max, s_max, LSL#6		/* set is in bits [11:6] */
	mov	s_inc, #(1<<6)			
	mov	w_max, ip, LSL#19		/* get the number of ways [bits 12:3] */
	mov w_max, w_max, LSR#22
	mov	w_max, w_max, LSL#30	/* way is in bits [31:30] */
	mov	w_inc, #(1<<30)	
	ldr	ip, .Lcortexa8_dcache_data
	stmia	ip, {s_max, i_max, s_inc, i_inc}

	mov	ip, #0x1
	mcr p15, 2, ip, c0, c0, 0	/* set the cache selection register to 0x0 (L1 dcache) */
	mrc p15, 1, ip, c0, c0, 0	/* read the cache size identification register */
	mov	s_max, ip, LSL#4		/* get the number of sets [bits 27:13] */
	mov s_max, s_max, LSR#17
	mov	s_max, s_max, LSL#6		/* set is in bits [11:6] */
	mov	s_inc, #(1<<6)			
	mov	w_max, ip, LSL#19		/* get the number of ways [bits 12:3] */
	mov w_max, w_max, LSR#22
	mov	w_max, w_max, LSL#30	/* way is in bits [31:30] */
	mov	w_inc, #(1<<30)	
	ldr	ip, .Lcortexa8_icache_data
	stmia	ip, {s_max, i_max, s_inc, i_inc}

	RET
#endif




/**
 * @macro: get_cache_sets_way
 * @desc:  Gets the number of cache 'sets' and 'ways'
 * @uses:  ip
 */
.macro cortexa8_get_cache_sets_way cache s_max s_inc w_max w_inc
	mov	ip, #\cache
	mcr p15, 2, ip, c0, c0, 0	/* set the cache selection register to 0x0 (L1 dcache) */
	mrc p15, 1, ip, c0, c0, 0	/* read the cache size identification register */

	mov	\s_max, ip, LSL#4		/* get the number of sets [bits 27:13] */
	mov \s_max, \s_max, LSR#17
	mov	\s_max, \s_max, LSL#6		/* set is in bits [11:6] */
	mov	\s_inc, #(1<<6)			

	mov	\w_max, ip, LSL#19		/* get the number of ways [bits 12:3] */
	mov \w_max, \w_max, LSR#22
	mov	\w_max, \w_max, LSL#30	/* way is in bits [31:30] */
	mov	\w_inc, #(1<<30)		
.endm


/*
 * Cache operations.  For the entire cache we use the set/index
 * operations.
 */
	s_max	.req r0
	w_max	.req r1
	s_inc	.req r2
	w_inc	.req r3

.Lcortexa8_dcache_line_size:
	.word	_C_LABEL(arm_pdcache_line_size)
.Lcortexa8_icache_line_size:
	.word	_C_LABEL(arm_picache_line_size)


/**
 * cortexa8_dcache_wb_range(vm_offset_t, vm_size_t)
 *
 * r0 = virtual_addr
 * r1 = size
 *
 */
ENTRY(cortexa8_dcache_wb_range)
	ldr	ip, .Lcortexa8_dcache_line_size
	cmp	r1, #0x4000
	bcs	.Lcortexa8_dcache_wb
	ldr	ip, [ip]
	sub	r1, r1, #1		/* Don't overrun */
	sub	r3, ip, #1
	and	r2, r0, r3
	add	r1, r1, r2
	bic	r0, r0, r3
.Lcortexa8_wb_next:
	mcr	p15, 0, r0, c7, c10, 1	/* Clean D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bpl	.Lcortexa8_wb_next
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr


/**
 * cortexa8_icache_sync_range(vm_offset_t, vm_size_t)
 *
 * r0 = virtual_addr
 * r1 = size
 *
 */
ENTRY_NP(cortexa8_icache_sync_range)
	ldr	ip, .Lcortexa8_icache_line_size
	cmp	r1, #0x4000
	bcs	.Lcortexa8_icache_sync_all
	ldr	ip, [ip]
	sub	r1, r1, #1		/* Don't overrun */
	sub	r3, ip, #1
	and	r2, r0, r3
	add	r1, r1, r2
	bic	r0, r0, r3
.Lcortexa8_sync_next:
	mcr	p15, 0, r0, c7, c5, 1	/* Invalidate I cache SE with VA */
	mcr	p15, 0, r0, c7, c10, 1	/* Clean D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bpl	.Lcortexa8_sync_next
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr


ENTRY_NP(cortexa8_icache_sync_all)
.Lcortexa8_icache_sync_all:
	/*
	 * We assume that the code here can never be out of sync with the
	 * dcache, so that we can safely flush the Icache and fall through
	 * into the Dcache cleaning code.
	 */
	mcr	p15, 0, r0, c7, c5, 0	/* Flush I cache */
	/* Fall through to clean Dcache. */

.Lcortexa8_dcache_wb:
	cortexa8_get_cache_sets_way 0x0 s_max s_inc w_max w_inc
.Lnext_set:
	orr	ip, s_max, w_max
.Lnext_way:
	mcr	p15, 0, ip, c7, c10, 2	/* Clean D cache SE with Set/Index */
	sub	ip, ip, w_inc
	tst	ip, w_max		/* Index 0 is last one */
	bne	.Lnext_way		/* Next index */
	mcr	p15, 0, ip, c7, c10, 2	/* Clean D cache SE with Set/Index */
	subs	s_max, s_max, s_inc
	bpl	.Lnext_set		/* Next set */
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr


/**
 * cortexa8_dcache_wbinv_range(vm_offset_t, vm_size_t)
 *
 * r0 = virtual_addr
 * r1 = size
 *
 */
ENTRY(cortexa8_dcache_wbinv_range)
	ldr	ip, .Lcortexa8_dcache_line_size
	cmp	r1, #0x4000
	bcs	.Lcortexa8_dcache_wbinv_all
	ldr	ip, [ip]
	sub	r1, r1, #1		/* Don't overrun */
	sub	r3, ip, #1
	and	r2, r0, r3
	add	r1, r1, r2
	bic	r0, r0, r3
.Lcortexa8_wbinv_next:
	mcr	p15, 0, r0, c7, c14, 1	/* Purge D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bpl	.Lcortexa8_wbinv_next
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr

/*
 * Note, we must not invalidate everything.  If the range is too big we
 * must use wb-inv of the entire cache.
 *
 * r0 = virtual_addr
 * r1 = size
 */
ENTRY(cortexa8_dcache_inv_range)
	ldr	ip, .Lcortexa8_dcache_line_size
	cmp	r1, #0x4000
	bcs	.Lcortexa8_dcache_wbinv_all
	ldr	ip, [ip]				/* ip = cache_line_size                       */
	sub	r1, r1, #1				/* size -= 1 : Don't overrun                  */
	sub	r3, ip, #1				/* r3 = cache_bitmask = (cache_line_size - 1) */
	and	r2, r0, r3				/* r2 = vaddr & cache_bitmask                 */
	add	r1, r1, r2				/* r1 = size = size + (vaddr & cache_bitmask) */
	bic	r0, r0, r3				/* r0 = vaddr = vaddr & ~cache_bitmask        */
.Lcortexa8_inv_next:
	mcr	p15, 0, r0, c7, c6, 1	/* Invalidate D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bpl	.Lcortexa8_inv_next
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr


/**
 * cortexa8_idcache_wbinv_range(vm_offset_t, vm_size_t)
 *
 * r0 = offset
 * r1 = size
 *
 */
ENTRY(cortexa8_idcache_wbinv_range)
	ldr	ip, .Lcortexa8_dcache_line_size
	cmp	r1, #0x4000
	bcs	.Lcortexa8_idcache_wbinv_all
	ldr	ip, [ip]
	sub	r1, r1, #1		/* Don't overrun */
	sub	r3, ip, #1
	and	r2, r0, r3
	add	r1, r1, r2
	bic	r0, r0, r3
.Lcortexa8_id_wbinv_next:
	mcr	p15, 0, r0, c7, c5, 1	/* Invalidate I cache SE with VA */
	mcr	p15, 0, r0, c7, c14, 1	/* Purge D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bpl	.Lcortexa8_id_wbinv_next
	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr



/*
 * 
 *
 * There are some restrictions on the registers that this function can use
 * (unless of course a stack is used).
 *
 * Can't use: r5, r6, r7, r8, r9, r10
 */
ENTRY_NP(cortexa8_idcache_wbinv_all)
.Lcortexa8_idcache_wbinv_all:
	/*
	 * We assume that the code here can never be out of sync with the
	 * dcache, so that we can safely flush the Icache and fall through
	 * into the Dcache purging code.
	 */
	mcr	p15, 0, r0, c7, c5, 0	/* Flush I cache */
	/* Fall through to purge Dcache. */

ENTRY(cortexa8_dcache_wbinv_all)
.Lcortexa8_dcache_wbinv_all:
	cortexa8_get_cache_sets_way 0x0 s_max s_inc w_max w_inc
	
.Lnext_set_inv:
	orr	ip, s_max, w_max
.Lnext_way_inv:
	mcr	p15, 0, ip, c7, c14, 2	/* Purge D cache SE with Set/Index */
	sub	ip, ip, w_inc
	tst	ip, w_max				/* Index 0 is last one */
	bne	.Lnext_way_inv			/* Next way */
	mcr	p15, 0, ip, c7, c14, 2	/* Purge D cache SE with Set/Index */
	subs	s_max, s_max, s_inc
	bpl	.Lnext_set_inv		/* Next set */

	mcr	p15, 0, r0, c7, c10, 4	/* drain the write buffer */
	bx	lr



