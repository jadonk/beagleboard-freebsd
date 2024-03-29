/*	$NetBSD: bus_space_asm_generic.S,v 1.3 2003/03/27 19:46:14 mycroft Exp $	*/

/*-
 * Copyright (c) 1997 Causality Limited.
 * Copyright (c) 1997 Mark Brinicombe.
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
 *	This product includes software developed by Mark Brinicombe
 *	for the NetBSD Project.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
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

#include <machine/asm.h>
#include <machine/cpuconf.h>
__FBSDID("$FreeBSD$");

/*
 * Generic bus_space functions.
 */

/*
 * read single
 */

ENTRY(generic_armv7_bs_r_2)
	ldrh	r0, [r1, r2]
	RET

/*
 * write single
 */

ENTRY(generic_armv7_bs_w_2)
	strh	r3, [r1, r2]
	RET

/*
 * read multiple
 */

ENTRY(generic_armv7_bs_rm_2)
	add	r0, r1, r2
	mov	r1, r3
	ldr	r2, [sp, #0]
	teq	r2, #0
	RETeq

1:	ldrh	r3, [r0]
	strh	r3, [r1], #2
	subs	r2, r2, #1
	bne	1b

	RET

/*
 * write multiple
 */

ENTRY(generic_armv7_bs_wm_2)
	add	r0, r1, r2
	mov	r1, r3
	ldr	r2, [sp, #0]
	teq	r2, #0
	RETeq

1:	ldrh	r3, [r1], #2
	strh	r3, [r0]
	subs	r2, r2, #1
	bne	1b

	RET

/*
 * read region
 */

ENTRY(generic_armv7_bs_rr_2)
	add	r0, r1, r2
	mov	r1, r3
	ldr	r2, [sp, #0]
	teq	r2, #0
	RETeq

1:	ldrh	r3, [r0], #2
	strh	r3, [r1], #2
	subs	r2, r2, #1
	bne	1b

	RET

/*
 * write region.
 */

ENTRY(generic_armv7_bs_wr_2)
	add	r0, r1, r2
	mov	r1, r3
	ldr	r2, [sp, #0]
	teq	r2, #0
	RETeq

1:	ldrh	r3, [r1], #2
	strh	r3, [r0], #2
	subs	r2, r2, #1
	bne	1b

	RET

/*
 * set region
 */

ENTRY(generic_armv7_bs_sr_2)
	add	r0, r1, r2
	mov	r1, r3
	ldr	r2, [sp, #0]
	teq	r2, #0
	RETeq

1:	strh	r1, [r0], #2
	subs	r2, r2, #1
	bne	1b

	RET

/*
 * copy region
 */

ENTRY(generic_armv7_bs_c_2)
	add	r0, r1, r2
	ldr	r2, [sp, #0]
	add	r1, r2, r3
	ldr	r2, [sp, #4]
	teq	r2, #0
	RETeq

	cmp	r0, r1
	blt	2f

1:	ldrh	r3, [r0], #2
	strh	r3, [r1], #2
	subs	r2, r2, #1
	bne	1b

	RET

2:	add	r0, r0, r2, lsl #1
	add	r1, r1, r2, lsl #1
	sub	r0, r0, #2
	sub	r1, r1, #2

3:	ldrh	r3, [r0], #-2
	strh	r3, [r1], #-2
	subs	r2, r2, #1
	bne	3b

	RET
