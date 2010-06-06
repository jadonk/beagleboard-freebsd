/*-
 * Copyright (c) 2009 Guillaume Ballet
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#define _ARM32_BUS_DMA_PRIVATE

#include <sys/types.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <sys/pcpu.h>
#include <sys/cons.h>
#include <sys/conf.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <machine/pmap.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <machine/pcb.h>
#include <machine/machdep.h>
#include <machine/undefined.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <sys/kdb.h>
#define DEBUG_INITARM
#define VERBOSE_INIT_ARM

#include <arm/cortexa8/omap3/omap3reg.h>
#include <arm/cortexa8/omap3/omap3var.h>



#ifdef VERBOSE_INIT_ARM
// #define  DPRINTF(x)		beagle_printf(x)
#define  DPRINTF(x)
#else
#define  DPRINTF(x)
#endif


struct pcpu	__pcpu;
struct pcpu	*pcpup = &__pcpu;

/* Physical page ranges */
vm_paddr_t	phys_avail[4];
vm_paddr_t	dump_avail[4];

struct pv_addr systempage;

#define FIQ_STACK_SIZE	1
#define IRQ_STACK_SIZE	1
#define ABT_STACK_SIZE	1
#define UND_STACK_SIZE	1


extern int _start[];
extern int _end[];


extern void beagle_putchar(unsigned char);
extern void beagle_putstr(unsigned char *);
extern void beagle_printf(const char *fmt, ...);

extern void beagle_dump_ttb(vm_offset_t va);

extern u_int data_abort_handler_address;
extern u_int prefetch_abort_handler_address;
extern u_int undefined_handler_address;

static struct pv_addr	fiqstack;				/* Stack page descriptors for all modes */
static struct pv_addr	irqstack;
static struct pv_addr	undstack;
static struct pv_addr	abtstack;
static struct pv_addr	kernelstack;
static struct pv_addr	kernel_l1pt;				/* Level-1 page table entry */




#define KERNEL_PT_SYS			0	/* Page table for mapping proc0 zero page */
#define	KERNEL_PT_IO			1
#define KERNEL_PT_IO_NUM		3
#define KERNEL_PT_BEFOREKERN	KERNEL_PT_IO + KERNEL_PT_IO_NUM
#define KERNEL_PT_AFKERNEL		KERNEL_PT_BEFOREKERN + 1	/* L2 table for mapping after kernel */
#define	KERNEL_PT_AFKERNEL_NUM	9

#define NUM_KERNEL_PTS	12

static struct pv_addr	kernel_page_tables[NUM_KERNEL_PTS];	/* Level-2 page table entries for the kernel */



static struct trapframe	proc0_tf;

#define PHYS2VIRT(x)	((x - KERNPHYSADDR) + KERNVIRTADDR)
#define VIRT2PHYS(x)	((x - KERNVIRTADDR) + KERNPHYSADDR)

/* Macro stolen from the Xscale part, used to simplify TLB allocation */
#define valloc_pages(var, np)                   \
        alloc_pages((var).pv_pa, (np));         \
        (var).pv_va = PHYS2VIRT((var).pv_pa);	\
        DPRINTF(("va=%p pa=%p\n", (void*)(var).pv_va, (void*)(var).pv_pa));

#define alloc_pages(var, np)                    \
        (var) = freemempos;             \
        freemempos += (np * PAGE_SIZE);         \
        memset((char *)(var), 0, ((np) * PAGE_SIZE));

#define round_L_page(x) (((x) + L2_L_OFFSET) & L2_L_FRAME)

#define VERBOSE_INIT_ARM

static const struct pmap_devmap omap3_devmap[] = {
	/*
	 * For the moment, map devices with PA==VA.
	 */
	{
		/* 16MB of L4, covering all L4 core registers */
		OMAP35XX_L4_CORE_VBASE,
		OMAP35XX_L4_CORE_HWBASE,
		OMAP35XX_L4_CORE_SIZE,
		VM_PROT_READ|VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		/* 1MB of L4, covering the periph registers */
		OMAP35XX_L4_PERIPH_VBASE,
		OMAP35XX_L4_PERIPH_HWBASE,
		OMAP35XX_L4_PERIPH_SIZE,
		VM_PROT_READ|VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		/* 64Kb of L3, covering the SDRAM controller registers */
		OMAP35XX_L4_WAKEUP_VBASE,
		OMAP35XX_L4_WAKEUP_HWBASE,
		OMAP35XX_L4_PERIPH_SIZE,
		VM_PROT_READ|VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{
		/* 64Kb of L3, covering the SDRAM controller registers */
		OMAP35XX_SDRC_VBASE,
		OMAP35XX_SDRC_HWBASE,
		OMAP35XX_SDRC_SIZE,
		VM_PROT_READ|VM_PROT_WRITE,
		PTE_NOCACHE
	},
	{ 0, 0, 0, 0, 0 }	/* Array terminator */
};


static void *
initarm_boilerplate(void *arg1, void *arg2)
{
	vm_offset_t	       freemempos;
	vm_offset_t        freemem_pt;
	vm_offset_t        lastaddr;
	vm_offset_t	       offset;
	uint32_t           i, j;
	volatile uint32_t  cpsr;
	uint32_t           sdram_size;
	u_int32_t          l1pagetable;
	size_t	           textsize;
	size_t	           totalsize;
	
	
	/*
	 * Sets the CPU functions based on the CPU ID, this code is all
	 * in the cpufunc.c file. The function sets the cpufunc structure
	 * to match the machine and also initialises the pmap/pte code.
	 */
	set_cpufuncs();
	
	
	/*
	 * fake_preload_metadata() creates a fake boot descriptor table and
	 * returns the last address in the kernel.
	 */
	lastaddr = fake_preload_metadata();
	
	
	/*
	 * Initialize the MI portions of a struct per cpu structure.
	 */
	pcpu_init(pcpup, 0, sizeof(struct pcpu));
	PCPU_SET(curthread, &thread0);
	
	
	
#ifdef VERBOSE_INIT_ARM
	__asm __volatile("mrc	p15, 0, %0, c1, c0, 0 ;" : "=r" (cpsr));
	beagle_printf("cp15:c1:c0=0x%08x\n", cpsr);
	beagle_printf("PMAP_DOMAIN_KERNEL=0x%08x\n", PMAP_DOMAIN_KERNEL);
#endif
	
	
	/*
	 * Initialize freemempos. Pages are allocated from the end of the RAM's
	 * first 64MB, as it is what is covered by the default TLB in locore.S.
	 */
	freemempos	= VIRT2PHYS(round_L_page(lastaddr));

	beagle_printf("freemempos=0x%08x %u %u\n", (unsigned long)freemempos, L1_S_SIZE, L1_TABLE_SIZE);
	
	/* Reserve L1 table pages now, as freemempos is 64K-aligned */
	valloc_pages(kernel_l1pt, L1_TABLE_SIZE / PAGE_SIZE);
	beagle_printf("l1pa=0x%08x l1va=0x%08x\n", (unsigned long)kernel_l1pt.pv_pa, (unsigned long)kernel_l1pt.pv_va);
	
	/* 
	 * Reserve the paging system pages, page #0 is reserved as a L2 table for
	 * the exception vector. Now allocate L2 page tables; they are linked to L1 below
	 */
	for (i = 0; i < NUM_KERNEL_PTS; ++i) {
		j = (i % (PAGE_SIZE / L2_TABLE_SIZE_REAL));
		
		if (j == 0) {
			valloc_pages(kernel_page_tables[i], L2_TABLE_SIZE / PAGE_SIZE);
		} else {
			
			kernel_page_tables[i].pv_pa	= kernel_page_tables[i - j].pv_pa 
			                              + (j * L2_TABLE_SIZE_REAL);
			kernel_page_tables[i].pv_va	= kernel_page_tables[i - j].pv_va
			                              + (j * L2_TABLE_SIZE_REAL);
		}
#ifdef VERBOSE_INIT_ARM
		beagle_printf("kernel_page_tables[%d]  PA:0x%08x VA:0x%08x\n", i, kernel_page_tables[i].pv_pa, kernel_page_tables[i].pv_va);
#endif
	}

	/* base of allocated pt's */
	freemem_pt = freemempos;

/*
	for (i=0; i<NUM_KERNEL_PTS; i++) {
		if ((i % (PAGE_SIZE / L2_TABLE_SIZE_REAL)) == 0) {
			valloc_pages(kernel_page_tables[i], L2_TABLE_SIZE / PAGE_SIZE);
		} else {
			j = i % (PAGE_SIZE / L2_TABLE_SIZE_REAL);
			kernel_page_tables[i].pv_pa	= kernel_page_tables[i - j].pv_pa 
			                              + (j * L2_TABLE_SIZE_REAL);
			kernel_page_tables[i].pv_va	= kernel_page_tables[i - j].pv_va
			                              + (j * L2_TABLE_SIZE_REAL);
		}
	}
*/
	
	/*
	 * Allocate a page for the system page mapped to V0x00000000. This page
	 * will just contain the system vectors and can be shared by all processes.
	 * This is where the interrupt vector is stored.
	 */
	valloc_pages(systempage, 1);
	systempage.pv_va = ARM_VECTORS_HIGH;
#ifdef VERBOSE_INIT_ARM
	beagle_printf("ARM_VECTORS_HIGH=0x%08x\n", (unsigned long)ARM_VECTORS_HIGH);
#endif
	
	/* Allocate dynamic per-cpu area. */
	/* TODO: valloc_pages(dpcpu, DPCPU_SIZE / PAGE_SIZE); */
	/* TODO: dpcpu_init((void *)dpcpu.pv_va, 0); */
	
	
	/* Allocate stacks for all modes */
	valloc_pages(fiqstack, FIQ_STACK_SIZE);
	valloc_pages(irqstack, IRQ_STACK_SIZE);
	valloc_pages(abtstack, ABT_STACK_SIZE);
	valloc_pages(undstack, UND_STACK_SIZE);
	valloc_pages(kernelstack, KSTACK_PAGES + 1);
	/* valloc_pages(msgbuf, round_page(MSGBUF_SIZE) / PAGE_SIZE); */
	
	 
	 
	/* ** Build the TLBs **************************************************** */
	
	/*
	 * Now construct the L1 page table.  First map the L2
	 * page tables into the L1 so we can replace L1 mappings
	 * later on if necessary
	 */
	l1pagetable = kernel_l1pt.pv_va;
	
	/* L2 table for the exception vector */
	pmap_link_l2pt(l1pagetable, ARM_VECTORS_HIGH & ~(0x100000 - 1),
	               &kernel_page_tables[0]);
	
	/* Insert a reference to the kernel L2 page tables into the L1 page. */
	for (i=1; i<NUM_KERNEL_PTS; i++) {
		pmap_link_l2pt(l1pagetable,
					   KERNVIRTADDR + (i-1) * L1_S_SIZE,
					   &kernel_page_tables[i]);
	}
	
	/*
	 * Map the kernel
	 */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Mapping the kernel pages\n");
#endif

	textsize = round_L_page((unsigned long)etext - KERNVIRTADDR);
	totalsize = round_L_page((unsigned long)_end - KERNVIRTADDR);
	
#ifdef VERBOSE_INIT_ARM
	beagle_printf(".text=0x%08x total=0x%08x _end=0x%08x 0x%08x 0x%08x\n",
	              textsize, totalsize, (unsigned int)_end,
				  (unsigned int) etext, KERNVIRTADDR);
#endif
	offset = 0;
	offset  += pmap_map_chunk(l1pagetable, KERNVIRTADDR + offset,
							 KERNPHYSADDR + offset, textsize,
							 VM_PROT_READ|VM_PROT_EXECUTE|VM_PROT_WRITE, PTE_CACHE);
	
	offset += pmap_map_chunk(l1pagetable, KERNVIRTADDR + offset,
							 KERNPHYSADDR + offset, totalsize - textsize,
							 VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	
	/* Map the L1 page table of the kernel */
	pmap_map_chunk(l1pagetable,
				   kernel_l1pt.pv_va, kernel_l1pt.pv_pa,
				   L1_TABLE_SIZE, VM_PROT_READ|VM_PROT_WRITE,
				   PTE_PAGETABLE);
	
	/* Map the L2 page tables of the kernel */
	for (i=0; i<NUM_KERNEL_PTS; i++) {
		if ((i % (PAGE_SIZE / L2_TABLE_SIZE_REAL)) == 0) { 
			pmap_map_chunk(l1pagetable,
						   kernel_page_tables[i].pv_va, kernel_page_tables[i].pv_pa,
						   L2_TABLE_SIZE, VM_PROT_READ|VM_PROT_WRITE,
						   PTE_PAGETABLE);
		}
	}
	
	
	/*
	 * Map the vector page
	 */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Mapping the vector pages\n");
#endif
	pmap_map_entry(l1pagetable, ARM_VECTORS_HIGH, systempage.pv_pa,
				   VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	
	
	/*
	 * Map the stack pages
	 */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Mapping the stack pages\n");
#endif
	pmap_map_chunk(l1pagetable, fiqstack.pv_va, fiqstack.pv_pa,
				   FIQ_STACK_SIZE * PAGE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, irqstack.pv_va, irqstack.pv_pa,
				   IRQ_STACK_SIZE * PAGE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, abtstack.pv_va, abtstack.pv_pa,
				   ABT_STACK_SIZE * PAGE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, undstack.pv_va, undstack.pv_pa,
				   UND_STACK_SIZE * PAGE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, kernelstack.pv_va, kernelstack.pv_pa,
				   (KSTACK_PAGES+1) * PAGE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	
	/*
	 * Device map
	 */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Mapping device map\n");
#endif
	pmap_devmap_bootstrap(l1pagetable, omap3_devmap);
	//dump_table((void*)kernel_l1pt.pv_va, 0, 16384, "initial");


	
	/*
	 * Setup the interrupt handler
	 */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Initialising interrupt handler\n");
#endif
	
	
	
	
	
	/* ** Switch L1 TLB table *********************************************************************** */
#ifdef VERBOSE_INIT_ARM
	beagle_printf("DOMAIN_CLIENT=%x, PMAP_DOMAIN_KERNEL=%x\n", DOMAIN_CLIENT, PMAP_DOMAIN_KERNEL);
#endif
	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);
	beagle_printf("cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT); -- Done\n");

	beagle_dump_ttb(kernel_l1pt.pv_va);
	
	setttb(kernel_l1pt.pv_pa);
	beagle_printf("setttb(kernel_l1pt.pv_pa); -- Done\n");

	cpu_tlb_flushID();
	beagle_printf("cpu_tlb_flushID(); -- Done\n");

	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)));
	
#ifdef VERBOSE_INIT_ARM
	beagle_printf("Now using the new L1.\n");
#endif
	

	/*
	 * Pages were allocated during the secondary bootstrap for the
	 * stacks for different CPU modes.
	 * We must now set the r13 registers in the different CPU modes to
	 * point to these stacks.
	 * Since the ARM stacks use STMFD etc. we must set r13 to the top end
	 * of the stack memory.
	 */
	set_stackptr(PSR_FIQ32_MODE, fiqstack.pv_va + FIQ_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_IRQ32_MODE, irqstack.pv_va + IRQ_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_ABT32_MODE, abtstack.pv_va + ABT_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_UND32_MODE, undstack.pv_va + UND_STACK_SIZE * PAGE_SIZE);
	
#ifdef VERBOSE_INIT_ARM
	beagle_printf("STACK: 0x%08x\n", (void *)(fiqstack.pv_va + FIQ_STACK_SIZE * PAGE_SIZE));
	beagle_printf("STACK: 0x%08x\n", (void*)(irqstack.pv_va + IRQ_STACK_SIZE * PAGE_SIZE));
	beagle_printf("STACK: 0x%08x\n", (void *)(abtstack.pv_va + ABT_STACK_SIZE * PAGE_SIZE));
	beagle_printf("STACK: 0x%08x\n", (void *)(undstack.pv_va + UND_STACK_SIZE * PAGE_SIZE));
#endif
	
	/*
	 * We must now clean the cache again....
	 * Cleaning may be done by reading new data to displace any
	 * dirty data in the cache. This will have happened in setttb()
	 * but since we are boot strapping the addresses used for the read
	 * may have just been remapped and thus the cache could be out
	 * of sync. A re-clean after the switch will cure this.
	 * After booting there are no gross relocations of the kernel thus
	 * this problem will not occur after initarm().
	 */
	cpu_idcache_wbinv_all();
	
#ifdef VERBOSE_INIT_ARM
	beagle_printf("About to setup the console ...\n");
#endif
	
	/* ready to setup the console (XXX move earlier if possible) */
	cninit();
	
	
	/* ** Miscellaneous ***************************************************************************** */
	
	/* Exception handlers */
	data_abort_handler_address	= (u_int) data_abort_handler;
	prefetch_abort_handler_address	= (u_int) prefetch_abort_handler;
	undefined_handler_address	= (u_int) undefinedinstruction_bounce;
	undefined_init();
	
	/* Prepares the context of the first process */
	proc_linkup(&proc0, &thread0);
	thread0.td_kstack		= kernelstack.pv_va;
	//thread0.td_pcb			= (struct pcb *) (thread0.td_kstack + (KSTACK_PAGES + 1) * PAGE_SIZE) - 1;
	thread0.td_pcb			= (struct pcb *) (thread0.td_kstack + KSTACK_PAGES * PAGE_SIZE) - 1;
	thread0.td_pcb->pcb_flags	= 0;
	thread0.td_frame		= &proc0_tf;
	pcpup->pc_curpcb		= thread0.td_pcb;
	
	/* Exception vector */
#ifdef VERBOSE_INIT_ARM 
	printf("\nException vector at (0x%08x)\n", ((unsigned int *)systempage.pv_va)[0]);
#endif
	arm_vector_init(ARM_VECTORS_HIGH, ARM_VEC_ALL);
	
	/* First unbacked address of KVM */
	pmap_curmaxkvaddr	= KERNVIRTADDR + 0x100000 * NUM_KERNEL_PTS;
	
	/* Get the size of the SDRAM */
	sdram_size = omap3_sdram_size();
#ifdef VERBOSE_INIT_ARM 
	printf("\nSDRAM size 0x%08X, %dMB\n", sdram_size, (sdram_size / 1024 / 1024));
#endif
	
	/* Physical ranges of available memory. */
	phys_avail[0]	= freemempos;
	phys_avail[1]	= PHYSADDR + sdram_size;
	phys_avail[2]	= 0;
	phys_avail[3]	= 0;
	
	dump_avail[0]	= PHYSADDR;
	dump_avail[1]	= PHYSADDR + sdram_size;
	dump_avail[2]	= 0;
	dump_avail[3]	= 0;
	
	physmem		= sdram_size / PAGE_SIZE;
	
	init_param1();
	init_param2(physmem);
	
	pmap_bootstrap((freemempos&0x007fffff)|0xc0000000, KERNVIRTADDR+0x10000000, &kernel_l1pt);
	
	/* Locking system */
	mutex_init();
	
	/* Kernel debugger */
	kdb_init();
	
	/* initarm returns the address of the kernel stack */
	return (void *)(kernelstack.pv_va + (KSTACK_PAGES + 1) * PAGE_SIZE);
}




void *initarm(void *arg1, void *arg2)
{
	return initarm_boilerplate(arg1, arg2);
}



struct arm32_dma_range *
bus_dma_get_range(void)
{
	/* TODO: Need implementation ? */
	return (NULL);
}

int
bus_dma_get_range_nb(void)
{
	/* TODO: Need implementation ? */
	return (0);
}


