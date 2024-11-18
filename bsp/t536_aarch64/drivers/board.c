/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-11-20     Bernard    the first version
 * 2018-11-22     Jesven     add rt_hw_spin_lock
 *                           add rt_hw_spin_unlock
 *                           add smp ipi init
 */

#include <rthw.h>
#include <rtthread.h>
#include <mmu.h>

#ifdef RT_USING_SMART
#include <lwp_arch.h>
#endif

#include "board.h"
#include <mm_aspace.h>
#include <mm_page.h>
#include <interrupt.h>

#ifdef RT_USING_FDT
#include "interrupt.h"
#include "dtb_node.h"
#include <psci_api.h>
#include <cpu.h>
#endif
#include "sunxi_hal_common.h"
#include <gtimer.h>

#ifdef RT_USING_SMART
struct mem_desc platform_mem_desc[] = {
    {KERNEL_VADDR_START, KERNEL_VADDR_START + 0x0fffffff, (rt_size_t)ARCH_MAP_FAILED, NORMAL_MEM}
};
#else

#define PAGE_POOL_SIZE (2ul << 20)
#define PHYMEM_END (0x58000000ul)

struct mem_desc platform_mem_desc[] =
{
    {0x07000000, 0x07800000 - 1, 0x07000000, DEVICE_MEM}, //for cpus peripherals
    {0x03400000, 0x03500000 - 1, 0x03400000, DEVICE_MEM}, //for gic peripherals
    {KERNEL_VADDR_START, PHYMEM_END - 1, KERNEL_VADDR_START, NORMAL_MEM},
};
#endif

const rt_uint32_t platform_mem_desc_size = sizeof(platform_mem_desc)/sizeof(platform_mem_desc[0]);

void idle_wfi(void)
{
    asm volatile ("wfi");
}

/**
 * This function will initialize board
 */

extern size_t MMUTable[];

#ifdef RT_USING_SMART
rt_region_t init_page_region = {
    PAGE_START,
    PAGE_END,
};
#else
rt_region_t init_page_region = {
    PHYMEM_END - PAGE_POOL_SIZE,
    PHYMEM_END,
};
#endif

void rt_hw_board_init(void)
{
#ifdef RT_USING_SMART
    rt_hw_mmu_map_init(&rt_kernel_space, (void*)0xfffffffff0000000, 0x10000000, MMUTable, PV_OFFSET);
#else
    rt_hw_mmu_map_init(&rt_kernel_space, (void*)0x80000000, 0x10000000, MMUTable, 0);
#endif
    rt_page_init(init_page_region);
    rt_hw_mmu_setup(&rt_kernel_space, platform_mem_desc, platform_mem_desc_size);

    /* initialize system heap */
    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);

    /* initialize hardware interrupt */
    rt_hw_interrupt_init();

    /* support debug feature before components init */
    rt_hw_uart_init();
    
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

    /* initialize timer for os tick */
    rt_hw_gtimer_init();

    rt_thread_idle_sethook(idle_wfi);
#if 0
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
#endif
}
