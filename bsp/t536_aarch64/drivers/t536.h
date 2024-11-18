/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-17     GuEe-GUI     the first version
 */

#ifndef VIRT_H__
#define VIRT_H__

#include <rtdef.h>

#ifdef RT_USING_SMART
#include <mmu.h>
#include <ioremap.h>

#else
#define rt_ioremap(x, ...) (x)
#endif

#define __REG32(x)          (*((volatile unsigned int *)(x)))
#define __REG16(x)          (*((volatile unsigned short *)(x)))

/* UART */
#define UART0_BASE_ADDR (0x02600000)
#define UART1_BASE_ADDR (0x02602000)
#define UART2_BASE_ADDR (0x02603000)
#define UART15_BASE_ADDR (0x07080000)

#define UART0_INTERRUPT            (34)
#define UART1_INTERRUPT            (35)
#define UART2_INTERRUPT            (36)
#define UART15_INTERRUPT           (237)

#define UART_REG_OFFSET (0XFC)

/* GIC */
#define MAX_HANDLERS        287
#define GIC_IRQ_START       0
#define ARM_GIC_NR_IRQS     287
#define ARM_GIC_MAX_NR      1

#define IRQ_ARM_IPI_KICK    0
#define IRQ_ARM_IPI_CALL    1

/* GICv3 */
#define GIC_PL500_DISTRIBUTOR_PPTR      0x03400000
#define GIC_PL500_REDISTRIBUTOR_PPTR    0x03460000
#define GIC_PL500_CONTROLLER_PPTR       RT_NULL
#define GIC_PL500_ITS_PPTR              0x03440000

/* the basic constants and interfaces needed by gic */
rt_inline rt_ubase_t platform_get_gic_dist_base(void)
{
#ifdef BSP_USING_GICV2
    return GIC_PL390_DISTRIBUTOR_PPTR;
#else
    return GIC_PL500_DISTRIBUTOR_PPTR;
#endif
}

rt_inline rt_ubase_t platform_get_gic_redist_base(void)
{
    return GIC_PL500_REDISTRIBUTOR_PPTR;
}

rt_inline rt_ubase_t platform_get_gic_cpu_base(void)
{
#ifdef BSP_USING_GICV2
    return GIC_PL390_CONTROLLER_PPTR;
#else
    return GIC_PL500_CONTROLLER_PPTR;
#endif
}

rt_inline rt_ubase_t platform_get_gic_its_base(void)
{
    return GIC_PL500_ITS_PPTR;
}

#endif
