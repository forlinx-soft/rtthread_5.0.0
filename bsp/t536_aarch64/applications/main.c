/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020/10/7      bernard      the first version
 */

#include <stdio.h>
#include "rthw.h"

#define __section_t(s)	__attribute((__section__(#s)))
#define __resource  __section_t(.resource_table)

struct resource_table {
	unsigned int ver;
	unsigned int num;
	unsigned int reserved[2];
	unsigned int offset[20];
} __packed;

/**
 *	Note: vdev da and vring da need to same with linux dts reserved-memory
 *	otherwise linux will alloc new memory
 * */
static struct resource_table __resource __unused resource_table = {
	.ver = 1,
    .num = 1,
    .reserved = {0, 0},
};

//#define SCHEDULER_TEST
#ifdef SCHEDULER_TEST
void test1_entry(void *parameter)
{
    while (1)
    {
		rt_thread_mdelay(100);
		printf("%s %d\n", __func__, __LINE__);
    }
}


void test2_entry(void *parameter)
{
    while (1)
    {
		rt_thread_mdelay(1000);
		printf("%s %d\n", __func__, __LINE__);
    }
}
#endif

int main(void)
{
	printf("hello rt-thread\n");
#ifdef SCHEDULER_TEST
	rt_thread_t test1_tid;
	rt_thread_t test2_tid;

	test1_tid = rt_thread_create("test1_thread",
								test1_entry, RT_NULL,
								100 * 1024,
								5, 10);

	if (test1_tid != RT_NULL) {
		rt_thread_startup(test1_tid);
	}

	test2_tid = rt_thread_create("test2_thread",
								test2_entry, RT_NULL,
								100 * 1024,
								10, 10);
	if (test2_tid != RT_NULL) {
		rt_thread_startup(test2_tid);
	}
#endif

	return 0;
}
