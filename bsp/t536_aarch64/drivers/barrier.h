#ifndef __BARRIER_H_
#define __BARRIER_H_

#define ISB	asm volatile ("isb sy" : : : "memory")
#define DSB	asm volatile ("dsb sy" : : : "memory")
#define DMB	asm volatile ("dmb sy" : : : "memory")

#define isb()	ISB
#define dsb()	DSB
#define dmb()	DMB

#define soft_break(...)     	do { __asm__ __volatile__("bkpt #0" ::: "memory", "cc"); } while(0)

#define wmb()         dmb()
#define rmb()         dmb()
#endif