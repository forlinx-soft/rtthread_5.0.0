#ifndef SUNXI_HAL_COMMON_H
#define SUNXI_HAL_COMMON_H

#include <barrier.h>

/* common register access operation. */
#define hal_readb(reg)          ({uint8_t __v; dsb(); __v = *(volatile uint8_t  *)(long)(reg); __v;})
#define hal_readw(reg)          ({uint16_t __v; dsb(); __v = *(volatile uint16_t  *)(long)(reg); __v;})
#define hal_readl(reg)          ({uint32_t __v; dsb(); __v = *(volatile uint32_t  *)(long)(reg); __v;})
#define hal_writeb(value,reg)   ({dsb(); *(volatile uint8_t  *)(long)(reg) = (value); })
#define hal_writew(value,reg)   ({dsb(); *(volatile uint16_t  *)(long)(reg) = (value);})
#define hal_writel(value,reg)   ({dsb(); *(volatile uint32_t  *)(long)(reg) = (value);})

#ifndef BIT
#define BIT(x) (1 << x)
#endif

#endif  /*SUNXI_HAL_COMMON_H*/