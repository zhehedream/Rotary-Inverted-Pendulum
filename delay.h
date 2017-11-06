#ifndef _DELAY_H
#define _DELAY_H
#include "intrinsics.h"
#define F_CPU ((double) 12100000)//16M MCLK=DCO
#define _delay_us(x) __delay_cycles((long)((F_CPU * (double)x / 1000000.0)))
#define _delay_ms(x) __delay_cycles((long)((F_CPU * (double)x / 1000.0)))
#define delay_us(x) __delay_cycles((long)((F_CPU * (double)x / 1000000.0)))
#define delay_ms(x) __delay_cycles((long)((F_CPU * (double)x / 1000.0)))

#endif