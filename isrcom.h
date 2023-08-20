/*
 * Released under the GPL License, Version 3
 */

#ifndef _ISRCOM_H_
#define _ISRCOM_H_

// The concept here is loaned from linux; except that SDCC doesnt do typeof(x), so we need one per type.
#define READ_ONCE_U16(x)	(*(const volatile uint16_t *)&(x))
#define WRITE_ONCE_U16(x, val)						\
do {									\
	*(volatile uint16_t *)&(x) = (val);				\
} while (0)

#define READ_ONCE_U8(x)	(*(const volatile uint8_t *)&(x))
#define WRITE_ONCE_U8(x, val)						\
do {									\
	*(volatile uint8_t *)&(x) = (val);				\
} while (0)

#define READ_ONCE_S8(x)	(*(const volatile int8_t *)&(x))
#define WRITE_ONCE_S8(x, val)						\
do {									\
	*(volatile int8_t *)&(x) = (val);				\
} while (0)


#define READ_ONCE_U32(x)	(*(const volatile uint32_t *)&(x))
#define WRITE_ONCE_U32(x, val)						\
do {									\
	*(volatile uint32_t *)&(x) = (val);				\
} while (0)

#endif