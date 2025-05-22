/*
 * misc-types.h
 *
 *  Created on: May 22, 2025
 *      Author: ocanath
 *
 *
 *      This header contains type-punning unions and other helper structures that are for general purpose data manipulation
 */

#ifndef INC_MISC_TYPES_H_
#define INC_MISC_TYPES_H_
#include <stdint.h>

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

typedef struct i32_t
{
	int32_t i32;
	int32_t radix;	//'decimal' point. true value is i32/2^radix
}i32_t;


#endif /* INC_MISC_TYPES_H_ */
