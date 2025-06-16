/*
 * tick.c
 *
 *  Created on: Jun 16, 2025
 *      Author: ocanath
 */

#include "tick.h"
#include "stm32g4xx_hal.h"


/*Our own wrapper for gettick.
 * They made it a weak, so this should just replace all of em
 * this way we can keep the _hal.h header include in the .c file for mocking
 * */
uint32_t HAL_GetTick(void)
{
	return uwTick;
}
