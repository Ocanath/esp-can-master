/*
 * init.h
 *
 *  Created on: Aug 21, 2024
 *      Author: ocanath
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_
#include "main.h"


extern FDCAN_HandleTypeDef hfdcan1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
void MX_FDCAN1_Init(void);

#endif /* INC_INIT_H_ */
