/*
 * init.h
 *
 *  Created on: Aug 20, 2024
 *      Author: Ocanath Robotman
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

#include "main.h"


extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_USART1_UART_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);


#endif /* INC_INIT_H_ */
