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

extern I2S_HandleTypeDef hi2s2;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2S2_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

#endif /* INC_INIT_H_ */
