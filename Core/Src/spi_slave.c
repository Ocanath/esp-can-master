/*
 * spi_slave.c
 *
 *  Created on: Jan 13, 2021
 *      Author: Ocanath
 */
#include "spi_slave.h"
//#include "stm32g0xx_hal_spi.h"
//
//#define SPI_MODE_3 0x00000003UL	//CPOL and CPHA both are 1
//#define SSM_BIT		(1 << 9)
//#define SSI_BIT		(1 << 8)
//
//
//typedef union
//{
//	uint16_t v;
//	uint8_t d[sizeof(uint16_t)/sizeof(uint8_t)];
//}uint16_fmt_t;
//
//
//volatile int rxidx = 0;
//volatile int txidx = 0;
//
//
///*
// * Fast spi init.
// *
// * See reference manual 'configuration of spi' for more details
// * */
//void spi1_reinit_fast(void)
//{
//
//	/*Reset the SPI peripheral using RCC*/
//	hspi1.Instance->CR1 &= ~(1 << 6);	//disable SPI1
//	RCC->APBRSTR2 |= (1 << 12);	//set SPI1 reset flag!
//	while( (RCC->APBRSTR2 & (1 << 12)) == 0 );	//spin until flag is set
//	RCC->APBRSTR2 &= ~(1 << 12);	//clear SPI1 reset flag!
//	while( (RCC->APBRSTR2 & (1 << 12)) != 0 );	//spin until flag is cleared.
//
//	/*Initialize SPI FAST. Set only values which need to be changed from reset value*/
//
//	//cpol and cpha
//	SPI1->CR1 |= SPI_MODE_3;	//CPOL and CPHA are both set to 1
//
//	//set ssm/ssi
//	SPI1->CR1 |= SSM_BIT;
//	hspi1.Instance->CR1 |= SSI_BIT;	//de-select ss line
//
//	//frxth
//	hspi1.Instance->CR2 |= (1 << 12);	//set the fifo trigger to 1/4, which allows you to handle stray bytes without alignment issues
//
//	//enable!
//	hspi1.Instance->CR1 |= 0x1 << 6;	//enable SPI.
//	__HAL_SPI_ENABLE_IT(&hspi1, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));	//enable all interrupts
//}
