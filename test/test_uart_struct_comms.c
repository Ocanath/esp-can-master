/*
 * test_uart_struct_comms.c
 *
 *  Created on: Jun 16, 2025
 *      Author: ocanath
 */
#include "checksum.h"
#include "PPP.h"
#include "serial-comms.h"
#include "serial-motor-comms.h"
#include "uart_struct_comms.h"
#include "unity.h"
#include "mock_m_uart.h"
#include "mock_tick.h"


uint8_t gl_ppp_stuff_buf[128] = {};
/*Initialize a baremetal uart handler structure for UART 1*/
uart_it_t m_huart2 =
{
		.uart_instance = NULL,
		.bytes_received = 0,
		.bytes_to_send = 0,
		.rx_buf = {0},
		.tx_buf = 0,
		.rx_idx = 0,
		.tx_idx = 0,
		.rs485_de_gpio_port = NULL,
		.rs485_de_gpio_pin = 0
};

uart_it_t m_huart1 =
{
		.uart_instance = NULL,
		.bytes_received = 0,
		.bytes_to_send = 0,
		.rx_buf = {0},
		.tx_buf = 0,
		.rx_idx = 0,
		.tx_idx = 0,
		.rs485_de_gpio_port = NULL,
		.rs485_de_gpio_pin = 0
};


void setUp(void)
{
    // Setup code
}

void tearDown(void)
{
    // Teardown code
}

void test_uart_write_struct_mem_ppp(void)
{
    // Test implementation will go here
}
