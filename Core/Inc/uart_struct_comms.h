/*
 * uart_struct_comms.h
 *
 *  Created on: Jun 16, 2025
 *      Author: ocanath
 */

#ifndef INC_UART_STRUCT_COMMS_H_
#define INC_UART_STRUCT_COMMS_H_
#include "serial-comms.h"
#include "serial-motor-comms.h"

extern buffer_t gl_wifi_msg;
extern uint8_t gl_communication_buf[60];
extern buffer_t gl_msg;	//buffer struct wrapper for the gl_communication_buf with portable size and length labelling
extern buffer_t gl_rx_unstuffed;
extern uint8_t gl_reply_received;

int uart_write_struct_mem_ppp(void * pword, comms_t * pcomms, size_t size, uint32_t timeout);
int uart_read_struct_mem_ppp(void * pword, comms_t * pcomms, size_t size, uint32_t timeout);


#endif /* INC_UART_STRUCT_COMMS_H_ */
