/*
 * uart_struct_comms.c
 *
 *  Created on: Jun 16, 2025
 *      Author: ocanath
 *
 *
 *
 *      Wrapper that plugs the comms library into the UART hardware interface.
 *      Intention is for this to be portable and unit-testable with mocks.
 */
#include "m_uart.h"
#include "uart_struct_comms.h"
#include "PPP.h"
#include "checksum.h"

buffer_t gl_wifi_msg = {
		.buf = m_huart2.ppp_unstuff_buf,
		.size = sizeof(m_huart2.ppp_unstuff_buf),
		.len = 0
};

uint8_t gl_communication_buf[60] = {};
buffer_t gl_msg = {
		.buf = (unsigned char *)gl_communication_buf,
		.size = sizeof(gl_communication_buf),
		.len = 0
};
buffer_t gl_rx_unstuffed = {.buf = m_huart1.ppp_unstuff_buf, .size = sizeof(m_huart1.ppp_unstuff_buf), .len = 0};

uint8_t gl_reply_received = 0;
uint8_t gl_use_ppp = 1;

/*This is the general comms handler*/
void ppp_uart1_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size > NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM)
	{
		gl_rx_unstuffed.len = h->ppp_unstuffed_size;
		gl_reply_received = 1;
	}

}


void ppp_uart2_rx_cplt_callback(uart_it_t * h)
{
	gl_wifi_msg.len = h->ppp_unstuffed_size;
	//general structure:
	//header (32bit, just for us to filter messages):
	//payload	(packed motor command)
	//checksum
}

int uart_write_struct_mem_ppp(void * pword, comms_t * pcomms, size_t size, uint32_t timeout)
{
	int msg_len = create_write_struct_mem_message(pword, size, pcomms, &gl_msg);
	if(msg_len > 0)
	{
		if(gl_use_ppp != 0)
		{
			int len = PPP_stuff(gl_msg.buf, gl_msg.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);

			uint32_t wait_start = HAL_GetTick();
			while(m_huart1.tx_cplt == 0 && HAL_GetTick() - wait_start < timeout);
		}
		else
		{
			m_uart_tx_start(&m_huart1, gl_msg.buf, gl_msg.len);
		}
		return 0;
	}
	else
		return msg_len;
}


int uart_read_struct_mem_ppp(void * pword, comms_t * pcomms, size_t size, uint32_t timeout)
{
	if(size % sizeof(uint32_t) != 0)	//it's fine to pass a sizeof() param, but we gotta make sure it's a multiple of 4 for this to play nice with the message protocol
	{
		return ERROR_MALFORMED_MESSAGE;
	}
	create_read_struct_mem_message(pword, size/sizeof(uint32_t), pcomms, &gl_msg);
	if(gl_msg.len > 0)
	{
		if(gl_use_ppp != 0)
		{
			int len = PPP_stuff(gl_msg.buf, gl_msg.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
			uint32_t wait_start = HAL_GetTick();
			while(gl_reply_received == 0 && (HAL_GetTick() - wait_start) < timeout);
			if(gl_reply_received)
			{
				gl_reply_received = 0;

				int checksum_idx = m_huart1.ppp_unstuffed_size - sizeof(uint16_t);
				uint16_t crc = get_crc16(m_huart1.ppp_unstuff_buf, checksum_idx);
				uint16_t * p_checksum = (uint16_t*)(&m_huart1.ppp_unstuff_buf[checksum_idx]);
				unsigned char address = m_huart1.ppp_unstuff_buf[0];
				if(*p_checksum == crc && address == MASTER_MISC_ADDRESS)	//crc and address filtering
				{
					return update_comms_with_read_reply(pword, pcomms, &gl_rx_unstuffed);
				}
				else if (address != MASTER_MISC_ADDRESS && *p_checksum == crc)	//return filtered address only if the checksum matches; otherwise that error takes precedence
				{
					return ADDRESS_FILTERED;
				}
				else	//in the remaining two cases the checksum is invalid. in either, the checksum mismatching takes precedence for error reporting
				{
					return ERROR_CHECKSUM_MISMATCH;
				}
			}
			else	//timeout
			{
				return ERROR_TIMEOUT;	//timeout error message
			}
		}
		else
		{
			//TODO: implement this
			return SUCCESS;
		}
	}
	else
	{
		return gl_msg.len;
	}
}


