#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"
#include "sin-math.h"
#include "serial-motor-comms.h"

uint8_t gl_communication_buf[60] = {};
buffer_t gl_msg = {
		.buf = (unsigned char *)gl_communication_buf,
		.size = sizeof(gl_communication_buf),
		.len = 0
};
buffer_t gl_rx_unstuffed = {.buf = m_huart1.ppp_unstuff_buf, .size = sizeof(m_huart1.ppp_unstuff_buf), .len = 0};

uint8_t gl_reply_received = 0;
uint8_t gl_use_ppp = 1;
uint8_t gl_rc = 0;	//for dbugging, rc that can't get optimtized out

/*This is the general comms handler*/
void ppp_uart1_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size > NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM)
	{
		gl_rx_unstuffed.len = h->ppp_unstuffed_size;
		int checksum_idx = h->ppp_unstuffed_size - sizeof(uint16_t);
		uint16_t crc = get_crc16(h->ppp_unstuff_buf, checksum_idx);
		uint16_t * p_checksum = (uint16_t*)(&h->ppp_unstuff_buf[checksum_idx]);
		if(*p_checksum == crc)
		{
			gl_reply_received = 1;
		}
	}

}


void ppp_uart2_rx_cplt_callback(uart_it_t * h)
{

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
//			while(gl_reply_received == 0 && (HAL_GetTick() - wait_start) < timeout);
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
				update_comms_with_read_reply(pword, pcomms, &gl_rx_unstuffed);
				return SUCCESS;
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
/*
 * Saturate output
 * */
int32_t saturate(int32_t v, int32_t sat)
{
	if(v > sat)
		return sat;
	if(v < -sat)
		return -sat;
	return v;
}

comms_t gl_motors[2] = {};
uint32_t prev_command_mode[sizeof(gl_motors)/sizeof(comms_t)];
uint8_t gl_do_pctl = 0;
int32_t pctl_targs[sizeof(gl_motors)/sizeof(comms_t)] = {0};
int main(void)
{
 	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	m_uart_enable_interrupt_flags(&m_huart1);
	m_uart_enable_interrupt_flags(&m_huart2);
	MX_FDCAN1_Init();
	FDCAN_Config();

	uint32_t led_ts = 0;
	uint32_t uart_tx_ts = 0;


	gl_motors[0].fds.module_number = 0x03;
	gl_motors[1].fds.module_number = 0x04;





//	/*TODO: turn this into a generalized function that writes, modifies, reads, and confirms consistency*/
	gl_rc = 1;
	gl_motors[0].motor_command_mode = PCTL_VQ;
	uart_write_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t), 1);
	HAL_Delay(1);
	gl_motors[0].motor_command_mode++;
	int rc = uart_read_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t), 3);
	HAL_Delay(1);
	if(rc == SUCCESS && gl_motors[0].motor_command_mode == PCTL_VQ)
	{
		gl_rc = 0;
	}

//	uart_read_struct_mem_ppp(&gl_motors[0].mpctl_rotor_vq.kpki.kp.i32, &gl_motors[0], sizeof(int32_t)*4, 3000);
//	HAL_Delay(1);
//	gl_motors[0].mpctl_rotor_vq.kpki.kp.i32++;
//	gl_motors[0].mpctl_rotor_vq.kpki.kp.radix--;
//	gl_motors[0].mpctl_rotor_vq.kpki.ki.i32++;
//	gl_motors[0].mpctl_rotor_vq.kpki.ki.radix--;
//	uart_write_struct_mem_ppp(&gl_motors[0].mpctl_rotor_vq, &gl_motors[0], sizeof(int32_t)*4);
//	HAL_Delay(1);
//	uart_read_struct_mem_ppp(&gl_motors[0].mpctl_rotor_vq.kpki.kp.i32, &gl_motors[0], sizeof(int32_t)*4, 3000);


	int motor_index = 0;
	uint8_t reply_pending = 0;	//
	uint32_t misc_read_ts = 0;
	uint32_t wifi_publish_ts = 0;
	while (1)
	{
		uint32_t tick = HAL_GetTick();
		if(tick - wifi_publish_ts > 10)
		{
			wifi_publish_ts = tick;

			int32_t * pmsgbuf = (int32_t*)(gl_msg.buf);
			int idx = 0;
			pmsgbuf[idx++] = gl_motors[0].foc.gl_iq;
			pmsgbuf[idx++] = gl_motors[1].foc.gl_iq;
//			pmsgbuf[idx++] = gl_motors[0].foc.gl_theta_rem_m;
//			pmsgbuf[idx++] = gl_motors[1].foc.gl_theta_rem_m;
			pmsgbuf[idx++] = tick;
			gl_msg.len = idx*sizeof(int32_t);
			int len = PPP_stuff(gl_msg.buf, gl_msg.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
		}

//		gl_motors[0].command_word = 1000;
//		gl_motors[1].command_word = -1000;
		if(gl_do_pctl)
		{
//			for(int i = 0; i < sizeof(gl_motors)/sizeof(comms_t); i++)
//			{
				gl_motors[0].motor_command_mode = 0;
				gl_motors[1].motor_command_mode = 0;

				int32_t iq = (int32_t)(((int64_t)pctl_targs[0] - (int64_t)gl_motors[0].foc.gl_theta_rem_m)/10);
				iq = saturate(iq,1000);
				if(iq > 0)
				{
					gl_motors[0].command_word = iq;
					gl_motors[1].command_word = 0;
				}
				else
				{
					gl_motors[0].command_word = 0;
					gl_motors[1].command_word = -iq;
				}
//				gl_motors[0].command_word = iq;
//				gl_motors[1].command_word = -iq;
//			}
		}

		/*Handle write*/
		if(reply_pending == 0)
		{
			uart_tx_ts = tick;
			create_motor_command(gl_motors[motor_index].fds.module_number, gl_motors[motor_index].command_word, &gl_msg);
			int len = PPP_stuff(gl_msg.buf, gl_msg.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
			reply_pending = 1;
		}
		/*handle read*/
		if(gl_reply_received)
		{
			reply_pending = 0;
			gl_reply_received = 0;
			if(m_huart1.ppp_unstuff_buf[0] == MASTER_MOTOR_ADDRESS)
			{
				parse_motor_message_reply(&gl_rx_unstuffed, &gl_motors[motor_index]);
				motor_index = (motor_index + 1) % (sizeof(gl_motors)/sizeof(comms_t));
			}
			else if(m_huart1.ppp_unstuff_buf[0] == MASTER_MISC_ADDRESS)
			{
				//parse misc message reply
			}
		}
		else if((tick - uart_tx_ts) > 1 && reply_pending != 0)	//read timeout
		{
			motor_index = (motor_index + 1) % (sizeof(gl_motors)/sizeof(comms_t));
			reply_pending = 0;
		}

		if(tick - misc_read_ts > 1 && reply_pending == 0)
		{
			misc_read_ts = tick;
			uart_read_struct_mem_ppp(&gl_motors[0].foc.gl_id, &gl_motors[0], sizeof(int32_t), 1);
			uart_read_struct_mem_ppp(&gl_motors[1].foc.gl_id, &gl_motors[1], sizeof(int32_t), 1);

			for(int i = 0; i < sizeof(gl_motors)/sizeof(comms_t); i++)
			{
				if(gl_motors[i].motor_command_mode != prev_command_mode[i])
				{
					//write, but don't bother to read
					uart_write_struct_mem_ppp(&gl_motors[i].motor_command_mode, &gl_motors[i], sizeof(int32_t), 1);
					prev_command_mode[i] = gl_motors[i].motor_command_mode;
				}
			}
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

