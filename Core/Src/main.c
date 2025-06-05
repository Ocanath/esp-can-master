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

uint8_t gl_msg_buf[62] = {};
uint8_t gl_reply_received = 0;
uint8_t gl_use_ppp = 1;
uint8_t gl_rc = 0;	//for dbugging, rc that can't get optimtized out

/*This is the general comms handler*/
void ppp_uart1_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size > NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM)
	{
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


int uart_write_struct_mem_ppp(void * pword, comms_t * pcomms, size_t size)
{
	int msg_len = create_write_struct_mem_message(pword, size, pcomms, gl_msg_buf, sizeof(gl_msg_buf));
	if(msg_len > 0)
	{
		if(gl_use_ppp != 0)
		{
			int len = PPP_stuff(gl_msg_buf, msg_len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
		}
		else
		{
			m_uart_tx_start(&m_huart1, gl_msg_buf, msg_len);
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
	int msg_len = create_read_struct_mem_message(pword, size/sizeof(uint32_t), pcomms, gl_msg_buf, sizeof(gl_msg_buf));
	if(msg_len > 0)
	{
		if(gl_use_ppp != 0)
		{
			int len = PPP_stuff(gl_msg_buf, msg_len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
			uint32_t wait_start = HAL_GetTick();
			while(gl_reply_received == 0 && (HAL_GetTick() - wait_start) < timeout);
			if(gl_reply_received)
			{
				gl_reply_received = 0;
				update_comms_with_read_reply(pword, pcomms, m_huart1.ppp_unstuff_buf, m_huart1.ppp_unstuffed_size);
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
		return msg_len;
}

comms_t gl_motors[2] = {};

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

	/*TODO: turn this into a generalized function that writes, modifies, reads, and confirms consistency*/
	gl_rc = 1;
	gl_motors[0].motor_command_mode = PCTL_VQ;
	uart_write_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t));
	HAL_Delay(1);
	gl_motors[0].motor_command_mode++;
	int rc = uart_read_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t), 3000);
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




	while (1)
	{
		uint32_t tick = HAL_GetTick();

		/*Handle comms*/
		if(tick - uart_tx_ts > 5)
		{
			uart_tx_ts = tick;

			int msg_len = create_motor_command(0x03, 1000, gl_msg_buf, sizeof(gl_msg_buf));
			int len = PPP_stuff(gl_msg_buf, msg_len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart1, gl_ppp_stuff_buf, len);
		}
		if(gl_reply_received)
		{
			if(m_huart1.ppp_unstuff_buf[0] == MASTER_ADDRESS)
			{
				parse_motor_message_reply(m_huart1.ppp_unstuff_buf, m_huart1.ppp_unstuffed_size, &gl_motors[0]);
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

