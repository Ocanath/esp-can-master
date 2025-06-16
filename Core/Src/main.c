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
#include "uart_struct_comms.h"


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
	int rc = 1;
	gl_motors[0].motor_command_mode = PCTL_VQ;
	uart_write_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t), 1);
	HAL_Delay(1);
	gl_motors[0].motor_command_mode++;
	rc = uart_read_struct_mem_ppp(&gl_motors[0].motor_command_mode, &gl_motors[0], sizeof(int32_t), 3);
	HAL_Delay(1);
	if(rc == SUCCESS && gl_motors[0].motor_command_mode == PCTL_VQ)
	{
		rc = 0;
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

		if(gl_wifi_msg.len == 8)
		{
			int msgbidx = 0;
			for(int i = 0; i < sizeof(gl_motors)/sizeof(comms_t); i++)
			{
				int32_t val;
				unsigned char * pval = (unsigned char *)(&val);
				for(int b = 0; b < sizeof(int32_t); b++)
				{
					pval[b] = gl_wifi_msg.buf[msgbidx++];
				}
				gl_motors[i].command_word = val;
			}
			gl_wifi_msg.len = 0;
		}



		if(gl_do_pctl)
		{
			for(int i = 0; i < sizeof(gl_motors)/sizeof(comms_t); i++)
			{
				gl_motors[0].motor_command_mode = 0;
				gl_motors[1].motor_command_mode = 0;
//
				int32_t iq = (int32_t)(((int64_t)pctl_targs[i] - (int64_t)gl_motors[i].foc.gl_theta_rem_m)/10);
				iq = saturate(iq,1000);
				gl_motors[i].command_word = iq;
//				if(iq > 0)
//				{
//					gl_motors[0].command_word = iq;
//					gl_motors[1].command_word = 0;
//				}
//				else
//				{
//					gl_motors[0].command_word = 0;
//					gl_motors[1].command_word = -iq;
//				}
//				gl_motors[0].command_word = iq;
//				gl_motors[1].command_word = -iq;
			}
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
				//parse misc message reply. Will depend on what misc message was sent to begin with
			}



			/*Note. if you send a huart2 frame */
			if(tick - wifi_publish_ts > 10 && m_huart2.tx_cplt != 0)
			{
				wifi_publish_ts = tick;

				gl_msg.len = 0;
				gl_msg.buf[gl_msg.len++] = 'f';
				gl_msg.buf[gl_msg.len++] = 'u';
				gl_msg.buf[gl_msg.len++] = 'c';
				gl_msg.buf[gl_msg.len++] = 'k';
				int32_t * pmsgbuf = (int32_t*)(&gl_msg.buf[gl_msg.len]);
				int idx = 0;
				pmsgbuf[idx++] = gl_motors[0].foc.gl_iq;
				pmsgbuf[idx++] = gl_motors[1].foc.gl_iq;
				pmsgbuf[idx++] = gl_motors[0].foc.gl_theta_rem_m/50;
				pmsgbuf[idx++] = gl_motors[0].command_word/50;
				pmsgbuf[idx++] = gl_motors[1].foc.gl_theta_rem_m/50;
				pmsgbuf[idx++] = tick;
				gl_msg.len += (idx*sizeof(int32_t));
				int len = PPP_stuff(gl_msg.buf, gl_msg.len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));
				m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
				uint32_t waitstart_ts = tick;
				while(m_huart2.tx_cplt == 0)	//block write here.
				{
					 if(HAL_GetTick() - waitstart_ts > 3)	//this sometimes happens from (i believe) interrupt pre-emption?
					 {
						 m_huart2.tx_cplt = 1;
						 break;
					 }
				}
			}

		}
		else if((tick - uart_tx_ts) > 1 && reply_pending != 0)	//read timeout
		{
			motor_index = (motor_index + 1) % (sizeof(gl_motors)/sizeof(comms_t));
			reply_pending = 0;
		}

		if(tick - misc_read_ts > 1 && reply_pending == 0)		//separate for blocking misc reads/writes. Blocking makes more sense for misc, unless you are doing block misc for motor control instead of the real motor command
		{
			misc_read_ts = tick;
//			uart_read_struct_mem_ppp(&gl_motors[0].foc.gl_id, &gl_motors[0], sizeof(int32_t), 1);		//for example, read the ID value once every millisecond
//			uart_read_struct_mem_ppp(&gl_motors[1].foc.gl_id, &gl_motors[1], sizeof(int32_t), 1);

			for(int i = 0; i < sizeof(gl_motors)/sizeof(comms_t); i++)	//if the command mode value changes (i.e. from a watch expression), update it with a write command
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

