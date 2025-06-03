#include "init.h"
#include "m_uart.h"
#include "PPP.h"
#include "checksum.h"
#include "FDCAN.h"
#include "trig_fixed.h"
#include "IIRsos.h"
#include "m_mcpy.h"
#include "sin-math.h"

#define NUM_MOTORS 2

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;

enum {POSITION = 0xFA, TURBO = 0xFB, STEALTH = 0xFC, PCTL_VELOCITY = 0xFD};

void m_uart2_rx_cplt_callback(uart_it_t * h)
{

}


typedef struct uart_can_request_t
{
	uint8_t mode;
	int32_t commands[NUM_MOTORS];
}uart_can_request_t;
uart_can_request_t gl_crq = {0};

static uint8_t uart_buf_received = 0;
static uint32_t last_ppp_message_recieved_ts = 0;

void ppp_rx_cplt_callback(uart_it_t * h)
{
	if(h->ppp_unstuffed_size != 0 && (h->ppp_unstuffed_size % 2) == 0)	//nonzero and even is our basic callback entry filter
	{
		uart_buf_received = 1;
	}
}

uint8_t firststuff[14*2 + 2];


void send_misc_u8(uint16_t id, uint8_t header, uint8_t val)
{
	for(int i = 0; i < 8; i++)
		can_tx_data.d[i] = 0;
	can_tx_data.d[0] = header;
	can_tx_data.d[1] = val;
	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
}

void send_misc_i32(uint16_t id, uint8_t header, int32_t val)
{
	for(int i = 0; i < 8; i++)
		can_tx_data.d[i] = 0;
	can_tx_data.d[0] = header;
	u32_fmt_t fmt;
	fmt.i32 = val;
	for(int i = 0; i < sizeof(int32_t); i++)
	{
		can_tx_data.d[i+1] = fmt.u8[i];
	}
	can_tx_data.d[1] = val;
	can_tx_header.Identifier = (0x7FF - id);	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
	HAL_Delay(1);	//delay to let the message go out
}

void send_motor_i32(uint16_t id, int32_t val)
{

	can_tx_data.i32[0] = val;
	can_tx_header.Identifier = id;	//0x7FF for misc commands
	can_tx_header.DataLength = (8 & 0xF) << 16;	//note: len value above 8 will index into higher values. i.e. F corresponds to 64bytes
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.d);
	while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);
}

typedef struct m_motor_t
{
	uint16_t id;
	int32_t can_command;

	int32_t position;
	int16_t current;
	int16_t velocity;

	uint8_t control_mode;
	//position control iq vq settings
	uint8_t enabled_uart_encoder;
	uint8_t led_state;
}m_motor_t;

static m_motor_t motors[NUM_MOTORS] =
{
			{
					.id = 7,
					.control_mode = SET_SINUSOIDAL_MODE,
					.led_state = 1
			},
			{
					.id = 8,
					.control_mode = SET_SINUSOIDAL_MODE,
					.led_state = 1
			}
};


int32_t m1_velocitypos = 0;
int32_t m2_velocitypos = 0;
int32_t m1_velocity = 0;
int32_t m2_velocity = 0;

int32_t m0_targ14 = 0;
int32_t m1_targ14 = 0;

//setport 6701

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

	while (1)
	{
		uint32_t tick = HAL_GetTick();

		/*Handle comms*/
		if(uart_buf_received != 0)
		{
			uart_buf_received = 0;
			last_ppp_message_recieved_ts = tick;
			//mode with 1 byte of padding, position, checksum
			/*Blast out the motor data back to the person who asked us to move! client doesn't really need to parse it*/
			uint8_t prestuff[3*sizeof(int32_t)+1*sizeof(int16_t)] = {0};	//motor1 pos, motor2 pos, fletcher's
			/*
			* Bytes 0,1,2,3 - motor1 position
			* Bytes 4,5,6,7 - motor2 position
			* Bytes 8,9,10,11 - time ms
			 * Bytes 12,13: checksum16
			 * */
			int32_t * pbi32 = (int32_t*)(&prestuff[0]);
			uint16_t * pbu16 = (uint16_t*)(&prestuff[0]);
			pbi32[0] = motors[0].position; //sizeof(int32_t)*index + sizeof(int32_t) - 1
			pbi32[1] = motors[1].position;
			pbi32[2] = tick;
			pbu16[6] = fletchers_checksum16(pbu16, 6);

			int len = PPP_stuff(prestuff, sizeof(prestuff), firststuff, sizeof(firststuff));
			len = PPP_stuff(firststuff, len, gl_ppp_stuff_buf, sizeof(gl_ppp_stuff_buf));	//double stuff the buffer! AAAH
			m_uart_tx_start(&m_huart2, gl_ppp_stuff_buf, len);
		}

		/*LED blink*/
		if(tick - led_ts > 100)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			led_ts = tick;	//led stays on for 10ms if there is can tx activity (or rx activity?)
		}
	}
}

