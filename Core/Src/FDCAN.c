/*
 * CAN.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 *
 *      CAN Motor Protocol Breakdown:
 *
 *      For 'NORMAL OPERATION': (torque and position)
 *      	Message ID will be equal to the serial number of the motor (i.e. 23)
 *      	Master controller sends a floating point (ieee 32bit spec) number, parsed as a torque command
 *      	Upon receiving the matching message ID, the motor will put its current position on the bus
 *
 *      For 'EXTENDED COMMAND': (miscellaneous settings)
 *      	Message ID will be equal to MAX ID (0x7FF) minus the motor serial number (i.e. 0x7FF - 23)
 *      	The byte index 3 will contain a word that corresponds to setting or resetting a given binary setting
 *      	Upon receiving this message ID, the motor will also respond with its current position on the bus.
 *
 */
#include <FDCAN.h>

#define DEFAULT_V_REPORT_RADIX	4
#define DEFAULT_IQ_RSHIFT		3	//estimated max value of iq is in the ~17bit domain. shift 3 to get it between 14-15bit

uint16_t gl_ext_cmd_id;

FDCAN_TxHeaderTypeDef		can_tx_header;
FDCAN_RxHeaderTypeDef		can_rx_header;

uint32_t				can_tx_mailbox;

//unsigned char can_tx_buf[64] = {};
can_payload_t can_tx_mem = {};
//buffer_t can_tx =
//{
//		.buf = (unsigned char *)(&can_tx_mem.u8[0]),
//		.size = sizeof(can_tx_mem),
//		.len = 0
//};


//unsigned char can_rx_buf[64] = {};
can_payload_t can_rx_mem = {};
//buffer_t can_rx =
//{
//		.buf = (unsigned char *)(&can_rx_mem.u8[0]),
//		.size = sizeof(can_rx_mem),
//		.len = 0
//};

//lookup table. index is the shifted DATALENGTH value, and value is the true message length
static uint8_t fdcan_length_lookup[] =
{
		0,
		1,
		2,
		3,
		4,
		5,
		6,
		7,
		8,
		12,
		16,
		20,
		24,
		32,
		48,
		64
};

///*
// * Generic can buffer send function
// * */
//int send_fdcan_frame(uint16_t id, buffer_t * buffer)
//{
//	can_tx_header.Identifier = id;
//	if(buffer->len > 0 && buffer->len <= 8)
//	{
//		can_tx_header.DataLength = (buffer->len & 0xF) << 16;
//	}
//	else if (buffer->len > 8)	//could build a function that uses division and modulo arithmetic to accomplish this but i believe this is more performant for short messages cus you fall thru the if statements
//	{
//		if(buffer->len == 12)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
//		}
//		else if(buffer->len == 16)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_16;
//		}
//		else if(buffer->len == 20)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_20;
//		}
//		else if(buffer->len == 24)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_24;
//		}
//		else if(buffer->len == 32)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_32;
//		}
//		else if(buffer->len == 48)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_48;
//		}
//		else if(buffer->len == 64)
//		{
//			can_tx_header.DataLength = FDCAN_DLC_BYTES_64;
//		}
//	}
//	else
//	{
//		return ERROR_INVALID_ARGUMENT;
//	}
//
//	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, buffer->buf);
//	while((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) != 0U);
//
//	return DARTT_PROTOCOL_SUCCESS;
//}
//
//


void FDCAN_Config(void)
{
	//This is all different for FD CAN
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = (0x7FF - 0);
	sFilterConfig.FilterID2 = 0;	//idfk. Guess we're always 0
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	can_tx_header.Identifier = 0;
	can_tx_header.IdType = FDCAN_STANDARD_ID;
	can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
	can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
	can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	can_tx_header.MessageMarker = 0;
}

