/*
 * serial-comms-FDCAN.c
 *
 *  Created on: Jun 17, 2025
 *      Author: ocanath
 */
#include "serial-comms-FDCAN.h"
#include "checksum.h"

/*
 * The use of a 2-byte index and FDCAN's size truncation means we have to get clever about padding messages.
 * My thoughts:
 *
 * 1. Motor command sizes could be of size 4,6,7, whatever. They should likely remain under 8 though
 * 2. misc command sizes will be of the following sizes:
 * 		Read:
 * 			4 bytes, fixed always
 * 		Write:
 * 			Any write size between 3 and 8 bytes should be accepted.
 * 			This is because, although we'd be writing incomplete words, we may want to type-pun
 * 			settings with partial writes. SO these remain valid.
 *
 *			Above 8, that's when we need to start padding messages.
 *			The method is simple: just pad off two bytes from the end
 *
 *
 *
 * */


/*
 * Take a 'serial write' data frame and parse it into a can struct (i.e. separate out the fields for address, ID, and checksum)
 * This adds a bit of compute overhead and a redundant crc, so it's better to load the can_frame directly rather than
 * */
int create_can_frame_from_message(buffer_t * msg, can_frame_t * canframe)
{
	if(msg == NULL || canframe == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	if(msg->size < msg->len || msg->buf == NULL)	//sanity check
	{
		return ERROR_INVALID_ARGUMENT;
	}
	canframe->id = (uint32_t)msg->buf[0]; //address is always the first argument
    canframe->data = (uint8_t*)(&msg->buf[NUM_BYTES_ADDRESS]);	//start the data section after the address. Instead of copying the buffer, use a pointer
    uint32_t len = (uint32_t)(msg->len - (NUM_BYTES_ADDRESS + NUM_BYTES_CHECKSUM));	//CRC and address are built-in for CAN, so chop them off the length
    if(len >= 0 && len <= 8)
	{
    	canframe->length = len;
	}
	else if(len > 8 && len <= 24)
	{
		if(len % 4 == 0)	//only allow 12, 16, 20, and 24. This is defined on a protocol level.
		{
			canframe->length = len;
		}
		else	//we could automatically pad here, but in the interest of forcing the programmer to use this correctly we'll throw an error instead.
		{
			canframe->length = 0;
			return ERROR_INVALID_ARGUMENT;	//IF YOU ERROR OUT HERE, IT'S BECAUSE YOU NEED TO PAD YOUR MESSAGE
		}
	}
	else if(len > 24 && len <= 64)
	{
		if(len % 8 == 0)	//after 24, only allow 32, 48 and 64. This is defined on a protocol level.
		{
			canframe->length = len;
		}
		else
		{
			canframe->length = 0;
			return ERROR_INVALID_ARGUMENT;
		}
	}
	else
	{
		canframe->length = 0;
		return ERROR_INVALID_ARGUMENT;
	}
    return SERIAL_PROTOCOL_SUCCESS;
}


/*
 * Take a CAN frame and load it into a buffer_t
 * This does require copying
 * */
int create_message_from_can_frame(can_frame_t * canframe, buffer_t * msg)
{
	if(msg == NULL || canframe == NULL)
	{
		return ERROR_INVALID_ARGUMENT;
	}
	//load the address
	int cur_byte_index = 0;
	msg->buf[cur_byte_index++] = (uint8_t)(canframe->id & 0xFF);
	//bounds check for array overrun protection
	if(canframe->length + cur_byte_index >= msg->size)
	{
		return ERROR_INVALID_ARGUMENT;	//don't overrun the message buffer. pre-check before loop entry
	}


	int can_message_end = canframe->length;
	//if it's greater than 8, it must be 12, 16, 20, 24, 32, 48 or 64
	if(can_message_end > 8)
	{
		can_message_end -= NUM_BYTES_INDEX;
	}
	//copy the payload section
	for(int i = 0; i < can_message_end; i++)
	{
		msg->buf[cur_byte_index++] = canframe->data[i];
	}
	//compute and load teh checksum
    uint16_t checksum = get_crc16(msg->buf, cur_byte_index);
    unsigned char * p_checksum = (unsigned char *)(&checksum);
    msg->buf[cur_byte_index++] = p_checksum[0];    //byte 5 loaded
    msg->buf[cur_byte_index++] = p_checksum[1];    //byte 6 loaded
    //done, load size and return
    msg->len = cur_byte_index;
    return SERIAL_PROTOCOL_SUCCESS;
}
