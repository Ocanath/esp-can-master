#include "cobs.h"


int cobs_encode_single_buffer(cobs_buf_t * msg)
{
	//Encoding an encoded buffer should be allowed. This could be useful for forwarding to 'stupid' devices with constrained COBS handling (take in stream and forward unstuffed payload)
	if(msg == NULL)
	{
		return COBS_ERROR_NULL_POINTER;	//null pointer check
	}
	//overrun check. we offset the buffer by one to prepend the pointer, and index into length+2 to append the delimiter, so this is critical for memory safety
	if(msg->length == 0 || (msg->length + 2 + (int)(msg->length/254)) > msg->size || msg->size < COBS_MIN_BUF_SIZE)
	{
		return COBS_ERROR_SIZE;
	}

	//shift the entire buffer up by one index. 
	//Note: this is unnecessary if cobs_buf_t tracks two pointers; one for 'encoded' and one for 'decoded' on the same run of memory, off by 1. 
	for(int i = msg->length-1; i >= 0; i--)
	{
		msg->buf[i+1] = msg->buf[i];
	}
	msg->length += 1;	//prepend the first pointer byte
	msg->buf[msg->length++] = 0;	//append delimiter byte

	int pointer_idx = 0;
	int block_start = 1;//index of the first byte of the current block. Used primarily for pointer overflow handling
	for(int i = 1; i < msg->length; i++)
	{
		if(msg->buf[i] == 0)
		{
			int pointer_value = i - pointer_idx;
			if(pointer_value > 0xFF)
			{
				return COBS_ERROR_POINTER_OVERFLOW;	//pointer overflow. This should never happen!
			}
			msg->buf[pointer_idx] = (unsigned char)pointer_value;	//we already shift by 1 when we start analyzing the shifted payload, so this load works
			pointer_idx = i;	//the zero bytes are always replaced by pointers
			block_start = i + 1;
		}
		else if(i - block_start >= 254)	//this conditional may only catch the first full block - check multi consecutive full block encoing
		{
			//full block. Shift the buffer up by one, including the delimiter, and increase the length by one.
			for(int j = msg->length-1; j >= i; j--)
			{
				msg->buf[j+1] = msg->buf[j];
			}
			msg->buf[msg->length++] = 0;
			
			//then load the pointer value and update the pointer index.
			msg->buf[pointer_idx] = 0xFF;
			pointer_idx = i;
			block_start = i + 1;
		}
	}

	msg->encoded_state = COBS_ENCODED;
	return COBS_SUCCESS;
}

int cobs_decode_double_buffer(cobs_buf_t* encoded_msg, cobs_buf_t* decoded_msg)
{
	//null pointer checks
	if(encoded_msg == NULL || decoded_msg == NULL)
	{
		return COBS_ERROR_NULL_POINTER;
	}
	//zero length checks
	if(encoded_msg->length == 0)
	{
		return COBS_ERROR_SIZE;
	}
	//quick check to ensure the decode buffer is large enough
	if(decoded_msg->size < encoded_msg->length)
	{
		return COBS_ERROR_SIZE;	
	}

	int pointer_value = encoded_msg->buf[0];
	int pointer_idx = pointer_value;	//the buffer index of the pointer. this is offset by 'i', which is zero when we start.
	int decode_buffer_idx = 0;	//must maintain a decode buffer index because in the case of a full block, we need to skip instead of loading a zero.
	for(int i = 1; i < encoded_msg->length; i++)
	{
		if(encoded_msg->buf[i] == 0)	//Stop at the delimiter.
		{
			//bug - length handler?
			break;
		}

		if(i != pointer_idx)	//This covers 99.9% of the execution of this function
		{
			decoded_msg->buf[decode_buffer_idx++] = encoded_msg->buf[i];
		}
		else if(pointer_value != 255)	//if you're at the pointer and the value is not equal to 255, you need to decode a zero
		{
			decoded_msg->buf[decode_buffer_idx++] = 0;
			pointer_value = encoded_msg->buf[i];
			pointer_idx = pointer_value + i;
		}
		else	//if you're at the next pointer and the pointer value is equal to 255
		{
			pointer_value = encoded_msg->buf[i];
			pointer_idx = pointer_value + i;	//if you have a complete block you don't need to copy a zero, but you do need to update the pointer index
		}
	}
	decoded_msg->length = decode_buffer_idx;	//update length as we copy
	decoded_msg->encoded_state = COBS_DECODED;
	return COBS_SUCCESS;	
}

/*
This function is designed to handle an incoming serial stream of bytes.

This variant does not use a double buffer, which means that we need explicit state management
in order to determine when to parse the buffer.

That state management is handled with the streaming_state variable.

If streaming is complete, that means that the buffer needs to be parsed. The parser should always set the state flag to COBS_STREAMING_PARSED

 */
int cobs_stream(unsigned char new_byte, cobs_buf_t *encoded_msg, cobs_buf_t *decoded_msg)
{
	if(encoded_msg == NULL || decoded_msg == NULL)
	{
		return COBS_ERROR_NULL_POINTER;
	}
	
	if(encoded_msg->length + 1 > encoded_msg->size)
	{
		encoded_msg->length = 0;	//reset the length to recover for the next frame
		return COBS_ERROR_SERIAL_OVERRUN;
	}
	
	// Buffer in the new byte
	encoded_msg->buf[encoded_msg->length++] = new_byte;
	
	if(new_byte == 0)
	{
		//decode the encoded message into the decoded message
		//using cobs_decode_double_buffer
		
		encoded_msg->encoded_state = COBS_ENCODED;
		int rc = cobs_decode_double_buffer(encoded_msg, decoded_msg);
		encoded_msg->length = 0;
		return rc;
	}

	return COBS_STREAMING_IN_PROGRESS;
}
