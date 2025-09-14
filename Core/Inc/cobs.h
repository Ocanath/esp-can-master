#ifndef COBS_STUFFING_H
#define COBS_STUFFING_H
#include <stddef.h>
#include <stdint.h>
#include <assert.h>

#define COBS_MIN_BUF_SIZE	3

enum {COBS_DECODED, COBS_ENCODED};  //encoding state

//Return values
enum 
{
	COBS_SUCCESS = 0, 	//success message, zero always

	//error fields
	COBS_ERROR_NULL_POINTER = -1,
	COBS_ERROR_SIZE = -2,
	COBS_ERROR_POINTER_OVERFLOW = -4,
	COBS_ERROR_SERIAL_OVERRUN = -5,
	COBS_ERROR_STREAMING_FRAME_DROPPED = -6,

	//state fields (not errors, not 'success')
	COBS_STREAMING_IN_PROGRESS = -7
};


typedef struct cobs_buf_t
{
    unsigned char * buf;
    size_t size;
    int length;
    uint8_t encoded_state;
} cobs_buf_t;

//Double-buffer decode. Saves an additional copy operation if you are using a double buffered apporach, and relaxes parsing timing on decoded message which is best-practice for efficient serial streaming.
int cobs_decode_double_buffer(cobs_buf_t * encoded_msg, cobs_buf_t * decoded_msg);
//Function template for basic cobs encoding. Takes a DECODED buffer and encodes it into an ENCODED buffer.
int cobs_encode_single_buffer(cobs_buf_t * msg);
//Function template to handle an incoming byte stream. Uses ZERO as the delimiter, and double-buffers the message to allow sufficient time for parsing.
int cobs_stream(unsigned char new_byte, cobs_buf_t * encoded_msg, cobs_buf_t * decoded_msg);	//this almost works if the encoded and decoded messages are the same size - the only issue is the length gets reset to 0 after encoding. You could have this function return the decoded message size instead of success

#endif