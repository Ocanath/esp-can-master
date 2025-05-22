#include "serial-comms.h"
#include "checksum.h"

comms_t gl_mem =
{
		.foc = {},		//init to zero
		.fds = {		//definitely do not init to zero
	       .module_number = 0,

	       /*The all important align offset*/
	       .align_offset_fixed = 0,

	       /*These are not hand obtained ever, so no commands for them*/
	       .is_flipped = 1,
	       .misc2 = {{0}},
	       .misc3 = {{0}},
	       .misc4 = {{0}},
	       .misc5 = {{0}},

	       /*every single one of these values needs a command. i might run out of room with an 8bit header...*/
	       .elec_conv_ratio_fixed = 11,
	       .gl_prop_delayloop_interval = 100,
	       .gl_prop_delay_const_12b = 0x7FFFFFFF,
	       .iq_pi = {
	               .kp = {
	                               .i32 = 0,
	                               .radix = 0
	               },
	               .ki = {
	                               .i32 = 0,
	                               .radix = 0
	               },
	               .x_integral_div = 1,    //this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
	               .x_sat = 0,
	               .out_rshift = 31
	       },
	       .id_pi = {
	               .kp = {
	                               .i32 = 0,
	                               .radix = 31
	               },
	               .ki = {
	                               .i32 = 0,
	                               .radix = 31
	               },
	               .x_integral_div = 1,
	               .x_sat = 0,
	               .out_rshift = 31
	       }
	}
};



enum {REPLY_MODE_1 = 0, REPLY_MODE_2 = 1};

/**
TODO: fill this out based on your project needs. 
 */
int cherrypick_reply_data(unsigned char * reply, int reply_len, comms_t * comms)
{
    if(reply == NULL || comms == NULL)
    {
        return 0;
    }
    else if(comms->motor_command_mode == REPLY_MODE_1)
    {
        if(reply_len < sizeof(int32_t) * 2) //check that the target buffer is large enough
        {
            return 0;
        }

        int bidx = 0;   //index for the reply buffer

        //load the first value
        unsigned char * p_val = (unsigned char *)(&comms->foc.gl_iq);
        for(int i = 0; i < sizeof(int32_t); i++)
        {
            reply[bidx++] = p_val[i];
        }

        //load the second value
        p_val = (unsigned char *)(&comms->foc.gl_theta_rem_m);
        for(int i = 0; i < sizeof(int32_t); i++)
        {
            reply[bidx++] = p_val[i];
        }
        return bidx;
    }
    else if(comms->motor_command_mode == REPLY_MODE_2)
    {
        //ETC.
    	return 0;
    }
    else
    {
    	return 0;
    }
}



/*
    Parse an unstuffed (raw) message, including checksum and address splitting.
    Returns:
        -2 if the message is malformed
        -1 if the message is not intended for this device
        0 if the message is successfully parsed

    This function does address filtering and checksum validation
 */
int parse_motor_message(unsigned char motor_address, unsigned char misc_address, unsigned char * msg, int len, unsigned char * p_replybuf, int replybuf_size, int * reply_len,  comms_t * comms)
{   
    if(len < MINIMUM_MESSAGE_LENGTH)
    {
        return ERROR_MALFORMED_MESSAGE;
    }
    uint16_t * pchecksum = (uint16_t *)(msg + len - sizeof(uint16_t));
    uint16_t checksum = get_crc16(msg, len - sizeof(uint16_t));
    if(checksum != *pchecksum)
    {
        return ERROR_CHECKSUM_MISMATCH;
    }
    else
    {
        if(msg[0] == motor_address)
        {
//            int32_t * p_cmd = (int32_t *)(&msg[1]);	//load out the p_cmd to the appropriate variable
            //do something with the command we parsed
            return cherrypick_reply_data(p_replybuf, replybuf_size, comms);
        }
        else if(msg[0] == misc_address)
        {
            //remove address and checksum from the message and then parse
            msg = &(msg[1]);
            return parse_misc_command(msg, (len-(NUM_BYTES_CHECKSUM + NUM_BYTES_ADDRESS)), p_replybuf, replybuf_size, reply_len, comms);
        }
        else
        {
            return ADDRESS_FILTERED;
        }
    }
}
