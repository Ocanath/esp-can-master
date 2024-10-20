/*
 * iirSOS.c
 *
 *  Created on: May 9, 2018
 *      Author: Ocanath
 */
#include "IIRsos.h"

const uint32_t lpf_emg_update_period = 5;

/*hpf coefficients for pressure sensor filtering. This will
get copied. TODO: make const*/

iirSOS gl_upsampling_filter = {
		.a1 = -1.946074986119715 , //a1
		.a2 = 0.9470357307185885 ,//a2
		.b0 = 0.40824829046386296 , //b0
		.b1 = 0.8164965809277259 , //b1
		.b2 = 0.40824829046386296 , //b2
		.gain = 0.0005883335100936652 , //gain
		.w = {0, 0, 0}
};

float sos_f(iirSOS * f, float newSample)
{
	f->w[2] = f->w[1];
	f->w[1] = f->w[0];
	f->w[0] = newSample - f->a1*f->w[1] - f->a2*f->w[2];
	return f->gain*(f->b0 * f->w[0] + f->b1*f->w[1] + f->b2*f->w[2]);
}

float filter_iir(iirSOS * filter, int order, float new_sample)
{
	float filtered_sample;
	int i = 0;
	filtered_sample = sos_f(&(filter[i]), new_sample);
	for(i=1;i<order;i++)
		filtered_sample = sos_f(&(filter[i]), filtered_sample);
	return filtered_sample;
}
