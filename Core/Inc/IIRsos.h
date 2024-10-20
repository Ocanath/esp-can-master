/*
 * iirSOS.h
 *
 *  Created on: May 9, 2018
 *      Author: Ocanath
 */

#ifndef IIRSOS_H_
#define IIRSOS_H_
#include "stdint.h"

typedef struct iirSOS
{
	float a1,a2;
	float b0,b1,b2;
	float gain;
	float w[3];
}iirSOS;

extern iirSOS gl_upsampling_filter;

float sos_f(iirSOS * f, float newSample);
float filter_iir(iirSOS * filter, int order, float new_sample);


#endif /* IIRSOS_H_ */
