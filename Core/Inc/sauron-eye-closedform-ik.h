/*
 * sauron-eye-closedform-ik.h
 *
 *  Created on: Apr 11, 2025
 *      Author: ocanath
 */

#ifndef INC_SAURON_EYE_CLOSEDFORM_IK_H_
#define INC_SAURON_EYE_CLOSEDFORM_IK_H_


int get_ik_angles_float(float vx, float vy, float vz, float * theta1 , float*theta2);
int get_ik_angles_double(double vx, double vy, double vz, double* theta1, double* theta2);

#endif /* INC_SAURON_EYE_CLOSEDFORM_IK_H_ */
