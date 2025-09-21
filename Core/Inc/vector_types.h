/*
 * vector_types.h
 *
 *  Created on: May 8, 2025
 *      Author: ocanath
 */

#ifndef INC_VECTOR_TYPES_H_
#define INC_VECTOR_TYPES_H_

/*3 vector variable structure*/
typedef struct vect3_t
{
    float v[3];
}vect3_t;

typedef struct vect6_t
{
	float v[6];
}vect6_t;

/* two-vector floating point structure*/
typedef struct vect2
{
	float v[2];
}vect2;

typedef struct mat4_t
{
	float m[4][4];
}mat4_t;

typedef struct mat3_t
{
	float m[3][3];
}mat3_t;


#endif /* INC_VECTOR_TYPES_H_ */
