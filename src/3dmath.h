#ifndef _3DMATH_H
#define _3DMATH_H


#include <math.h>
#include "stm32f4xx.h"


typedef struct {
	float w;
	float x;
	float y;
	float z;
} quaternion;

typedef struct {
	float x;
	float y;
	float z;
} vector;


void 		quaternion_normalize(quaternion *q);
float		quaternion_get_magnitude(quaternion q);
quaternion	quaternion_get_product(quaternion q1, quaternion q2);
quaternion	quaternion_get_conjugate(quaternion q);
void		quaternion_zero(quaternion *q);
void		vector_normalize(vector *v);
float		vector_get_magnitude(vector v);
void		vector_zero(vector *v);
vector		vector_rotate(vector v, quaternion q);
float		vector_dot_product(vector u, vector v);
vector		vector_cross_product(vector u, vector v);
vector 		vector_scale(float scale, vector v);
vector		vector_add(vector u, vector v);


#endif
