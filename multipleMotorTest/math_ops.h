#ifndef MATH_OPS_H
#define MATH_OPS_H
 
#include "math.h"
#include <stdint.h>
 
float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
float roundf(float x);
void limit_norm(float *x, float *y, float limit);
int float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, uint8_t bits);
 
#endif
 
