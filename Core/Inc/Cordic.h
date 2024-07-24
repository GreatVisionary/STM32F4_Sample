#ifndef __CORDIC_H__
#define __CORDIC_H__
#include "main.h"

void cordic_config(void);
int32_t value_to_cordic31(float value, float coeff);
void cordic_calculate_start(float arg1)	;
void cordic_get_result(float *res1, float *res2);
void cordic31_to_value(int cordic31, float *res);

#endif

