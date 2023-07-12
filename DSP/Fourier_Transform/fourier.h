#ifndef FOURIER_H
#define FOURIER_H

#include "complex_numbers.h"
#include "arm_math_types.h"

#define iir_inc .5
#define LIST_LEN 32

uint32_t update_and_calculate_average(uint32_t);

void calculate_FFT(void);

#endif