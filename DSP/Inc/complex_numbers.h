#ifndef COMPLEX_NUMBERS_H
#define COMPLEX_NUMBERS_H

/* Private structs ----------------------------------------------------------*/
typedef struct complexNumber {
	float real;
	float img;
} complex;

complex complexAdd(complex, complex);

complex complexMult(complex, complex);

float cabsf_0(complex);

int my_abs(int);

#endif