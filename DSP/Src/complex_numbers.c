#include "complex_numbers.h"

#include <math.h>

complex complexAdd(complex a, complex b) {
	complex temp;
	temp.real = a.real + b.real;
	temp.img = a.img + b.img;
	return temp;
}

complex complexMult(complex a, complex b) {
	complex temp;
	temp.real = a.real*b.real - a.img*b.img;
	temp.img = a.real*b.img + a.img*b.real;
	return temp;
}

/* floating complex absolute value */
float cabsf_0(complex a) {
	float term1 = a.real*a.real;
	float term2 = a.img * a.img;
	return sqrt(term1+term2);
}

/* int absolute value */
int my_abs(int n)
{
	if (n == 0) return 0;
	else if (n > 0) return n;
	else return -1*n;
}