#include "fourier.h"
#include "complex_numbers.h"
#include "arm_math_types.h"

static float first_vals[16];
/* return vals is written to by the running average calculator */
/* it is used by fourier transform and also by our main loop */
volatile float32_t return_vals[16];
/* list counter is used for our average volume list ring buffer */
static uint8_t list_counter = 0;
/* average volume list is written to by average volume and read from by fourier */
static uint32_t average_volume_list[LIST_LEN];

complex second_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};
complex third_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex fourth_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex fifth_vals[16] =
{
		[0] =  { .real = 0.0,     .img = 0.0 },
		[1] =  { .real = 0.0,     .img = 0.0 },
		[2] =  { .real = 0.0,     .img = 0.0 },
		[3] =  { .real = 0.0,     .img = 0.0 },
		[4] =  { .real = 0.0,     .img = 0.0 },
		[5] =  { .real = 0.0,     .img = 0.0 },
		[6] =  { .real = 0.0,     .img = 0.0 },
		[7] =  { .real = 0.0,     .img = 0.0 },
		[8] =  { .real = 0.0,     .img = 0.0 },
		[9] =  { .real = 0.0,     .img = 0.0 },
		[10] = { .real = 0.0,     .img = 0.0 },
		[11] = { .real = 0.0,     .img = 0.0 },
		[12] = { .real = 0.0,     .img = 0.0 },
		[13] = { .real = 0.0,     .img = 0.0 },
		[14] = { .real = 0.0,     .img = 0.0 },
		[15] = { .real = 0.0,     .img = 0.0 }
};

complex twiddle_factors[16] =
{
		[0] =  { .real = 1.0,     .img = 0.0     },
		[1] =  { .real = 0.9239,  .img = -0.3827 },
		[2] =  { .real = 0.7071,  .img = -0.7072 },
		[3] =  { .real = 0.3827,  .img = -0.9239 },
		[4] =  { .real = 0.0,     .img = -1.0    },
		[5] =  { .real = -0.3827, .img = -0.9239 },
		[6] =  { .real = -0.7071, .img = -0.7071 },
		[7] =  { .real = -0.9239, .img = -0.3827 },
		[8] =  { .real = -1.0,    .img = 0.0     },
		[9] =  { .real = -0.9239, .img = 0.3827  },
		[10] = { .real = -0.7071, .img = 0.7071  },
		[11] = { .real = -0.3827, .img = 0.9239  },
		[12] = { .real = 0.0,     .img = 1.0     },
		[13] = { .real = 0.3827,  .img = 0.9239  },
		[14] = { .real = 0.7071,  .img = 0.7071  },
		[15] = { .real = 0.9239,  .img = 0.3827  }
};

uint32_t update_and_calculate_average(uint32_t value) {
	average_volume_list[list_counter] = value;
	int sum = 0;
	for (int i = 0; i < LIST_LEN; i++) {
		sum += average_volume_list[i];
	}
	sum = sum / LIST_LEN;
	list_counter++;
	list_counter = list_counter % LIST_LEN;
	return sum;
}

void calculate_FFT(void) {
	// use average value list to calculate 16 point FFT.
	// we only want the top 11 values of the FFT so Y[15]...Y[5]

	uint8_t counter_pos = list_counter + 1;

	uint32_t temp_average_volume_list[16];

	for (int i = 0; i < 16; i++) {
		temp_average_volume_list[i] = average_volume_list[(counter_pos+i)%LIST_LEN];
	}

	first_vals[0] = (float)temp_average_volume_list[0];
	first_vals[1] = (float)temp_average_volume_list[1];
	first_vals[2] = (float)temp_average_volume_list[2];
	first_vals[3] = (float)temp_average_volume_list[3];
	first_vals[4] = (float)temp_average_volume_list[4];
	first_vals[5] = (float)temp_average_volume_list[5];
	first_vals[6] = (float)temp_average_volume_list[6];
	first_vals[7] = (float)temp_average_volume_list[7];
	first_vals[8] = (float)temp_average_volume_list[8];
	first_vals[9] = (float)temp_average_volume_list[9];
	first_vals[10] = (float)temp_average_volume_list[10];
	first_vals[11] = (float)temp_average_volume_list[11];
	first_vals[12] = (float)temp_average_volume_list[12];
	first_vals[13] = (float)temp_average_volume_list[13];
	first_vals[14] = (float)temp_average_volume_list[14];
	first_vals[15] = (float)temp_average_volume_list[15];

	second_vals[0] = (complex){.real = first_vals[0] + first_vals[1], .img = 0.0};
	second_vals[1] = (complex){.real = first_vals[0] + first_vals[1], .img = 0.0};
	second_vals[2] = (complex){.real = first_vals[2] + first_vals[3], .img = 0.0};
	second_vals[3] = (complex){.real = first_vals[2] + first_vals[3], .img = 0.0};
		second_vals[3] = complexMult(second_vals[3], twiddle_factors[4]);
	second_vals[4] = (complex){.real = first_vals[4] + first_vals[5], .img = 0.0};
	second_vals[5] = (complex){.real = first_vals[4] + first_vals[5], .img = 0.0};
	second_vals[6] = (complex){.real = first_vals[6] + first_vals[7], .img = 0.0};
	second_vals[7] = (complex){.real = first_vals[6] + first_vals[7], .img = 0.0};
		second_vals[7] = complexMult(second_vals[7], twiddle_factors[4]);
	second_vals[8] = (complex){.real = first_vals[8] + first_vals[9], .img = 0.0};
	second_vals[9] = (complex){.real = first_vals[8] + first_vals[9], .img = 0.0};
	second_vals[10] =(complex){.real = first_vals[10] +first_vals[11],.img = 0.0};
	second_vals[11] =(complex){.real = first_vals[10] +first_vals[11],.img = 0.0};
		second_vals[11] = complexMult(second_vals[11], twiddle_factors[4]);
	second_vals[12] =(complex){.real = first_vals[12] +first_vals[13],.img = 0.0};
	second_vals[13] =(complex){.real = first_vals[12] +first_vals[13],.img = 0.0};
	second_vals[14] =(complex){.real = first_vals[14] +first_vals[15],.img = 0.0};
	second_vals[15] =(complex){.real = first_vals[14] +first_vals[15],.img = 0.0};
		second_vals[15]= complexMult(second_vals[15], twiddle_factors[4]);

	third_vals[0] = complexAdd(second_vals[0], second_vals[2]);
	third_vals[1] = complexAdd(second_vals[1], second_vals[3]);
	third_vals[2] = complexAdd(second_vals[0], second_vals[2]);
	third_vals[3] = complexAdd(second_vals[1], second_vals[3]);
	third_vals[4] = complexAdd(second_vals[4], second_vals[6]);
	third_vals[5] = complexAdd(second_vals[5], second_vals[7]);
		third_vals[5] = complexMult(third_vals[5], twiddle_factors[2]);
	third_vals[6] = complexAdd(second_vals[4], second_vals[6]);
		third_vals[6] = complexMult(third_vals[6], twiddle_factors[4]);
	third_vals[7] = complexAdd(second_vals[5], second_vals[7]);
		third_vals[7] = complexMult(third_vals[7], twiddle_factors[6]);
	third_vals[8] = complexAdd(second_vals[8], second_vals[10]);
	third_vals[9] = complexAdd(second_vals[9], second_vals[11]);
	third_vals[10] = complexAdd(second_vals[8], second_vals[10]);
	third_vals[11] = complexAdd(second_vals[9], second_vals[11]);
	third_vals[12] = complexAdd(second_vals[12], second_vals[14]);
	third_vals[13] = complexAdd(second_vals[13], second_vals[15]);
		third_vals[13] = complexMult(third_vals[13], twiddle_factors[2]);
	third_vals[14] = complexAdd(second_vals[12], second_vals[14]);
		third_vals[14] = complexMult(third_vals[14], twiddle_factors[4]);
	third_vals[15] = complexAdd(second_vals[13], second_vals[15]);
		third_vals[15] = complexMult(third_vals[15], twiddle_factors[6]);

	fourth_vals[0] = complexAdd(third_vals[0], third_vals[4]);
	fourth_vals[1] = complexAdd(third_vals[1], third_vals[5]);
	fourth_vals[2] = complexAdd(third_vals[2], third_vals[6]);
	fourth_vals[3] = complexAdd(third_vals[3], third_vals[7]);
	fourth_vals[4] = complexAdd(third_vals[0], third_vals[4]);
	fourth_vals[5] = complexAdd(third_vals[1], third_vals[5]);
	fourth_vals[6] = complexAdd(third_vals[2], third_vals[6]);
	fourth_vals[7] = complexAdd(third_vals[3], third_vals[7]);
	fourth_vals[8] = complexAdd(third_vals[8], third_vals[12]);
	fourth_vals[9] = complexAdd(third_vals[9], third_vals[13]);
		fourth_vals[9] = complexMult(fourth_vals[9], twiddle_factors[1]);
	fourth_vals[10] = complexAdd(third_vals[10], third_vals[14]);
		fourth_vals[10] = complexMult(fourth_vals[10], twiddle_factors[2]);
	fourth_vals[11] = complexAdd(third_vals[11], third_vals[15]);
		fourth_vals[11] = complexMult(fourth_vals[11], twiddle_factors[3]);
	fourth_vals[12] = complexAdd(third_vals[8], third_vals[12]);
		fourth_vals[12] = complexMult(fourth_vals[12], twiddle_factors[4]);
	fourth_vals[13] = complexAdd(third_vals[9], third_vals[13]);
		fourth_vals[13] = complexMult(fourth_vals[13], twiddle_factors[5]);
	fourth_vals[14] = complexAdd(third_vals[10], third_vals[14]);
		fourth_vals[14] = complexMult(fourth_vals[14], twiddle_factors[6]);
	fourth_vals[15] = complexAdd(third_vals[11], third_vals[15]);
		fourth_vals[15] = complexMult(fourth_vals[15], twiddle_factors[7]);

	fifth_vals[0] = complexAdd(fourth_vals[0],fourth_vals[8]);
	fifth_vals[1] = complexAdd(fourth_vals[1],fourth_vals[9]);
	fifth_vals[2] = complexAdd(fourth_vals[2],fourth_vals[10]);
	fifth_vals[3] = complexAdd(fourth_vals[3],fourth_vals[11]);
	fifth_vals[4] = complexAdd(fourth_vals[4],fourth_vals[12]);
	fifth_vals[5] = complexAdd(fourth_vals[5],fourth_vals[13]);
	fifth_vals[6] = complexAdd(fourth_vals[6],fourth_vals[14]);
	fifth_vals[7] = complexAdd(fourth_vals[7],fourth_vals[15]);
	fifth_vals[8] = complexAdd(fourth_vals[0],fourth_vals[8]);
	fifth_vals[9] = complexAdd(fourth_vals[1],fourth_vals[9]);
	fifth_vals[10] = complexAdd(fourth_vals[2],fourth_vals[10]);
	fifth_vals[11] = complexAdd(fourth_vals[3],fourth_vals[11]);
	fifth_vals[12] = complexAdd(fourth_vals[4],fourth_vals[12]);
	fifth_vals[13] = complexAdd(fourth_vals[5],fourth_vals[13]);
	fifth_vals[14] = complexAdd(fourth_vals[6],fourth_vals[14]);
	fifth_vals[15] = complexAdd(fourth_vals[7],fourth_vals[15]);

	return_vals[0] = cabsf_0(fifth_vals[0]) + iir_inc*return_vals[0];
	return_vals[1] = cabsf_0(fifth_vals[1]) + iir_inc*return_vals[1];
	return_vals[2] = cabsf_0(fifth_vals[2]) + iir_inc*return_vals[2];
	return_vals[3] = cabsf_0(fifth_vals[3]) + iir_inc*return_vals[3];
	return_vals[4] = cabsf_0(fifth_vals[4]) + iir_inc*return_vals[4];
	return_vals[5] = cabsf_0(fifth_vals[5]) + iir_inc*return_vals[5];
	return_vals[6] = cabsf_0(fifth_vals[6]) + iir_inc*return_vals[6];
	return_vals[7] = cabsf_0(fifth_vals[7]) + iir_inc*return_vals[7];
	return_vals[8] = cabsf_0(fifth_vals[8]) + iir_inc*return_vals[8];
	return_vals[9] = cabsf_0(fifth_vals[9]) + iir_inc*return_vals[9];
	return_vals[10] = cabsf_0(fifth_vals[10]) + iir_inc*return_vals[10];
	return_vals[11] = cabsf_0(fifth_vals[11]) + iir_inc*return_vals[11];
	return_vals[12] = cabsf_0(fifth_vals[12]) + iir_inc*return_vals[12];
	return_vals[13] = cabsf_0(fifth_vals[13]) + iir_inc*return_vals[13];
	return_vals[14] = cabsf_0(fifth_vals[14]) + iir_inc*return_vals[14];
	return_vals[15] = cabsf_0(fifth_vals[15]) + iir_inc*return_vals[15];
}