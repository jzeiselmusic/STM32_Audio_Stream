#include "lpf.h"

#include "arm_math_types.h"
/* cmsis dsp includes */
#include "basic_math_functions.h"
#include "support_functions.h"

static float32_t input_list[BUFF_SIZE + 1];
static float32_t output_list[BUFF_SIZE + 1];

// butterworth filter with N= 5 and cutoff = .5
static float32_t b[6] = {0.052786, 0.263932, 0.527864, 0.263932, 0.052786};
static float32_t a[5] = {0, .63344, 0, .055728, 0};

float32_t low_pass_filter(float32_t left) {

    /* declare static variable lpf count */
    static int lpf_count = 0;

	int i;
	float32_t return_value;

	for (i = 0; i < BUFF_SIZE; i++) {
		input_list[BUFF_SIZE - i] = input_list[BUFF_SIZE - i - 1];
		output_list[BUFF_SIZE - i] = output_list[BUFF_SIZE - i - 1];
	}
	input_list[0] = left;
    float32_t temp_output_list[BUFF_SIZE];

    // this is where things get wonky
    for (i = 0; i < BUFF_SIZE; i++) {
    	temp_output_list[i] = output_list[i+1];
    }
	if (lpf_count < 16) {
		output_list[0] = left;
		return_value = output_list[0];
		lpf_count++;
	}
	else {
		float32_t result_fir;
		float32_t result_iir;
		arm_dot_prod_f32(input_list, b, BUFF_SIZE + 1, &result_fir);
		arm_dot_prod_f32(temp_output_list, a, BUFF_SIZE, &result_iir);
		output_list[0] = result_fir - result_iir;
		return_value = output_list[0];
	}

	return return_value;
}