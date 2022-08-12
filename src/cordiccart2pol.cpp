#include "cordiccart2pol.h"
#include <math.h>
#include <stdio.h>

/*data_t cordic_phase[NO_ITER] = {
		45,
		25.56,
		14.036,
		7.125,
	    3.576,
		1.790,
		0.895,
		0.4475,
		0.22375,
		0.111875,
		0.0559375,
		0.02796875,
		0.013984375,
		0.0069921875
};*/

data_t cordic_phase[NO_ITER] = {
		0.785398163397448,
		0.463647609000806,
		0.244978663126864,
		0.124354994546761,
		0.0624188099959574,
		0.0312398334302683,
		0.0156237286204768,
		0.00781234106010111,
		0.00390623013196697,
		0.00195312251647882,
		0.000976562189559320,
		0.000488281211194898,
		0.000244140620149362,
		0.000122070311893670,
		6.10351561742088e-05,
		3.05175781155261e-05
};

const data_t pi = 3.1415926535897932384626;


void cordiccart2pol(data_t x, data_t y, data_t *r,  data_t * theta)
{
#pragma HLS INTERFACE s_axilite port=x bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y bundle=CTRL
//#pragma HLS INTERFACE m_axi port=r offset=slave depth=1
//#pragma HLS INTERFACE m_axi port=theta offset=slave depth=1
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta bundle=CTRL

	ap_fixed<32,2> current_x = 0;
	ap_fixed<32,2> current_y = 0;
	data_t temp_theta = 0;

	//roate 90
	if(y > 0){
		current_x = y;
		current_y = -x;
		temp_theta = pi/2;
	}else {
		current_x = -y;
		current_y = x;
		temp_theta = -pi/2;
	}


ROTATE:
	for(int j=0; j< NO_ITER; j++){
#pragma HLS UNROLL
		ap_fixed<32,2> x_shift = current_x >> j;
		ap_fixed<32,2> y_shift = current_y >> j;

		if(current_y > 0){
			current_x = current_x + y_shift;
			current_y = current_y - x_shift;
			temp_theta += cordic_phase[j];
		}else{
			current_x = current_x - y_shift;
			current_y = current_y + x_shift;
			temp_theta -= cordic_phase[j];
		}
	}

	*r = float(current_x) * 0.60735;
	*theta = temp_theta;
}
