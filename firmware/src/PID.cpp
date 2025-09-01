#include "PID.hpp"
#include "usart.h"
#include <cstdio>
#include <string.h>
 char usr_buf[32];
//char usr_buf[256];
float PID::calc(float target,float feedback){
	error = target - feedback;
	float p = error * kp;

	float error_sum_old = error_sum;
	error_sum += error;
	float i = error_sum * ki;

	float d = (error - old_error) * kd;
	old_error = error;

	float out = p+i+d;
	printf("out: %d\n", (int)error);


	if(limit_enable){
		if(out < out_min){
			out = out_min;
			error_sum = error_sum_old;
		}
		if(out_max < out){
			out = out_max;
			error_sum = error_sum_old;
		}
	}
	return out;
}