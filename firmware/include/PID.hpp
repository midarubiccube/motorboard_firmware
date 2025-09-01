#pragma once

class PID{
private:
	const bool limit_enable;
	const float pid_freq;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float error = 0;
	float error_sum = 0;
	float old_error = 0;

	//output limit
	float out_min = 0;
	float out_max = 0;
public:
	PID(bool limit,float _pid_freq):limit_enable(limit),pid_freq(_pid_freq){}
	float calc(float target,float feedback);

	//inline functions
	void set_gain(float _kp,float _ki,float _kd){
		kp = _kp;

		ki = _ki/pid_freq;
		kd = _kd*pid_freq;
	}
	void set_limit(float min,float max){
		out_min = min;
		out_max = max;
	}
	void reset(void){
		error = 0;
		error_sum = 0;
		old_error = 0;
	}
};