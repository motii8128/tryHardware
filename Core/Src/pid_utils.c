/*
 * pid_utils.c
 *
 *  Created on: May 9, 2025
 *      Author: motii
 */

#include "pid_utils.h"

PID pidInitialize(float p_g, float i_g, float d_g)
{
	PID pid;
	pid.p_gain = p_g;
	pid.i_gain = i_g;
	pid.d_gain = d_g;
	pid.integral = 0.0;
	pid.prev_prop = 0.0;
	pid.prev_d = 0.0;

	return pid;
}

int16_t pidCompute(PID *pid, float target, float actual, float delta_time)
{
	float prop = target - actual;
	pid->integral += prop * delta_time;
	float derivative = (prop - pid->prev_prop) / delta_time;
	pid->prev_d = LowPathFilter(0.2, derivative, pid->prev_d);
	integralLimit(pid, 10230, -10230);
	float pid_out = pid->p_gain * prop + pid->i_gain * pid->integral + pid->d_gain * pid->prev_d;
	pid->prev_prop = prop;

	int16_t out_current = (int16_t)pid_out * MAX_CURRENT / MAX_RPM;


	return out_current;
}

void integralLimit(PID *pid, float max, float min)
{
	if(pid->integral > max)
	{
		pid->integral = max;
	}
	else if(pid->integral < min)
	{
		pid->integral = min;
	}
}

int LowPathFilter(float alpha, int new_rpm ,int prev_rpm)
{
	return alpha * new_rpm + (1.0 - alpha) * prev_rpm;
}
