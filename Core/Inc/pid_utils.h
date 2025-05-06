/*
 * pid_utils.h
 *
 *  Created on: May 6, 2025
 *      Author: motii
 */

#ifndef INC_PID_UTILS_H_
#define INC_PID_UTILS_H_


#include <stdint.h>

#define MAX_RPM 10000
#define MAX_CURRENT 10000

int LowPathFilter(float alpha, int new_rpm ,int prev_rpm)
{
	return alpha * new_rpm + (1.0 - alpha) * prev_rpm;
}


typedef struct PID
{
	float p_gain;
	float i_gain;
	float d_gain;
	float integral;
	float prev_prop;
}PID;

PID pidInitialize(float p_g, float i_g, float d_g);
int16_t pidCompute(PID *pid, int16_t target, int16_t actual, float delta_time);
void integralLimit(PID *pid, float max, float min);



PID pidInitialize(float p_g, float i_g, float d_g)
{
	PID pid;
	pid.p_gain = p_g;
	pid.i_gain = i_g;
	pid.d_gain = d_g;
	pid.integral = 0.0;
	pid.prev_prop = 0.0;

	return pid;
}

int16_t pidCompute(PID *pid, int16_t target, int16_t actual, float delta_time)
{
	float prop = target - actual;
	pid->integral += prop * delta_time;
	float derivative = (prop - pid->prev_prop) / delta_time;
	integralLimit(pid, 10230, -10230);
	float pid_out = pid->p_gain * prop + pid->i_gain * pid->integral + pid->d_gain * derivative;
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


#endif /* INC_PID_UTILS_H_ */
