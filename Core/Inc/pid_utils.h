/*
 * pid_utils.h
 *
 *  Created on: May 6, 2025
 *      Author: motii
 */

#ifndef INC_PID_UTILS_H_
#define INC_PID_UTILS_H_


#include <stdint.h>

#define MAX_RPM 9500
#define MAX_CURRENT 10000


typedef struct PID
{
	float p_gain;
	float i_gain;
	float d_gain;
	float integral;
	float prev_prop;
	float prev_d;
}PID;

PID pidInitialize(float p_g, float i_g, float d_g);
int16_t pidCompute(PID *pid, float target, float actual, float delta_time);
void integralLimit(PID *pid, float max, float min);
int LowPathFilter(float alpha, int new_rpm ,int prev_rpm);

#endif /* INC_PID_UTILS_H_ */
