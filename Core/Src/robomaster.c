/*
 * robomaster.c
 *
 *  Created on: May 9, 2025
 *      Author: motii
 */

#include "robomaster.h"

RoboMaster initalizeRoboMaster()
{
	RoboMaster a;
	a.max_current = MAX_CURRENT;
	a.m2006_max_rpm = M2006_RPM;
	a.m3508_max_rpm = M3508_RPM;
	for(int i = 0; i < 8; i++)
	{
		a.ampare[i] = 0;
		a.angle_count[i] = 0;
		a.prev_angle_count[i] = -1;
		a.revolution[i] = 0;
		a.position[i] = 0.0;
		a.rpm[i] = 0;
		a.temp[i] = 0;

		a.buf_1[i] = 0;
		a.buf_2[i] = 0;
		a.pid[i] = pidInitialize(1.0, 0.01, 0.01);

		a.motor_type[i] = M3508;
	}

	return a;
}

void setType(RoboMaster* rm, const uint8_t id, const enum MotorType m_type)
{
	if(id > 8 || id < 1)
	{
		return;
	}

	rm->motor_type[id-1] = m_type;
	return;
}

void setGain(RoboMaster* rm, const uint8_t id, float p_gain, float i_gain, float d_gain)
{
	if(id > 8 || id < 1)
	{
		return;
	}
	rm->pid[id-1] = pidInitialize(p_gain, i_gain, d_gain);
	return;
}

void setTarget(RoboMaster* rm, const uint8_t id, const enum ControlType control_type, const float target)
{
	if(id > 8 || id < 1)
	{
		return;
	}

	const uint8_t index = id-1;
	const float max_rpm = (rm->motor_type[index] == M2006) ? rm->m2006_max_rpm : rm->m3508_max_rpm;
	const float max_current = rm->max_current;

	float pid_out = 0;
	if(control_type == Velocity)
	{
		pid_out = pidCompute(&rm->pid[index], target, rm->rpm[index], 0.02, 0.2);
	}
	else if(control_type == Position)
	{
		float delta_position = target - rm->position[index];
		float rps = delta_position / 0.02;
		float target_rpm = rps * 60;

		if(target_rpm > max_rpm)target_rpm = max_rpm;
		if(target_rpm < (-1.0 * max_rpm))target_rpm = -1.0 * max_rpm;

		pid_out = pidCompute(&rm->pid[index], target_rpm, rm->rpm[index], 0.02, 1.0);
	}

	int16_t target_current = (int16_t)(pid_out * max_current / max_rpm);

	if(target_current > max_current)target_current = max_current;
	if(target_current < (-1.0* max_current))target_current = -1.0 * max_current;

	if(id < 5)
	{
		rm->buf_1[2*id-2] = (target_current >> 8) & 0xFF;
		rm->buf_1[2*id-1] = target_current & 0xFF;
	}
	else
	{
		rm->buf_2[2*id-10] = (target_current >> 8) & 0xFF;
		rm->buf_2[2*id-9] = target_current & 0xFF;
	}
}

void setRecvData(RoboMaster* rm, const uint8_t id, uint8_t* buf)
{
	int16_t angle_data = buf[0] << 8 | buf[1];
	int16_t rpm_data = buf[2] << 8 | buf[3];
	int16_t ampare_data = buf[4] << 8 | buf[5];
	int8_t temp_data = buf[6];

	rm->angle_count[id-1] = angle_data;
	rm->rpm[id-1] = rpm_data;
	rm->ampare[id-1] = ampare_data;
	rm->temp[id-1] = temp_data;

	if(rm->prev_angle_count[id-1] == -1)
	{
		rm->prev_angle_count[id-1] = rm->angle_count[id-1];
	}
	int16_t count_prop = rm->prev_angle_count[id-1] - rm->angle_count[id-1];
	if(count_prop > 4096)
	{
		rm->revolution[id-1] += 1;
	}
	else if(count_prop < -4096)
	{
		rm->revolution[id-1] -= 1;
	}
	rm->prev_angle_count[id-1] = rm->angle_count[id-1];
	rm->position[id-1] = rm->revolution[id-1] + (float)(rm->angle_count[id-1] / 8192.0);
}

void controlRoboMaster(RoboMaster* rm, CAN_HandleTypeDef* can)
{
	static CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = 0x200;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	static uint32_t TxMailBox;

	if(HAL_CAN_GetTxMailboxesFreeLevel(can) > 0)
	{
		HAL_CAN_AddTxMessage(can, &TxHeader, rm->buf_1, &TxMailBox);
	}

	TxHeader.StdId = 0x1FF;
}

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

float pidCompute(PID *pid, float target, float actual, float delta_time, float lpf_alpha)
{
	float prop = target - actual;
	pid->integral += prop * delta_time;
	float derivative = (prop - pid->prev_prop) / delta_time;
	pid->prev_d = pidLowPathFilter(lpf_alpha, derivative, pid->prev_d);
	pidIntegralLimit(pid, 10230, -10230);

	float pid_out = pid->p_gain * prop + pid->i_gain * pid->integral + pid->d_gain * pid->prev_d;

	pid->prev_prop = prop;

	return pid_out;
}

void pidIntegralLimit(PID *pid, float max, float min)
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

float pidLowPathFilter(float alpha, float new_rpm, float prev_rpm)
{
	return alpha * new_rpm + (1.0 - alpha) * prev_rpm;
}
