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

		a.control_type[i] = Velocity;
	}

	return a;
}

void setControlType(RoboMaster* rm, const uint8_t id, const enum ControlType type)
{
	if(id > 8 || id < 1)
	{
		return;
	}

	rm->control_type[id-1] = type;
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

void setTarget(RoboMaster* rm, const uint8_t id, const float target)
{
	if(id > 8 || id < 1)
	{
		return;
	}

	int16_t pid_out = 0;
	if(rm->control_type[id-1] == Velocity)
	{
		pid_out = pidCompute(&rm->pid[id-1], target, rm->rpm[id-1], 0.02);
	}
	else if(rm->control_type[id-1] == Position)
	{
		float delta_position = target - rm->position[id-1];
		float rps = delta_position / 0.2;
		float target_rpm = rps * 60;

		pid_out = pidCompute(&rm->pid[id-1], target_rpm, rm->rpm[id-1], 0.02);
	}

	if(pid_out > 10000)
	{
		pid_out = 10000;
	}
	else if(pid_out < -10000)
	{
		pid_out = -10000;
	}

	if(id < 5)
	{
		rm->buf_1[2*id-2] = (pid_out >> 8) & 0xFF;
		rm->buf_1[2*id-1] = pid_out & 0xFF;
	}
	else
	{
		rm->buf_2[2*id-10] = (pid_out >> 8) & 0xFF;
		rm->buf_2[2*id-9] = pid_out & 0xFF;
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
