/*
 * robomaster.hpp
 *
 *  Created on: May 9, 2025
 *      Author: motii
 */

#ifndef INC_ROBOMASTER_H_
#define INC_ROBOMASTER_H_

#include <stdint.h>
#include <math.h>
#include "stm32l4xx_hal.h"

#define MAX_CURRENT 10000
#define M2006_RPM 13000
#define M3508_RPM 9000

enum ControlType
{
	Position,
	Velocity
};

enum MotorType
{
	M3508,
	M2006
};

typedef struct PID
{
	float p_gain;
	float i_gain;
	float d_gain;
	float integral;
	float prev_prop;
	float prev_d;
}PID;

typedef struct
{
	uint8_t buf_1[8]; /** 送信データ ID1~4 **/
	uint8_t buf_2[8];

	int16_t angle_count[8];
	int16_t prev_angle_count[8];
	int16_t revolution[8];
	float position[8];
	int16_t rpm[8];
	int16_t ampare[8];
	int8_t temp[8];

	PID pid[8];
	enum MotorType motor_type[8];

	float max_current;
	float m2006_max_rpm;
	float m3508_max_rpm;
}RoboMaster;

RoboMaster initalizeRoboMaster();
void setType(RoboMaster* rm, const uint8_t id, const enum MotorType m_type);
void setGain(RoboMaster* rm, const uint8_t id, float p_gain, float i_gain, float d_gain);
void setTarget(RoboMaster* rm, const uint8_t id, const enum ControlType control_type, const float target);
void setRecvData(RoboMaster* rm, const uint8_t id, uint8_t* rx_buf);
void controlRoboMaster(RoboMaster* rm, CAN_HandleTypeDef* can);

PID pidInitialize(float p_g, float i_g, float d_g);
float pidCompute(PID *pid, float target, float actual, float delta_time);
void pidIntegralLimit(PID *pid, float max, float min);
float pidLowPathFilter(float alpha, float new_rpm, float prev_rpm);

#endif /* INC_ROBOMASTER_H_ */
