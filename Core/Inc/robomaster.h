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
#include "pid_utils.h"
#include "main.h"

enum ControlType
{
	Position,
	Velocity
};

typedef struct
{
	uint8_t buf_1[8];
	uint8_t buf_2[8];

	int16_t angle_count[8];
	int16_t prev_angle_count[8];
	int16_t revolution[8];
	float position[8];
	int16_t rpm[8];
	int16_t ampare[8];
	int8_t temp[8];

	PID pid[8];
	enum ControlType control_type[8];
}RoboMaster;

RoboMaster initalizeRoboMaster();
void setControlType(RoboMaster* rm, const uint8_t id, const enum ControlType type);
void setGain(RoboMaster* rm, const uint8_t id, float p_gain, float i_gain, float d_gain);
void setTarget(RoboMaster* rm, const uint8_t id, const float target);
void setRecvData(RoboMaster* rm, const uint8_t id, uint8_t* rx_buf);
void controlRoboMaster(RoboMaster* rm, CAN_HandleTypeDef* can);

#endif /* INC_ROBOMASTER_H_ */
