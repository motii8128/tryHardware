/*
 * robomaster_util.h
 *
 *  Created on: May 6, 2025
 *      Author: motii
 */

#ifndef INC_ROBOMASTER_UTILS_H_
#define INC_ROBOMASTER_UTILS_H_

#include <stdint.h>

typedef struct RoboMasterCmd
{
	uint8_t buf_1[8];
	uint8_t buf_2[8];
}RoboMasterCmd;

typedef struct RoboMasterFeedBack
{
	int16_t angle[8];
	int16_t rpm[8];
	int16_t ampare[8];
	int8_t temp[8];
}RoboMasterFeedBack;

void setCurrent(int id, int16_t current, RoboMasterCmd* packet)
{
	int16_t target_current = 0;
	if(current > 10000)
	{
		target_current = 10000;
	}
	else if(current < -10000)
	{
		target_current = -10000;
	}
	else
	{
		target_current = current;
	}

	if(id < 5)
	{
		packet->buf_1[2*id-2] = (target_current >> 8) & 0xFF;
		packet->buf_1[2*id-1] = target_current & 0xFF;
	}
	else
	{
		packet->buf_2[2*id-10] = (target_current >> 8) & 0xFF;
		packet->buf_2[2*id-9] = target_current & 0xFF;
	}
}

void parseRoboMasterFeedBack(uint32_t id, uint8_t *buf, RoboMasterFeedBack *rm_fb)
{
	int16_t angle_data = buf[0] << 8 | buf[1];
	int16_t rpm_data = buf[2] << 8 | buf[3];
	int16_t ampare_data = buf[4] << 8 | buf[5];
	int8_t temp_data = buf[6];

	rm_fb->angle[id-1] = ((double)angle_data/ 8192.0) * 360;
	rm_fb->rpm[id-1] = rpm_data;
	rm_fb->ampare[id-1] = ampare_data;
	rm_fb->temp[id-1] = temp_data;
}

#endif /* INC_ROBOMASTER_UTILS_H_ */
