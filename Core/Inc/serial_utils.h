/*
 * serial_utils.h
 *
 *  Created on: May 6, 2025
 *      Author: motii
 */

#ifndef INC_SERIAL_UTILS_H_
#define INC_SERIAL_UTILS_H_

#include "stm32l4xx_hal.h"
#include <stdlib.h>

typedef struct
{
	int pwm_1;
	int pwm_2;
	int pwm_3;

	float rpm_1;
	float rpm_2;
	float rpm_3;
}ReceiveValue;

ReceiveValue newValue()
{
	ReceiveValue a = {0,0,0,0.0,0.0,0.0};

	return a;
}

ReceiveValue SerialRead(UART_HandleTypeDef *huart)
{
	int index = 0;
	uint8_t buf[10];

	while(1)
	{
		if(HAL_UART_Receive(huart, &buf[index], 1, 1000) == HAL_OK)
		{
			if(buf[index] == '\n')
			{
				ReceiveValue value = newValue();

				float rate = (buf[0] - 127.0) / 127.0;
				value.pwm_1 = rate * 1000;

				rate = (buf[1] - 127.0) / 127.0;
				value.pwm_2 = rate * 1000;

				rate = (buf[2] - 127.0) / 127.0;
				value.pwm_3 = rate * 1000;

				rate = (buf[3] - 127.0) / 127.0;
				value.rpm_1 = rate * 100.0;

				rate = (buf[4] - 127.0) / 127.0;
				value.rpm_2 = rate * 10000.0;

				rate = (buf[5] - 127.0) / 127.0;
				value.rpm_3 = rate * 10000;

				if(index < 5)
				{
					return newValue();
				}

				return value;
			}
			else
			{
				index++;
				if(index == 10)
				{
					return newValue();
				}
			}
		}
	}
}


#endif /* INC_SERIAL_UTILS_H_ */
