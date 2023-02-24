/*
 * flash.h
 *
 *  Created on: Feb 23, 2023
 *      Author: bockn
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdio.h>
#include "w25qxx.h"

typedef struct LogMessage {
	uint32_t time_ms;
	float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
	uint8_t checksum;
} LogMessage;

void LogInit();
void LogFlightSetup();
void LogRead();
void LogWrite();
void LogData(uint32_t time_ms,
		float accel_x, float accel_y, float accel_z,
		float gyro_x, float gyro_y, float gyro_z);
void LogWipe();

#endif /* INC_FLASH_H_ */
