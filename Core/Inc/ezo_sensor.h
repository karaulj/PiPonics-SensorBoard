/*
 * ezo_sensor.h
 *
 *  Created on: Mar 3, 2021
 *      Author: Jacob Karaul
 */

#ifndef INC_EZO_SENSOR_H_
#define INC_EZO_SENSOR_H_

#include <stdint.h>

/* EZO RTD Temperature Sensor constants */
#define RTD_I2C_ADDR_DEFAULT			(uint16_t) 0x66
#define RTD_I2C_BUFFER_MIN_SZ			(uint8_t) 4
#define RTD_I2C_BUFFER_MAX_SZ			(uint8_t) 40				// from datasheet
#define RTD_I2C_READ_BUFFER_MAX_SZ		(uint8_t) 8					// max temp is 1254 C, plus decimal and 0.001 resolution
#define RTD_I2C_LONG_DELAY_MS			600U
#define RTD_I2C_SHORT_DELAY_MS			300U


extern const char* RTD_I2C_Cmd_Read;								// I2C reading command
extern uint8_t RTD_I2C_resBuf[RTD_I2C_BUFFER_MAX_SZ];				// use as I2C response buffer

typedef enum {
	NoData 			= 255,
	StillProcessing	= 254,
	SyntaxErr		= 2,
	Success			= 1
} RTD_i2cResCode;


double getRTDReading(uint8_t* buf);

#endif
