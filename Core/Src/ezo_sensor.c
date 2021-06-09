/*
 * ezo_sensor.c
 *
 *  Created on: Mar 3, 2021
 *      Author: Jacob Karaul
 */

#include <stdlib.h>
#include "ezo_sensor.h"


uint8_t RTD_I2C_resBuf[RTD_I2C_BUFFER_MAX_SZ] = {'\0'};

const char* RTD_I2C_Cmd_Read = "R\0";


/**
 *  Returns EZO RTD sensor reading as a double from ASCII string in response buffer.
 *	@param buf	The I2C response buffer from the Read command. Typically RTD_I2C_resBuf
 */
double getRTDReading(uint8_t* buf)
{
	uint8_t readingArr[RTD_I2C_READ_BUFFER_MAX_SZ] = {'0'};
	uint8_t readingArrIdx = 0, bufIdx = 0;
	while (buf[bufIdx] != '\0')
	{
		if ((buf[bufIdx] >= '0' && buf[bufIdx] <= '9') || buf[bufIdx] == '-' || buf[bufIdx] == '.')
		{
			readingArr[readingArrIdx++] = buf[bufIdx];
		}
		bufIdx++;
	}
	double res = atof((const char*)readingArr);
	return res;
}
