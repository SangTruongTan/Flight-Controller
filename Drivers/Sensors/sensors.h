/*
File:                sensors.h
Written by:             Sang Truong Tan
Date Written:           02/12/2022
Description:
References:

*Copyright (C) 2022 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#ifndef _SENSORS_H__
#define _SENSORS_H__

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "ST_MPU6050.h"
#include "HMC5883.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum Sensor_status_t {
  SENSOR_OK = 0,
  SENSOR_ERROR_MPU,
  SENSOR_ERROR_HMC,
  SENSOR_ERROR_BOTH,
} Sensor_status_t;

typedef struct Sensor_handle_t {
    MPU6050_handle_t mpuHandler;
    HMC5883L_Handle_t hmcHandler;
    I2C_HandleTypeDef *mpuhi2c;
    I2C_HandleTypeDef *hmchi2c;
    bool enableHMC;
    bool enableMPU;
} Sensor_handle_t;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief This function for user modify the initializations.
 * @param Handle The pointer of the handle type's sensor.
 * @param void
 */
void sensors_user_modify (Sensor_handle_t *Handle);

/**
  * @brief  Initialize sensors.
  * @param Handle The pointer of the handle type's sensor.
  * @retval Sensor_status_t
  */
Sensor_status_t sensors_init (Sensor_handle_t *Handle);

/**
  * @brief  Update sensors data. Must be call as interval with the dt value.
  * @param Handle The pointer of the handle type's sensor.
  * @retval Sensor_status_t
  */
Sensor_status_t sensors_update (Sensor_handle_t *Handle);

#endif    /* End of File */
