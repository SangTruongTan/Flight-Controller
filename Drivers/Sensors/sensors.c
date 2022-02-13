/*
File:                sensors.c
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

/* Private includes ----------------------------------------------------------*/
#include "sensors.h"

/* Private variables ---------------------------------------------------------*/
struct sensor_handle_t senHandler;

/* Function definations ------------------------------------------------------*/
/**
  * @brief  Initialize sensors.
  * @param init the initial condition for sensors.
  * @retval sensor_handle_t
  */
sensor_handle_t sensors_init (sensor_handle_t init) {
    //Init the mpu sensor
    senHandler.mpuHandler = init.mpuHandler;
    senHandler.enableMPU = init.enableMPU;
    if (senHandler.enableMPU == true) {
      MPU6050_Init(senHandler.mpuHandler.Init, senHandler.mpuHandler.hi2c);
    }
    return senHandler;
}

/**
  * @brief  Update sensors data. Must be call as interval with the dt value.
  * @retval sensor_handle_t
  */
sensor_handle_t sensors_update (void) {
    if (senHandler.enableMPU == true) {
      senHandler.mpuHandler = MPU6050_read_mpu_data();
    }
    return senHandler;
}
