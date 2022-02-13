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

#ifndef SENSORS_H__
#define SENSORS_H__

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "ST_MPU6050.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct sensor_handle_t {
    MPU6050_handle_t mpuHandler;
    bool enableMPU;
} sensor_handle_t;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Initialize sensors.
  * @param init the initial condition for sensors.
  * @retval sensor_handle_t
  */
sensor_handle_t sensors_init (sensor_handle_t init);
/**
  * @brief  Update sensors data. Must be call as interval with the dt value.
  * @retval sensor_handle_t
  */
sensor_handle_t sensors_update (void);
#endif