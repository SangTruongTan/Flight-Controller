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

/* Function definations ------------------------------------------------------*/
/**
 * @brief This function for user modify the initializations.
 * @param Handle The pointer of the handle type's sensor.
 * @param void
 */
void sensors_user_modify (Sensor_handle_t *Handle) {
  /* USER CODE BEGIN */
  //Enable the sensors
  Handle->enableHMC = true;
  Handle->enableMPU = false;
  //Init for the MPU
  Handle->mpuHandler.Init.ui8AcceFullScale = MPU6050_ACCE_FULLSCALE_2G;
  Handle->mpuHandler.Init.ui8GyroFullScale = MPU6050_GYRO_FULLSCALE_500DPS;
  Handle->mpuHandler.Init.ui8DLPF = MPU6050_DLPF_4;
  //Init for the HMC
  Handle->hmcHandler.Init.Average = HMC5883L_AVERAGING_8;
  Handle->hmcHandler.Init.Bias = HMC5883L_BIAS_NORMAL;
  Handle->hmcHandler.Init.Gain = HMC5883L_GAIN_1090;
  Handle->hmcHandler.Init.Mode = HMC5883L_MODE_CONTINUOUS;
  Handle->hmcHandler.Init.OutputRate = HMC5883L_RATE_75;
  /* USER CODE END */
}

/**
  * @brief  Initialize sensors.
  * @param Handle The pointer of the handle type's sensor.
  * @retval Sensor_status_t
  */
Sensor_status_t sensors_init (Sensor_handle_t *Handle) {
  //Declare the variables
  Sensor_status_t senStatus = SENSOR_OK;
  MPU6050_State mpuState;
  HMC5883L_State_t hmcState;
  //Init the sensors
  sensors_user_modify(Handle);
  //Call the mpu init function
  if(Handle->enableMPU == true) {
    mpuState = MPU6050_Init(Handle->mpuHandler.Init, Handle->mpuhi2c);
    if(mpuState != MPU6050_OK) {
      senStatus = SENSOR_ERROR_MPU;
    }
  }
  //Call the hmc init function
  if(Handle->enableHMC == true) {
    hmcState = HMC5883L_Init(&Handle->hmcHandler, Handle->hmchi2c);
    Handle->hmcHandler.State = hmcState;
    if(hmcState != HMC5883L_OK_STATE) {
      if(senStatus != SENSOR_OK) {
        senStatus = SENSOR_ERROR_BOTH;
      } else {
        senStatus = SENSOR_ERROR_HMC;
      }
    }
  }
  return senStatus;
}

/**
  * @brief  Update sensors data. Must be call as interval with the dt value.
  * @param Handle The pointer of the handle type's sensor.
  * @retval Sensor_status_t
  */
Sensor_status_t sensors_update (Sensor_handle_t *Handle) {
  //Get the mpu data
  //Get the hmc data
  if(Handle->enableHMC == true) {
    HMC5883L_Status_t status = HMC5883L_Get_Scaled(&Handle->hmcHandler);
    if(status  != HMC5883L_OK)
      return SENSOR_ERROR_HMC;
  }
  return SENSOR_OK;
}
