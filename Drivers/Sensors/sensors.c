/*
File:                sensors.c
Written by:             Sang Truong Tan
Date Written:           02/12/2022
Description:
References:

*Copyright (C) 2022 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or
modify it under the terms of the GNU General Public Licenseversion 3 as
published by the Free Software Foundation.

   This software library is shared with public for educational purposes, without
WARRANTY and Author is not liable for any damages caused directly or indirectly
by this software, read more about this on the GNU General Public License.
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
void sensors_user_modify(Sensor_handle_t *Handle) {
    /* USER CODE BEGIN */
    // Enable the sensors
    Handle->enableHMC = true;
    Handle->enableMPU = true;
    Handle->enableMS = true;
    Handle->enableGps = true;
    // Init for the MPU
    Handle->mpuHandler.Init.ui8AcceFullScale = MPU6050_ACCE_FULLSCALE_2G;
    Handle->mpuHandler.Init.ui8GyroFullScale = MPU6050_GYRO_FULLSCALE_500DPS;
    Handle->mpuHandler.Init.ui8DLPF = MPU6050_DLPF_4;
    // Init for the HMC
    Handle->hmcHandler.Init.Average = HMC5883L_AVERAGING_8;
    Handle->hmcHandler.Init.Bias = HMC5883L_BIAS_NORMAL;
    Handle->hmcHandler.Init.Gain = HMC5883L_GAIN_1090;
    Handle->hmcHandler.Init.Mode = HMC5883L_MODE_CONTINUOUS;
    Handle->hmcHandler.Init.OutputRate = HMC5883L_RATE_75;

    // Init for the presure sensor
    Handle->msHandler.Osr = MS5611_ULTRA_HIGH_RES;

    // Init for the GPS Module
    Handle->gpsHandler.Serial = Handle->gpsRing;
    /* USER CODE END */
}

/**
 * @brief  Initialize sensors.
 * @param Handle The pointer of the handle type's sensor.
 * @retval Sensor_status_t
 */
Sensor_status_t sensors_init(Sensor_handle_t *Handle) {
    // Initialize
    Sensor_status_t *SenStatus = &Handle->status;
    *SenStatus = SENSOR_OK;
    MPU6050_State_t mpuState;
    HMC5883L_State_t hmcState;
    MS5611_Status_t msStatus;
    // Init the sensors
    sensors_user_modify(Handle);
    // Call the mpu init function
    if (Handle->enableMPU == true) {
        mpuState = MPU6050_Init(&Handle->mpuHandler, Handle->mpuhi2c);
        if (mpuState != MPU6050_OK_STATE) {
            *SenStatus |= SENSOR_ERROR_MPU;
        }
    }
    // Call the hmc init function
    if (Handle->enableHMC == true) {
        hmcState = HMC5883L_Init(&Handle->hmcHandler, Handle->hmchi2c);
        if (hmcState != HMC5883L_OK_STATE) {
            *SenStatus |= SENSOR_ERROR_HMC;
        }
    }
    // Call the ms5611 init function
    if (Handle->enableMS == true) {
        msStatus = MS5611_Init(&Handle->msHandler, Handle->mshi2c);
        if (msStatus != MS5611_OK) {
            *SenStatus |= SENSOR_ERROR_MS;
        }
    }
    // Call the Gps module Init function

    return *SenStatus;
}

/**
 * @brief  Update sensors data. Must be call as interval with the dt value.
 * @param Handle The pointer of the handle type's sensor.
 * @retval Sensor_status_t
 */
Sensor_status_t sensors_update(Sensor_handle_t *Handle) {
    // Initialize
    Sensor_status_t *SenStatus = &Handle->status;
    *SenStatus = SENSOR_OK;
    // Get the mpu data
    if (Handle->enableMPU == true) {
        MPU6050_State_t State = MPU6050_read_mpu_data(&Handle->mpuHandler);
        if (State != MPU6050_OK_STATE) {
            *SenStatus |= SENSOR_ERROR_MPU;
        }
    }
    // Get the hmc data
    if (Handle->enableHMC == true) {
        HMC5883L_Status_t status = HMC5883L_Get_Scaled(&Handle->hmcHandler);
        if (status != HMC5883L_OK) *SenStatus |= SENSOR_ERROR_HMC;
    }
    // Get the pressure sensor data
    if (Handle->enableMS == true) {
        MS5611_Status_t status;
        status = MS5611_Update(&Handle->msHandler);
        if (status != MS5611_OK) *SenStatus |= SENSOR_ERROR_MS;
    }
    return SENSOR_OK;
}

/**
 * @brief Update data for the Gps Module. Must be called with below 500ms
 * period.
 * @param Handler The pointer of the sensors handler.
 * @retval Sensor_status_t
 */
Sensor_status_t Sensor_Gps_Update(Sensor_handle_t *Handler) {
    Sensor_status_t retval = SENSOR_OK;
    if(Handler->enableGps == true){
        GpsStatus_t status = Gps_Process(&Handler->gpsHandler);
        Handler->gpsHandler.Status = status;
        if(status != GPS_OK) {
            retval |= SENSOR_ERROR_GPS;
        }
    }
    return retval;
}
