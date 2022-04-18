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
    Handle->enableHMC = false;
    Handle->enableMPU = true;
    Handle->enableMS = false;
    Handle->enableGps = true;
    // Init for the MPU
    Handle->mpuHandler.Init.ui8AcceFullScale = MPU6050_ACCE_FULLSCALE_8G;
    Handle->mpuHandler.Init.ui8GyroFullScale = MPU6050_GYRO_FULLSCALE_500DPS;
    Handle->mpuHandler.Init.ui8DLPF = MPU6050_DLPF_3;
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
    // Calculate Battery Voltage
    Handle->Angle.VBat = (*Handle->AdcBat) * 36.3 / 4016;
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
    MPU6050_GyroAxis *Axis = &Handle->mpuHandler.GyroAxis;
    MPU6050_AcceAxis *Acc = &Handle->mpuHandler.AcceAxis;
    MPU6050_GyroAxis *Adj = &Handle->mpuHandler.AngleAjust;
    // Get the mpu data
    if (Handle->enableMPU == true) {
        MPU6050_State_t State = MPU6050_read_mpu_data(&Handle->mpuHandler);
        if (State != MPU6050_OK_STATE) {
            *SenStatus |= SENSOR_ERROR_MPU;
        } else {
            // Put the compass heading here
            // Complementary filter
            Axis->pitch = 0.9996 * Axis->pitch + 0.0004 * Acc->pitch;
            Axis->roll = 0.9996 * Axis->roll + 0.0004 * Acc->roll;
            // Calculate the angle with the adjustment
            Adj->pitch = Axis->pitch * 15;
            Adj->roll = Axis->roll * 15;
            //Assign for sensors Parameter
            Handle->Angle.Pitch = Axis->pitch;
            Handle->Angle.Roll = Axis->roll;
            Handle->Angle.Yaw = Axis->yaw;
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
    // Calculate Battery Voltage
    Handle->Angle.VBat =
        0.92 * Handle->Angle.VBat + 0.08 * (*Handle->AdcBat) * 36.3 / 4016;
    if (*SenStatus == SENSOR_OK) {
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
    if (Handler->enableGps == true) {
        GpsStatus_t status = Gps_Process(&Handler->gpsHandler);
        Handler->gpsHandler.Status = status;
        if (status != GPS_OK) {
            retval |= SENSOR_ERROR_GPS;
        }
    }
    return retval;
}

void Sensor_Gyro_Calibration(Sensor_handle_t *Handler) {
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    uint32_t TickStart;
    for (int i = 0; i < 2000; i++) {
        TickStart = HAL_GetTick();
        MPU6050_GyroRead_Raw(&Handler->mpuHandler);
        x += Handler->mpuHandler.GyroRaw.x;
        y += Handler->mpuHandler.GyroRaw.y;
        z += Handler->mpuHandler.GyroRaw.z;
        if (i % 25 == 0) HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        while (HAL_GetTick() < TickStart + 3)
            ;
    }
    Handler->mpuHandler.GyroOffset.x = x / 2000;
    Handler->mpuHandler.GyroOffset.y = y / 2000;
    Handler->mpuHandler.GyroOffset.z = z / 2000;
}
