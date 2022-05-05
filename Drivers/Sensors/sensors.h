/*
File:                sensors.h
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

#ifndef _SENSORS_H__
#define _SENSORS_H__

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>

#include "HMC5883.h"
#include "MS5611.h"
#include "ST_MPU6050.h"
#include "UbloxGps.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum Sensor_status_t {
    SENSOR_OK = 0U,
    SENSOR_ERROR_MPU = 0x01U,
    SENSOR_ERROR_HMC = 0x02U,
    SENSOR_ERROR_MS = 0x04U,
    SENSOR_ERROR_GPS = 0x08U,
} Sensor_status_t;

typedef struct SensorParameters_t {
    float Roll;
    float Pitch;
    float Yaw;
    float VBat;
    float Altitude;
    GpsData_t Gps;
} SensorParameters_t;

typedef struct Sensor_Calibration_t {
    void (*wait)(uint32_t);
    uint32_t (*GetTime)(void);
    void (*waitUntil)(uint32_t *, uint32_t);
    uint32_t StartTask;
    uint16_t *ThrustChannel;
} Sensor_Calibration_t;

typedef struct Sensor_handle_t {
    MPU6050_handle_t mpuHandler;
    HMC5883L_Handle_t hmcHandler;
    MS5611_Handle_t msHandler;
    GpsHandler_t gpsHandler;
    SensorParameters_t Angle;
    I2C_HandleTypeDef *mshi2c;
    I2C_HandleTypeDef *mpuhi2c;
    I2C_HandleTypeDef *hmchi2c;
    RingHandler_t *gpsRing;
    Sensor_Calibration_t Calibration;
    bool enableHMC;
    bool enableMPU;
    bool enableMS;
    bool enableGps;
    Sensor_status_t status;
    uint16_t *AdcBat;
} Sensor_handle_t;

/* Private definations -------------------------------------------------------*/
#define dt 0.004f

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief This function for user modify the initializations.
 * @param Handle The pointer of the handle type's sensor.
 * @param void
 */
void sensors_user_modify(Sensor_handle_t *Handle);

/**
 * @brief  Initialize sensors.
 * @param Handle The pointer of the handle type's sensor.
 * @retval Sensor_status_t
 */
Sensor_status_t sensors_init(Sensor_handle_t *Handle);

/**
 * @brief  Update sensors data. Must be call as interval with the dt value.
 * @param Handle The pointer of the handle type's sensor.
 * @retval Sensor_status_t
 */
Sensor_status_t sensors_update(Sensor_handle_t *Handle);

Sensor_status_t Sensor_Gps_Update(Sensor_handle_t *Handler);

void Sensor_Gyro_Calibration(Sensor_handle_t *Handler);
void Sensor_Compass_Calibration(Sensor_handle_t *Handler);
float course_deviation(float course_b, float course_c);

#endif /* End of File */
