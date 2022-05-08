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

/* Private defines -----------------------------------------------------------*/
#define PHI_DIV_180 0.0174533

/* Private variables ---------------------------------------------------------*/

/* Private functions declaration
 * ----------------------------------------------*/
void Sensor_compass_calculate_offset(HMC5883L_Offset_Scale_t *OffsetScale);

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
    Handle->enableMS = false;
    Handle->enableGps = false;
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
        Sensor_compass_calculate_offset(&Handle->hmcHandler.OffsetScale);
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
    if (Handle->status == SENSOR_OK) {
        sensors_update(Handle);
        Handle->mpuHandler.GyroAxis.yaw =
            Handle->hmcHandler.CompassData.actual_heading;
    }
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
            // Assign for sensors Parameter
            Handle->Angle.Pitch = Axis->pitch;
            Handle->Angle.Roll = Axis->roll;
            Handle->Angle.Yaw = Axis->yaw;
        }
    }
    // Get the hmc data
    if (Handle->enableHMC == true) {
        HMC5883L_Status_t status = HMC5883L_Update(&Handle->hmcHandler);
        if (status != HMC5883L_OK) *SenStatus |= SENSOR_ERROR_HMC;
        // Calcute horizontal compass value
        Handle->hmcHandler.CompassData.x_horizontal =
            (float)Handle->hmcHandler.Raw.x *
                cos(Handle->Angle.Pitch * -PHI_DIV_180) +
            (float)Handle->hmcHandler.Raw.y *
                sin(Handle->Angle.Roll * PHI_DIV_180) *
                sin(Handle->Angle.Pitch * -PHI_DIV_180) -
            (float)Handle->hmcHandler.Raw.z *
                cos(Handle->Angle.Roll * PHI_DIV_180) *
                sin(Handle->Angle.Pitch * -PHI_DIV_180);
        Handle->hmcHandler.CompassData.y_horizontal =
            (float)Handle->hmcHandler.Raw.y *
                cos(Handle->Angle.Roll * PHI_DIV_180) +
            (float)Handle->hmcHandler.Raw.z *
                sin(Handle->Angle.Roll * PHI_DIV_180);
        // Calculate actual heading
        if (Handle->hmcHandler.CompassData.y_horizontal < 0)
            Handle->hmcHandler.CompassData.actual_heading =
                180 +
                (180 + ((atan2(Handle->hmcHandler.CompassData.y_horizontal,
                               Handle->hmcHandler.CompassData.x_horizontal)) *
                        (180 / 3.14)));
        else
            Handle->hmcHandler.CompassData.actual_heading =
                ((atan2(Handle->hmcHandler.CompassData.y_horizontal,
                        Handle->hmcHandler.CompassData.x_horizontal)) *
                 (180 / 3.14));
        Handle->hmcHandler.CompassData.actual_heading +=
            Handle->hmcHandler.OffsetScale.declination;
        if (Handle->hmcHandler.CompassData.actual_heading < 0)
            Handle->hmcHandler.CompassData.actual_heading += 360;
        else if (Handle->hmcHandler.CompassData.actual_heading >= 360)
            Handle->hmcHandler.CompassData.actual_heading -= 360;
        // Calculate angle yaw with the compass value
        Handle->mpuHandler.GyroAxis.yaw -=
            course_deviation(Handle->mpuHandler.GyroAxis.yaw,
                             Handle->hmcHandler.CompassData.actual_heading) /
            1200.0;
        if (Handle->mpuHandler.GyroAxis.yaw < 0)
            Handle->mpuHandler.GyroAxis.yaw += 360;
        else if (Handle->mpuHandler.GyroAxis.yaw >= 360)
            Handle->mpuHandler.GyroAxis.yaw -= 360;
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
    Handler->mpuHandler.GyroOffset.x = 0;
    Handler->mpuHandler.GyroOffset.y = 0;
    Handler->mpuHandler.GyroOffset.z = 0;

    for (int i = 0; i < 2000; i++) {
        MPU6050_GyroRead_Raw(&Handler->mpuHandler);
        x += Handler->mpuHandler.GyroRaw.x;
        y += Handler->mpuHandler.GyroRaw.y;
        z += Handler->mpuHandler.GyroRaw.z;
        if (i % 25 == 0) HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        Handler->Calibration.waitUntil(&Handler->Calibration.StartTask, 4);
    }
    Handler->mpuHandler.GyroOffset.x = x / 2000;
    Handler->mpuHandler.GyroOffset.y = y / 2000;
    Handler->mpuHandler.GyroOffset.z = z / 2000;
    // Put the calibration value to the Flash here
}

void Sensor_Compass_Calibration(Sensor_handle_t *Handler) {
    int16_t compass_cal_values[6];
    uint32_t i = 0;
    HMC5883L_Raw_t *Raw = &Handler->hmcHandler.Raw;
    memset(compass_cal_values, 0, 6 * sizeof(int16_t));
    while (*Handler->Calibration.ThrustChannel > 1800) {
        if (HMC5883L_Get_Raw(&Handler->hmcHandler) != HMC5883L_OK) return;
        if (Raw->x < compass_cal_values[0])
            compass_cal_values[0] = Raw->x;  // Min X
        if (Raw->x > compass_cal_values[1])
            compass_cal_values[1] = Raw->x;  // Max X
        if (Raw->y < compass_cal_values[2])
            compass_cal_values[2] = Raw->y;  // Min Y
        if (Raw->y > compass_cal_values[3])
            compass_cal_values[3] = Raw->y;  // Max Y
        if (Raw->z < compass_cal_values[4])
            compass_cal_values[4] = Raw->z;  // Min Z
        if (Raw->z > compass_cal_values[5])
            compass_cal_values[5] = Raw->z;  // Max Z
        if (i % 25 == 0) HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        i++;
        Handler->Calibration.waitUntil(&Handler->Calibration.StartTask, 4);
    }
    memmove(Handler->hmcHandler.OffsetScale.compass_cal_values,
            compass_cal_values, 6 * sizeof(int16_t));
    printf("MinX=%d,MaxX=%d\r\nMinY=%d,MaxY=%d\r\nMinZ=%d,MaxZ=%d\r\n",
           compass_cal_values[0], compass_cal_values[1], compass_cal_values[2],
           compass_cal_values[3], compass_cal_values[4], compass_cal_values[5]);
    // Calculates the calibration and offset values
    Sensor_compass_calculate_offset(&Handler->hmcHandler.OffsetScale);
    // Put the function to store the calibration values to the Flash
}

void Sensor_compass_calculate_offset(HMC5883L_Offset_Scale_t *OffsetScale) {
    int16_t compass_cal_values[6];
    memmove(compass_cal_values, OffsetScale->compass_cal_values,
            6 * sizeof(int16_t));
    OffsetScale->scale_y =
        ((float)compass_cal_values[1] - compass_cal_values[0]) /
        (compass_cal_values[3] - compass_cal_values[2]);
    OffsetScale->scale_z =
        ((float)compass_cal_values[1] - compass_cal_values[0]) /
        (compass_cal_values[5] - compass_cal_values[4]);

    OffsetScale->offset_x =
        (compass_cal_values[1] - compass_cal_values[0]) / 2 -
        compass_cal_values[1];
    OffsetScale->offset_y =
        (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 -
         compass_cal_values[3]) *
        OffsetScale->scale_y;
    OffsetScale->offset_z =
        (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 -
         compass_cal_values[5]) *
        OffsetScale->scale_z;
}

// The following subrouting calculates the smallest difference between two
// heading values.
float course_deviation(float course_b, float course_c) {
    float course_a;
    float base_course_mirrored;
    float actual_course_mirrored;
    course_a = course_b - course_c;
    if (course_a < -180 || course_a > 180) {
        if (course_c > 180)
            base_course_mirrored = course_c - 180;
        else
            base_course_mirrored = course_c + 180;
        if (course_b > 180)
            actual_course_mirrored = course_b - 180;
        else
            actual_course_mirrored = course_b + 180;
        course_a = actual_course_mirrored - base_course_mirrored;
    }
    return course_a;
}
