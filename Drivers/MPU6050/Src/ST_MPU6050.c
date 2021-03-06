/*
Library:                MPU6050 - Accelerometer and gyroscope -  for ST
Microntroller Written by:             Sang Truong Tan Date Written: 03/26/2021
Last modified:      04/06/2021
Description:
References:
            1) STM Devices

            2) MPU6050 datasheet
                  https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

*Copyright (C) 2021 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or
modify it under the terms of the GNU General Public Licenseversion 3 as
published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without
WARRANTY and Author is not liable for any damages caused directly or indirectly
by this software, read more about this on the GNU General Public License.
*/

/********************* Header files *********************/
#include "ST_MPU6050.h"

/********************* Global Variable *********************/
#define M_PI		3.14159265358979323846

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/******************** Function Definitions *********************/

// 1. Read Register
MPU6050_Status_t MPU6050_Read(MPU6050_handle_t *Handle, uint8_t *ui8pDataR,
                              uint8_t ui8Add, uint8_t ui8size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &Handle->hi2c, MPU6050_Address, ui8Add, 1, ui8pDataR, ui8size, 10);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_ERROR:
                return MPU6050_ERROR;
                break;
            case HAL_BUSY:
                return MPU6050_BUSY;
                break;
            case HAL_TIMEOUT:
                return MPU6050_TIMEOUT;
                break;
            default:
                break;
        }
    }
    return MPU6050_OK;
}

// 2. Write Register
MPU6050_Status_t MPU6050_Write(MPU6050_handle_t *Handle, uint8_t *ui8pDataW,
                               uint8_t ui8Add, uint8_t ui8size) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &Handle->hi2c, MPU6050_Address, ui8Add, 1, ui8pDataW, ui8size, 10);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_ERROR:
                return MPU6050_ERROR;
                break;
            case HAL_BUSY:
                return MPU6050_BUSY;
                break;
            case HAL_TIMEOUT:
                return MPU6050_TIMEOUT;
                break;
            default:
                break;
        }
    }
    return MPU6050_OK;
}
// 3.  MPU6050 Initialize
MPU6050_State_t MPU6050_Init(MPU6050_handle_t *Handle,
                             I2C_HandleTypeDef *hi2c) {
    uint8_t ui8buffer;
    MPU6050_Status_t Status;
    uint32_t i = 0;
    MPU6050_InitTypedef mpuInitTypeDef = Handle->Init;
    float *Mpu6050_AcceSensi = &Handle->AcceSens;
    float *Mpu6050_GyroSensi = &Handle->GyroSens;
    // I2c Handle Typedef
    memcpy(&Handle->hi2c, hi2c, sizeof(*hi2c));
    // Read Who AM I Register
    Status = MPU6050_Read(Handle, &ui8buffer, MPU6050_WHO_AM_I_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    if (ui8buffer != 0x68) {
        Handle->State = MPU6050_ID_ERROR_STATE;
        return MPU6050_ID_ERROR_STATE;
    }
    // Reset IMU
    Status = MPU6050_Read(Handle, &ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    ui8buffer |= 0x80;
    Status = MPU6050_Write(Handle, &ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    for (i = 0; i < 2400000; i++) {
        asm("NOP");
    }

    ui8buffer = 0x00;  // Enable IMU
    Status = MPU6050_Write(Handle, &ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    // Setting Accelerometer full scale range
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef.ui8AcceFullScale;
    Status = MPU6050_Write(Handle, &ui8buffer, MPU6050_ACCEL_CONFIG_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    // Setting Gyroscope full scale range
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef.ui8GyroFullScale;
    Status = MPU6050_Write(Handle, &ui8buffer, MPU6050_GYRO_CONFIG_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    // Setting Digital Low Pass Filter
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef.ui8DLPF;
    Status = MPU6050_Write(Handle, &ui8buffer, MPU6050_CONFIG_REG, 1);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    // Setting sensitivity for Accelerometer
    switch (mpuInitTypeDef.ui8AcceFullScale) {
        case MPU6050_ACCE_FULLSCALE_2G:
            *Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_2G;
            break;
        case MPU6050_ACCE_FULLSCALE_4G:
            *Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_4G;
            break;
        case MPU6050_ACCE_FULLSCALE_8G:
            *Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_8G;
            break;
        case MPU6050_ACCE_FULLSCALE_16G:
            *Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_16G;
            break;
        default:
            Handle->State = MPU6050_ACCE_FULLSCALED_ERROR_STATE;
            return MPU6050_ACCE_FULLSCALED_ERROR_STATE;
            break;
    }

    switch (mpuInitTypeDef.ui8GyroFullScale) {
        case MPU6050_GYRO_FULLSCALE_250DPS:
            *Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_250DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_500DPS:
            *Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_500DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_1000DPS:
            *Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_1000DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_2000DPS:
            *Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_2000DPS;
            break;
        default:
            Handle->State = MPU6050_GYRO_FULLSCALED_ERROR_STATE;
            return MPU6050_GYRO_FULLSCALED_ERROR_STATE;
            break;
    }
    Handle->State = MPU6050_OK_STATE;
    return MPU6050_OK_STATE;
}

// 4 . MPU6050 Accelerometer read data raw
MPU6050_State_t MPU6050_AcceRead_Raw(MPU6050_handle_t *Handle) {
    uint8_t ui8Buffer[6];
    MPU6050_AcceDataRaw acRaw;
    MPU6050_Status_t Status = MPU6050_OK;
    // Read X, Y, Z value
    Status = MPU6050_Read(Handle, ui8Buffer, MPU6050_ACCEL_XOUT_H_REG, 6);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    acRaw.x = ui8Buffer[0] << 8 | ui8Buffer[1];
    acRaw.x -= Handle->AccOffset.x;
    acRaw.y = ui8Buffer[2] << 8 | ui8Buffer[3];
    acRaw.y *= -1;
    acRaw.y -= Handle->AccOffset.y;
    acRaw.z = ui8Buffer[4] << 8 | ui8Buffer[5];
    acRaw.totalVector =
        sqrt((acRaw.x * acRaw.x) + (acRaw.y * acRaw.y) + (acRaw.z * acRaw.z));
    Handle->AccRaw = acRaw;
    Handle->State = MPU6050_OK_STATE;
    return MPU6050_OK_STATE;
}

// 5. MPU6050 Accelerometer read data scaled
/**
 * @brief  Read the scaled data from MPU. The data using m/s^2 unit.
 * @retval MPU6050_State_t The state of the imu.
 */
MPU6050_State_t MPU6050_AcceRead_Scaled(MPU6050_handle_t *Handle) {
    MPU6050_AcceDataRaw mpuRaw;
    MPU6050_AcceDataScaled mpuScaled;
    float Mpu6050_AcceSensi = Handle->AcceSens;
    MPU6050_Status_t Status = MPU6050_AcceRead_Raw(Handle);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    mpuRaw = Handle->AccRaw;
    mpuScaled.x = (float)(mpuRaw.x) / Mpu6050_AcceSensi * G_VALUE;
    mpuScaled.y = (float)(mpuRaw.y) / Mpu6050_AcceSensi * G_VALUE;
    mpuScaled.z = (float)(mpuRaw.z) / Mpu6050_AcceSensi * G_VALUE;
    Handle->AccScaled = mpuScaled;
    return MPU6050_OK_STATE;
}

// 6. MPU6050 Gyroscope read data raw
MPU6050_State_t MPU6050_GyroRead_Raw(MPU6050_handle_t *Handle) {
    MPU6050_GyroDataRaw mpuRaw;
    uint8_t ui8Buffer[6];
    MPU6050_Status_t Status;
    // Read X, Y, Z value
    Status = MPU6050_Read(Handle, ui8Buffer, MPU6050_GYRO_XOUT_H_REG, 6);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    mpuRaw.x = ui8Buffer[0] << 8 | ui8Buffer[1];
    mpuRaw.x *= -1;
    mpuRaw.x -= Handle->GyroOffset.x;
    mpuRaw.y = ui8Buffer[2] << 8 | ui8Buffer[3];
    mpuRaw.y *= -1;
    mpuRaw.y -= Handle->GyroOffset.y;
    mpuRaw.z = ui8Buffer[4] << 8 | ui8Buffer[5];
    mpuRaw.z *= -1;
    mpuRaw.z -= Handle->GyroOffset.z;
    Handle->GyroRaw = mpuRaw;
    Handle->State = MPU6050_OK_STATE;
    return MPU6050_OK_STATE;
}

// 7. MPU6050 Gyroscope read data scaled
MPU6050_State_t MPU6050_GyroRead_Scaled(MPU6050_handle_t *Handle) {
    MPU6050_GyroDataRaw mpuRaw;
    MPU6050_GyroDataScaled mpuScaled;
    float Mpu6050_GyroSensi = Handle->GyroSens;
    MPU6050_Status_t Status = MPU6050_GyroRead_Raw(Handle);
    Handle->Status = Status;
    if (Status != MPU6050_OK) {
        Handle->State = MPU6050_ERROR_STATE;
        return MPU6050_ERROR_STATE;
    }
    mpuRaw = Handle->GyroRaw;
    mpuScaled.x = (float)(mpuRaw.x) / Mpu6050_GyroSensi;
    mpuScaled.y = (float)(mpuRaw.y) / Mpu6050_GyroSensi;
    mpuScaled.z = (float)(mpuRaw.z) / Mpu6050_GyroSensi;
    Handle->GyroScaled = mpuScaled;
    return MPU6050_OK_STATE;
}

/**
 * @brief  Read the acceleration and the gyroscope data.
 * @retval MPU6050_State_t.
 */
MPU6050_State_t MPU6050_read_mpu_data(MPU6050_handle_t *Handle) {
    MPU6050_State_t State;
    float Mpu6050_GyroSensi = Handle->GyroSens;
    State = MPU6050_AcceRead_Raw(Handle);
    Handle->State = State;
    if (State != MPU6050_OK_STATE) {
        return MPU6050_ERROR_STATE;
    }
    State = MPU6050_GyroRead_Raw(Handle);
    Handle->State = State;
    if (State != MPU6050_OK_STATE) {
        return MPU6050_ERROR_STATE;
    }
    //Apply the filter
    rtU.sensor_data_in.ddx = Handle->AccRaw.x;
    rtU.sensor_data_in.ddy = Handle->AccRaw.y;
    rtU.sensor_data_in.ddz = Handle->AccRaw.z;
    rtU.sensor_data_in.p = Handle->GyroRaw.x;
    rtU.sensor_data_in.q = Handle->GyroRaw.y;
    rtU.sensor_data_in.r = Handle->GyroRaw.z;
    Sensor_step();
    Handle->AccRaw.x = rtY.ddx;
    Handle->AccRaw.y = rtY.ddy;
    Handle->AccRaw.z = rtY.ddz;
    Handle->GyroRaw.x = rtY.p;
    Handle->GyroRaw.y = rtY.q;
    Handle->GyroRaw.z = rtY.r;
    //Calculate angle from the angularate
    Handle->GyroInput.pitch =
        Handle->GyroInput.pitch * 0.7 +
        ((float)Handle->GyroRaw.x / Mpu6050_GyroSensi) * 0.3;
    Handle->GyroInput.roll =
        Handle->GyroInput.roll * 0.7 +
        ((float)Handle->GyroRaw.y / Mpu6050_GyroSensi) * 0.3;
    Handle->GyroInput.yaw =
        Handle->GyroInput.yaw * 0.7 +
        ((float)Handle->GyroRaw.z / Mpu6050_GyroSensi) * 0.3;
    // Gyro angle calculations
    Handle->GyroAxis.roll += (float)Handle->GyroRaw.y * 0.0000611;
    Handle->GyroAxis.pitch += (float)Handle->GyroRaw.x * 0.0000611;
    Handle->GyroAxis.yaw += (float)Handle->GyroRaw.z * 0.0000611;
    if (Handle->GyroAxis.yaw < 0) {
        Handle->GyroAxis.yaw += 360;
    } else if (Handle->GyroAxis.yaw > 360) {
        Handle->GyroAxis.yaw -= 360;
    }
    //Compenstate for roll and pitch
    Handle->GyroAxis.pitch -= Handle->GyroAxis.roll*sin((float)Handle->GyroRaw.z * 0.000001066);
    Handle->GyroAxis.roll += Handle->GyroAxis.pitch*sin((float)Handle->GyroRaw.z * 0.000001066);
    // Acceleration calculated
    if(abs(Handle->AccRaw.x) < Handle->AccRaw.totalVector) {
        Handle->AcceAxis.roll = asin((float)Handle->AccRaw.x/Handle->AccRaw.totalVector)*180/M_PI;
    }
    if(abs(Handle->AccRaw.y) < Handle->AccRaw.totalVector) {
        Handle->AcceAxis.pitch = asin((float)Handle->AccRaw.y/Handle->AccRaw.totalVector)*180/M_PI;
    }

    return MPU6050_OK_STATE;
}

/** Copyright (C) 2021 - TTSang **/
