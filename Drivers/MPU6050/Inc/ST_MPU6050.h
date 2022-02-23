/*
Library:                MPU6050 - Accelerometer and gyroscope -  for ST Microntroller
Written by:             Sang Truong Tan
Date Written:           03/26/2021
Last modified:      04/06/2021
Description:
References:
            1) STM Devices 

            2) MPU6050 datasheet
                  https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

*Copyright (C) 2021 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/


#ifndef     ST_MPU6050_H__
#define     ST_MPU6050_H__


/* System Header files */
#include <stdint.h>
#include <stdbool.h>
#include <main.h>
#include <string.h>
/* User header files */


/* List of defines */

/* Constant value */
#define G_VALUE 9.80665f

/* I2C device address */
#define      MPU6050_Address  ((uint8_t)0x68<<1)

/* List of Register */
#define     MPU6050_SMPLRT_DIV_REG                      0x19
#define     MPU6050_CONFIG_REG                          0x1A
#define     MPU6050_GYRO_CONFIG_REG                     0x1B
#define     MPU6050_ACCEL_CONFIG_REG                    0x1C

#define     MPU6050_FIFO_EN_REG                         0x23

#define     MPU6050_I2C_MST_CTRL_REG                    0x24
#define     MPU6050_2C_MST_STATUS_REG                   0x36

#define     MPU6050_INT_PIN_CFG_REG                     0x37
#define     MPU6050_INT_ENABLE_REG                      0x38
#define     MPU6050_INT_STATUS_REG                      0x3A

#define     MPU6050_ACCEL_XOUT_H_REG                    0x3B
#define     MPU6050_ACCEL_XOUT_L_REG                    0x3C

#define     MPU6050_ACCEL_YOUT_H_REG                    0x3D
#define     MPU6050_ACCEL_YOUT_L_REG                    0x3E

#define     MPU6050_ACCEL_ZOUT_H_REG                    0x3F
#define     MPU6050_ACCEL_ZOUT_L_REG                    0x40

#define     MPU6050_GYRO_XOUT_H_REG                     0x43
#define     MPU6050_GYRO_XOUT_L_REG                     0x44

#define     MPU6050_GYRO_YOUT_H_REG                     0x45
#define     MPU6050_GYRO_YOUT_L_REG                     0x46

#define     MPU6050_GYRO_ZOUT_H_REG                     0x47
#define     MPU6050_GYRO_ZOUT_L_REG                     0x48

#define     MPU6050_USER_CTRL_REG                       0x6A
#define     MPU6050_PWR_MGMT_1_REG                      0x6B
#define     MPU6050_PWR_MGMT_2_REG                      0x6C

#define     MPU6050_WHO_AM_I_REG                        0x75

/* Digital Low Pass Filter */
#define     MPU6050_DLPF_0                              ((uint8_t)0x00<<0)
#define     MPU6050_DLPF_1                              ((uint8_t)0x01<<0)
#define     MPU6050_DLPF_2                              ((uint8_t)0x02<<0)
#define     MPU6050_DLPF_3                              ((uint8_t)0x03<<0)
#define     MPU6050_DLPF_4                              ((uint8_t)0x04<<0)
#define     MPU6050_DLPF_5                              ((uint8_t)0x05<<0)
#define     MPU6050_DLPF_6                              ((uint8_t)0x06<<0)

/* Gyroscope full scale range */
#define     MPU6050_GYRO_FULLSCALE_250DPS               ((uint8_t)0x00<<3)
#define     MPU6050_GYRO_FULLSCALE_500DPS               ((uint8_t)0x01<<3)
#define     MPU6050_GYRO_FULLSCALE_1000DPS              ((uint8_t)0x02<<3)
#define     MPU6050_GYRO_FULLSCALE_2000DPS              ((uint8_t)0x03<<3)

/* Gyroscope sensitivity */
#define     MPU6050_GYRO_SENSI_250DPS                   131.0f                 /* Gyroscope sensitivity 131 LSB/Degree/s for +-250Degrees/s */
#define     MPU6050_GYRO_SENSI_500DPS                   65.5f                  /* Gyroscope sensitivity 65.5 LSB/Degree/s for +-500Degrees/s */
#define     MPU6050_GYRO_SENSI_1000DPS                  32.8f                  /* Gyroscope sensitivity 32.8 LSB/Degree/s for +-1000Degrees/s */
#define     MPU6050_GYRO_SENSI_2000DPS                  16.4f                  /* Gyroscope sensitivity 16.4 LSB/Degree/s for +-2000Degrees/s */

/* Accelerometer full scale range */
#define     MPU6050_ACCE_FULLSCALE_2G                   ((uint8_t)0x00<<3)
#define     MPU6050_ACCE_FULLSCALE_4G                   ((uint8_t)0x01<<3)
#define     MPU6050_ACCE_FULLSCALE_8G                   ((uint8_t)0x02<<3)
#define     MPU6050_ACCE_FULLSCALE_16G                  ((uint8_t)0x03<<3)

/* Accelerometer sensitivity */
#define     MPU6050_ACCE_SENSI_2G                       16384.0f              /* Accelerometer sensitivity 16384 LSB/g for +-2g range */
#define     MPU6050_ACCE_SENSI_4G                       8192.0f               /* Accelerometer sensitivity 8192 LSB/g for +-4g range */
#define     MPU6050_ACCE_SENSI_8G                       4096.0f               /* Accelerometer sensitivity 4096 LSB/g for +-8g range */
#define     MPU6050_ACCE_SENSI_16G                      2048.0f               /* Accelerometer sensitivity 2048 LSB/g for +-16g range */


/******************** Typedefs ********************/
//Status of the IMU
typedef enum MPU6050_Status_t {
    MPU6050_OK = 0,
    MPU6050_ERROR = 1,
    MPU6050_BUSY = 2,
    MPU6050_TIMEOUT = 3,
}MPU6050_Status_t;
// State of IMU
typedef enum MPU6050_State_t {
    MPU6050_OK_STATE = 0,
    MPU6050_ERROR_STATE = 1,
    MPU6050_ID_ERROR_STATE = 2,
    MPU6050_ACCE_FULLSCALED_ERROR_STATE = 3,
    MPU6050_GYRO_FULLSCALED_ERROR_STATE = 4,
}MPU6050_State_t;

//Initialize Typedef
typedef struct MPU6050_InitTypedef {
    uint8_t  ui8AcceFullScale;
    uint8_t  ui8GyroFullScale;
    uint8_t  ui8DLPF;
}MPU6050_InitTypedef;

// Accelerometer Data Raw
typedef struct MPU6050_AcceDataRaw {
    int16_t x;
    int16_t y;
    int16_t z;
    }MPU6050_AcceDataRaw;

// Accelerometer Data Scaled
typedef struct MPU6050_AcceDataScaled {
    float x;
    float y;
    float z;
    }MPU6050_AcceDataScaled;


// Gyroscope Data Raw Struct
typedef struct MPU6050_GyroDataRaw {
    int16_t x;
    int16_t y;
    int16_t z;
    }MPU6050_GyroDataRaw;

// Gyroscope Data Scaled
typedef struct MPU6050_GyroDataScaled {
    float x;
    float y;
    float z;
    }MPU6050_GyroDataScaled;
// Data for axis
typedef struct MPU6050_AcceAxis {
        float pitch;
        float roll;
        float yaw;
     }MPU6050_AcceAxis;
typedef struct MPU6050_GyroAxis {
        float pitch;
        float roll;
        float yaw;
     }MPU6050_GyroAxis;

//Handle Typedef of the IMU
typedef struct MPU6050_handle_t {
    I2C_HandleTypeDef hi2c;
    MPU6050_State_t State;
    MPU6050_Status_t Status;
    MPU6050_InitTypedef Init;
    MPU6050_AcceDataRaw AccRaw;
    MPU6050_AcceDataScaled AccScaled;
    MPU6050_GyroDataRaw GyroRaw;
    MPU6050_GyroDataScaled GyroScaled;
    float AcceSens;
    float GyroSens;
} MPU6050_handle_t;

/******************** List of function prototypes *********************/
//1. Read Register
MPU6050_Status_t MPU6050_Read (MPU6050_handle_t *Handle, uint8_t* ui8pDataR,
                    uint8_t ui8Add, uint8_t ui8size);
//2. Write Register
MPU6050_Status_t MPU6050_Write (MPU6050_handle_t *Handle, uint8_t* ui8pDataW,
                    uint8_t ui8Add, uint8_t ui8size);
//3.  MPU6050 Initialize
MPU6050_State_t MPU6050_Init (MPU6050_handle_t *Handle, I2C_HandleTypeDef *hi2c);

//4.  MPU6050 Accelerometer read data raw
MPU6050_State_t MPU6050_AcceRead_Raw (MPU6050_handle_t *Handle);

//5. MPU6050 Accelerometer read data scaled
MPU6050_State_t MPU6050_AcceRead_Scaled (MPU6050_handle_t *Handle);

//6. MPU6050 Gyroscope read data raw
MPU6050_State_t MPU6050_GyroRead_Raw (MPU6050_handle_t *Handle);

//7. MPU6050 Gyroscope read data scaled
MPU6050_State_t MPU6050_GyroRead_Scaled (MPU6050_handle_t *Handle);

/**
  * @brief  Read the acceleration and the gyroscope data.
  * @retval MPU6050_State_t.
  */
MPU6050_State_t MPU6050_read_mpu_data (MPU6050_handle_t *Handle);

#endif      /* End of File */
