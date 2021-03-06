/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : MS5611.h
 * @brief          : Header for MS5611.c file.
 *                   This file contains the common defines of
 *                   the MS5611 library.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Sang Tan Truong.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MS5611_H_
#define __MS5611_H_

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "string.h"

/* Exported types ------------------------------------------------------------*/
typedef enum MS5611_Osr_t {
    MS5611_ULTRA_HIGH_RES = 0x08,
    MS5611_HIGH_RES = 0x06,
    MS5611_STANDARD = 0x04,
    MS5611_LOW_POWER = 0x02,
    MS5611_ULTRA_LOW_POWER = 0x00
} MS5611_Osr_t;

typedef enum MS5611_Status_t {
    MS5611_OK = 0,
    MS5611_ERROR = 1,
    MS5611_BUSY = 2,
    MS5611_TIMEOUT = 3,
} MS5611_Status_t;

typedef struct MS5611_PROM_t {
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
    uint16_t CRC_Value;
    uint32_t dTTemp;
    uint32_t OFFTemp;
    uint32_t SENSTemp;
    float TempSens;
} MS5611_PROM_t;

typedef struct MS5611_Temp_t {
    uint32_t Raw;
    int32_t dT;
    float Degree;
} MS5611_Temp_t;

typedef struct MS5611_Press_t {
    int32_t Raw;
    float Pressure;
} MS5611_Press_t;

typedef struct MS5611_Altitude_t {
    int32_t StandardPressure;
    float Altitude;
} MS5611_Altitude_t;

typedef struct MS5611_Handle_t {
    MS5611_Osr_t Osr;
    I2C_HandleTypeDef hi2c;
    MS5611_Status_t Status;
    MS5611_Temp_t Temp;
    MS5611_Press_t Pressure;
    MS5611_Altitude_t Altitude;
    MS5611_PROM_t PROM;
    uint32_t ADC_Value;
    void (*wait)(uint32_t);
} MS5611_Handle_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
MS5611_Status_t MS5611_Write(MS5611_Handle_t *Handle, uint8_t Command);

MS5611_Status_t MS5611_Read(MS5611_Handle_t *Handle, uint8_t *DataR,
                            uint8_t Size);

MS5611_Status_t MS5611_Reset(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Read_Prom(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Convert_Temperature(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Convert_Pressure(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Read_ADC(MS5611_Handle_t *Handle);

MS5611_Status_t MS56111_Get_Temperature(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Get_Pressure(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Get_Altitude(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Init(MS5611_Handle_t *Handle, I2C_HandleTypeDef *I2c);

MS5611_Status_t MS5611_Get_Temp_Wait(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Get_Press_Wait(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Get_Temp_Press_Wait(MS5611_Handle_t *Handle);

MS5611_Status_t MS5611_Update(MS5611_Handle_t *Handle);

/* Private defines -----------------------------------------------------------*/
#define MS5611_ADDRESS (0x77 << 1)

#define MS5611_CMD_ADC_READ (0x00)
#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_CONV_D1 (0x40)
#define MS5611_CMD_CONV_D2 (0x50)
#define MS5611_CMD_READ_PROM (0xA0)

#define MS5611_2_POW_8 256.0f
#define MS5611_2_POW_23 8388608.0f
#define MS5611_2_POW_7 128.0f
#define MS5611_2_POW_16 65536
#define MS5611_2_POW_15 32768.0f
#define MS5611_2_POW_21 2097152.0f

#define MS5611_PRESSURE_COEFFICIENT -0.117f  // 1m == 0.117mbar
#endif                                       /* __MS5611_H_ */
