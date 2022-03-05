/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : Ms5611.c
 * @brief          : The source file of the MS5611's library.
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
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "MS5611.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/**
 * @brief The function to write the ms5611 device.
 * @param Handle The handle of the device.
 * @param Command The command to write.
 * @retval MS5611_Status_t.
 */
MS5611_Status_t MS5611_Write(MS5611_Handle_t *Handle, uint8_t Command) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&Handle->hi2c, MS5611_ADDRESS, &Command, 1,
                                     100);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_BUSY:
                return MS5611_BUSY;
                break;
            case HAL_TIMEOUT:
                return MS5611_TIMEOUT;
                break;
            case HAL_ERROR:
                return MS5611_ERROR;
            default:
                break;
        }
    }
    return MS5611_OK;
}

/**
 * @brief Read the data from the device.
 * @param Handle The pointer of the handle.
 * @param DataR The pointer of the data to read.
 * @param Size The number of bytes to read.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Read(MS5611_Handle_t *Handle, uint8_t *DataR,
                            uint8_t Size) {
    HAL_StatusTypeDef status;
    status =
        HAL_I2C_Master_Receive(&Handle->hi2c, MS5611_ADDRESS, DataR, Size, 100);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_BUSY:
                return MS5611_BUSY;
                break;
            case HAL_TIMEOUT:
                return MS5611_TIMEOUT;
                break;
            case HAL_ERROR:
                return MS5611_ERROR;
            default:
                break;
        }
    }
    return MS5611_OK;
}

/**
 * @brief Reset the device.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t.
 */
MS5611_Status_t MS5611_Reset(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    status = MS5611_Write(Handle, MS5611_CMD_RESET);
    return status;
}

/**
 * @brief Read the PROM data.
 * @param Handle The pointer of the handle.
 * MS5611_Status_t.
 */
MS5611_Status_t MS5611_Read_Prom(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    uint8_t DataR[2];
    uint16_t RoomData[6];
    MS5611_PROM_t *Prom = &Handle->PROM;
    for (int i = 1; i <= 6; i++) {
        status = MS5611_Write(Handle, MS5611_CMD_READ_PROM | (i << 1));
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        status = MS5611_Read(Handle, DataR, 2);
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        RoomData[i] = DataR[0] << 8 | DataR[1];
    }
    //Assign data to Handle struct
    Prom->C1 = RoomData[0];
    Prom->C2 = RoomData[1];
    Prom->C3 = RoomData[2];
    Prom->C4 = RoomData[3];
    Prom->C5 = RoomData[4];
    Prom->C6 = RoomData[5];
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}

/**
 * @brief Convert the Temperature.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Convert_Temperature (MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    MS5611_Osr_t Osr = Handle->Osr;
    status = MS5611_Write(Handle, MS5611_CMD_CONV_D1 | Osr);
    Handle->Status = status;
    return status;
}

/**
 * @brief Convert the Pressure.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Convert_Pressure (MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    MS5611_Osr_t Osr = Handle->Osr;
    status = MS5611_Write(Handle, MS5611_CMD_CONV_D2 | Osr);
    Handle->Status = status;
    return status;
}

/**
 * @brief Read the ADC value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Read_ADC (MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    uint8_t DataR[3];
    status = MS5611_Read(Handle, DataR, 3);
    Handle->Status = status;
    if(status != MS5611_OK) {
        return status;
    }
    Handle->ADC_Value = DataR[0] << 16 | DataR[1] << 8 | DataR[2];
    return MS5611_OK;
}

/**
 * @brief Get the Temperature value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS56111_Get_Temperature (MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    int32_t dT;
    int16_t Temp;
    uint32_t D2;
    uint16_t C5 = Handle->PROM.C5;
    uint16_t C6 = Handle->PROM.C6;

    status = MS5611_Read_ADC(Handle);
    if(status != MS5611_OK) {
        return status;
    }
    D2 = Handle->ADC_Value;
    dT = D2 - C5*MS5611_2_POW_8;
    Temp = 2000 + dT*C6/MS5611_2_POW_23;
    bool Cond1 = dT > 16777216 || dT < -16776960;
    bool Cond2 = Temp > 8500 || Temp < -4000;
    if(Cond1 || Cond2) {
        status = MS5611_ERROR;
        return MS5611_ERROR;
    }
    Handle->Temp.Raw = Temp;
    Handle->Temp.dT = dT;
    Handle->Temp.Degree = Temp/100;
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}

/**
 * @brief Get the Pressure value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Pressure (MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    
}