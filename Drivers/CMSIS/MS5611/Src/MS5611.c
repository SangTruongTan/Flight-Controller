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
MS5611_Read(MS5611_Handle_t *Handle, uint8_t *DataR, uint8_t Size) {
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
MS5611_Reset(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    status = MS5611_Write(Handle, MS5611_CMD_RESET);
    return status;
}

/**
 * @brief Read the PROM data.
 * @param Handle The pointer of the handle.
 * MS5611_Status_t.
 */
