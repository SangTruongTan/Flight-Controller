/*
File:                HMC5883.c
Written by:             Sang Truong Tan
Date Written:           02/13/2022
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
#include "HMC5883.h"

/* Private variables ---------------------------------------------------------*/

/* Private function definations ----------------------------------------------*/
/**
 * @brief  The function read data from device's register.
 * @param Handle The handle of the hmc5883l ic.
 * @param Address The Address of the register.
 * @param DataR The buffer's pointer to read the data.
 * @param size The size of the data that needs to read.
 * @retval HMC5883_Status_t
 */
HMC5883L_Status_t HMC5883L_Read(HMC5883L_Handle_t *Handle, uint8_t Address,
                                uint8_t *DataR, uint8_t size) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&Handle->hi2c, HMC5883L_ADDRESS, Address, 1,
                              DataR, size, 100);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_BUSY:
                return HMC5883L_BUSY;
                break;
            case HAL_TIMEOUT:
                return HMC5883L_TIMEOUT;
                break;
            case HAL_ERROR:
                return HMC5883L_ERROR;
            default:
                break;
        }
    }
    return HMC5883L_OK;
}

/**
 * @brief  The function write data to device's register.
 * @param Handle The handle type of the hmc5883L ic.
 * @param Address The Address of the register.
 * @param DataW The buffer's pointer to write the data.
 * @param size The size of the data that needs to write.
 * @retval HMC5883_State_t
 */
HMC5883L_Status_t HMC5883L_Write(HMC5883L_Handle_t *Handle, uint8_t Address,
                                 uint8_t *DataW, uint8_t size) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&Handle->hi2c, HMC5883L_ADDRESS, Address, 1,
                               DataW, size, 100);
    if (status != HAL_OK) {
        switch (status) {
            case HAL_BUSY:
                return HMC5883L_BUSY;
                break;
            case HAL_TIMEOUT:
                return HMC5883L_TIMEOUT;
                break;
            case HAL_ERROR:
                return HMC5883L_ERROR;
            default:
                break;
        }
    }
    return HMC5883L_OK;
}

/**
 * @brief Find the Resolution rely on the Gain configuration.
 * @param GainConfig The configuration of the gain attribute.
 * @retval float The gain at float type.
 */
float HMC5883L_Get_Gain(uint8_t GainConfig) {
    float Resolution = 0;
    switch (GainConfig) {
        case HMC5883L_GAIN_1370:
            Resolution = HMC5883L_RESOLUTION_1370;
            break;
        case HMC5883L_GAIN_1090:
            Resolution = HMC5883L_RESOLUTION_1090;
            break;
        case HMC5883L_GAIN_820:
            Resolution = HMC5883L_RESOLUTION_820;
            break;
        case HMC5883L_GAIN_660:
            Resolution = HMC5883L_RESOLUTION_660;
            break;
        case HMC5883L_GAIN_440:
            Resolution = HMC5883L_RESOLUTION_440;
            break;
        case HMC5883L_GAIN_390:
            Resolution = HMC5883L_RESOLUTION_390;
            break;
        case HMC5883L_GAIN_330:
            Resolution = HMC5883L_RESOLUTION_230;
            break;
        case HMC5883L_GAIN_230:
            Resolution = HMC5883L_RESOLUTION_230;
            break;
        default:
            break;
    }
    return Resolution;
}

/**
 * @brief Check the DRY header. Must be check before read the value.
 * @param Handle The Handle type of the library.
 * @retval bool true means ready, false means not ready.
 */
bool HMC5883L_Check_DRY(HMC5883L_Handle_t *Handle) {
    bool retval = false;
    uint8_t dataR;
    // Read the status register
    HMC5883L_Read(Handle, HMC5883L_RA_STATUS, &dataR, 1);
    retval = dataR & 0x01;
    return retval;
}

/**
 * @brief Read the raw data at three axis.
 * @param Handle The pointer of the handle type.
 * @retval HMC5883L_Status_t The status of the HMC5883.
 */
HMC5883L_Status_t HMC5883L_Get_Raw(HMC5883L_Handle_t *Handle) {
    HMC5883L_Status_t status;
    uint8_t dataR[6];
    // Read the X, Y, Z register value
    status = HMC5883L_Read(Handle, HMC5883L_RA_DATAX_H, dataR, 6);
    Handle->Status = status;
    if (status == HMC5883L_OK) {
        Handle->Raw.x = dataR[0] << 8 | dataR[1];
        Handle->Raw.z = dataR[2] << 8 | dataR[3];
        Handle->Raw.y = dataR[4] << 8 | dataR[5];
        Handle->State = HMC5883L_OK_STATE;
    } else {
        Handle->State = HMC5883L_ERROR_STATE;
    }
    return status;
}

/**
 * @brief Read the scaled data at three axis in mG.
 * @param Handle The pointer of the handle type.
 * @retval HMC5883L_Status_t The status of the HMC5883.
 */
HMC5883L_Status_t HMC5883L_Get_Scaled(HMC5883L_Handle_t *Handle) {
    HMC5883L_Status_t status;
    float Resolution = Handle->Resolution;
    // Read the raw data
    status = HMC5883L_Get_Raw(Handle);
    Handle->Status = status;
    if (status != HMC5883L_OK) {
        Handle->State = HMC5883L_ERROR_STATE;
    } else {
        Handle->Scaled.x = (float)Handle->Raw.x * Resolution;
        Handle->Scaled.y = (float)Handle->Raw.y * Resolution;
        Handle->Scaled.z = (float)Handle->Raw.z * Resolution;
        Handle->State = HMC5883L_OK_STATE;
    }
    return status;
}

/**
 * @brief Initialize the HMC5883L.
 * @param Handle The handle of the initialization.
 * @param I2c The handle type of the I2C.
 * @retval HMC5883L_State_t The state of the IC after initialize.
 */
HMC5883L_State_t HMC5883L_Init(HMC5883L_Handle_t *Handle,
                               I2C_HandleTypeDef *I2c) {
    // Local Variable
    HMC5883L_Status_t Status;
    uint8_t DataR;
    uint8_t DataW;
    // Copy the I2C typedef and the Init struct
    memcpy(&Handle->hi2c, I2c, sizeof(*I2c));
    // Read the Identify A register
    Status = HMC5883L_Read(Handle, HMC5883L_RA_ID_A, &DataR, 1);
    if (Status != HMC5883L_OK) return HMC5883L_ERROR_STATE;
    if (DataR != HMC5883L_IDENTIFY_A) return HMC5883L_UN_IDENTIFY_STATE;
    // Configure the HMC5883L IC
    // Configure the Configuration Register A
    DataW = 0x00;
    DataW = Handle->Init.Average | Handle->Init.OutputRate | Handle->Init.Bias;
    Status = HMC5883L_Write(Handle, HMC5883L_RA_CONFIG_A, &DataW, 1);
    if (Status != HMC5883L_OK) return HMC5883L_ERROR_STATE;
    // Configure the Configuration Register B
    DataW = 0x00;
    DataW = Handle->Init.Gain;
    Status = HMC5883L_Write(Handle, HMC5883L_RA_CONFIG_B, &DataW, 1);
    if (Status != HMC5883L_OK) return HMC5883L_ERROR_STATE;
    // Configure the MODE
    DataW = 0x00;
    DataW = Handle->Init.Mode;
    Status = HMC5883L_Write(Handle, HMC5883L_RA_MODE, &DataW, 1);
    if (Status != HMC5883L_OK) return HMC5883L_ERROR_STATE;
    // Get the Resolution
    Handle->Resolution = HMC5883L_Get_Gain(Handle->Init.Gain);
    // Return value
    Handle->State = HMC5883L_OK_STATE;
    return HMC5883L_OK_STATE;
}
