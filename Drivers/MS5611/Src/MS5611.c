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
 * @brief The delay function.
 * @param Handle The pointer of the Handle.
 * @param Time The number of ms to delay.
 * @retval void
 */
void MS5611_Wait(MS5611_Handle_t *Handle, uint16_t Time) { Handle->wait(Time); }
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
    uint16_t RoomData[7];
    MS5611_PROM_t *Prom = &Handle->PROM;
    for (int i = 1; i <= 7; i++) {
        uint8_t Cmd = MS5611_CMD_READ_PROM | (i * 2);
        status = MS5611_Write(Handle, Cmd);
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        status = MS5611_Read(Handle, DataR, 2);
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        RoomData[i - 1] = DataR[0] << 8 | DataR[1];
    }
    // Assign data to Handle struct
    Prom->C1 = RoomData[0];
    Prom->C2 = RoomData[1];
    Prom->C3 = RoomData[2];
    Prom->C4 = RoomData[3];
    Prom->C5 = RoomData[4];
    Prom->C6 = RoomData[5];
    Prom->CRC_Value = RoomData[6] & 0x0F;
    Prom->dTTemp = Prom->C5 << 8;
    Prom->OFFTemp = Prom->C2 << 16;
    Prom->SENSTemp = Prom->C1 << 15;
    Prom->TempSens = (float)Prom->C6 / MS5611_2_POW_23;
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}

/**
 * @brief Convert the Temperature.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Convert_Temperature(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    MS5611_Osr_t Osr = Handle->Osr;
    status = MS5611_Write(Handle, MS5611_CMD_CONV_D2 | Osr);
    Handle->Status = status;
    return status;
}

/**
 * @brief Convert the Pressure.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Convert_Pressure(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    MS5611_Osr_t Osr = Handle->Osr;
    status = MS5611_Write(Handle, MS5611_CMD_CONV_D1 | Osr);
    Handle->Status = status;
    return status;
}

/**
 * @brief Read the ADC value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Read_ADC(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    uint8_t DataR[3];
    status = MS5611_Write(Handle, MS5611_CMD_ADC_READ);
    Handle->Status = status;
    if (status != MS5611_OK) {
        return status;
    }
    status = MS5611_Read(Handle, DataR, 3);
    Handle->Status = status;
    if (status != MS5611_OK) {
        return status;
    }
    Handle->ADC_Value =
        (uint32_t)DataR[0] << 16 | (uint32_t)DataR[1] << 8 | DataR[2];
    return MS5611_OK;
}

/**
 * @brief Get the Temperature value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS56111_Get_Temperature(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    int32_t dT;
    int16_t Temp;
    uint32_t D2;
    uint32_t dTTemp = Handle->PROM.dTTemp;
    float TempSens = Handle->PROM.TempSens;

    status = MS5611_Read_ADC(Handle);
    if (status != MS5611_OK) {
        return status;
    }
    D2 = Handle->ADC_Value;
    dT = (int32_t)(D2 - dTTemp);
    int16_t dTxTemp = dT * TempSens;
    Temp = 2000 + dTxTemp;
    bool Cond1 = dT > 16777216 || dT < -16776960;
    bool Cond2 = Temp > 8500 || Temp < -4000;
    if (Cond1 || Cond2) {
        status = MS5611_ERROR;
        return MS5611_ERROR;
    }
    Handle->Temp.Raw = Temp;
    Handle->Temp.dT = dT;
    Handle->Temp.Degree = Temp / 100.0;
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}

/**
 * @brief Get the Pressure value.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Pressure(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    uint32_t D1;
    int64_t OFF;
    int64_t SENS;
    int32_t P;
    uint64_t C3 = Handle->PROM.C3;
    uint64_t C4 = Handle->PROM.C4;
    uint64_t OFFTemp = Handle->PROM.OFFTemp;
    uint64_t SensTemp = Handle->PROM.SENSTemp;
    int64_t dT = Handle->Temp.dT;

    status = MS5611_Read_ADC(Handle);
    Handle->Status = status;
    if (status != MS5611_OK) {
        return status;
    }
    // Calculate the Pressure value
    D1 = Handle->ADC_Value;
    OFF = OFFTemp + (C4 * dT) / MS5611_2_POW_7;
    SENS = SensTemp + (C3 * dT) / MS5611_2_POW_8;
    P = ((D1 * SENS) / MS5611_2_POW_21 - OFF) / MS5611_2_POW_15;
    // Check the limit condition
    bool Cond1 = OFF > 12884705280 || OFF < -8589672450;
    bool Cond2 = SENS > 6442352640 || SENS < -4294836225;
    bool Cond3 = P > 120000 || P < 1000;
    if (Cond1 || Cond2 || Cond3) {
        Handle->Status = MS5611_ERROR;
        return MS5611_ERROR;
    }
    Handle->Pressure.Raw = P;
    Handle->Pressure.Pressure = P / 100.0;
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}

/**
 * @brief Get the altitude value from the standard Pressure.
 * @param Handle The pointer of the handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Altitude(MS5611_Handle_t *Handle) {
    float Difference = Handle->Pressure.Raw - Handle->Altitude.StandardPressure;
    Handle->Altitude.Altitude =
        Difference / MS5611_PRESSURE_COEFFICIENT / 100.0;
    return MS5611_OK;
}

/**
 * @brief Initialize the pressure sensor.
 * @param Handle The pointer of the handler.
 * @param I2c The pointer of the i2c interface.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Init(MS5611_Handle_t *Handle, I2C_HandleTypeDef *I2c) {
    MS5611_Status_t status;
    // Copy the I2C typedef and the Init struct
    memcpy(&Handle->hi2c, I2c, sizeof(*I2c));
    // Read the PROM
    status = MS5611_Reset(Handle);
    if (status != MS5611_OK) {
        return status;
    }
    MS5611_Wait(Handle, 200);
    status = MS5611_Read_Prom(Handle);
    if (status != MS5611_OK) {
        return status;
    }
    // Read the Temperature
    status = MS5611_Get_Temp_Wait(Handle);
    if (status != MS5611_OK) {
        Handle->Status = status;
        return status;
    }
    // Read standard altitude
    for (int i = 0; i < 500; i++) {
        status = MS5611_Update(Handle);
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        MS5611_Wait(Handle, 4);
    }
    Handle->Altitude.StandardPressure = Handle->Pressure.Raw;
    return status;
}
/**
 * @brief Get the Temperature Blocking.
 * @param Handle The pointer of the Handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Temp_Wait(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    status = MS5611_Convert_Temperature(Handle);
    if (status != MS5611_OK) {
        Handle->Status = status;
        return status;
    }
    MS5611_Wait(Handle, 12);
    status = MS56111_Get_Temperature(Handle);
    Handle->Status = status;
    return status;
}

/**
 * @brief Get the Pressure Blocking.
 * @param Handle The pointer of the Handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Press_Wait(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    status = MS5611_Convert_Pressure(Handle);
    if (status != MS5611_OK) {
        Handle->Status = status;
        return status;
    }
    MS5611_Wait(Handle, 12);
    status = MS5611_Get_Pressure(Handle);
    Handle->Status = status;
    return status;
}
/**
 * @brief Get the Temperature and the Pressure Blocking.
 * @param Handle The pointer of the Handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Get_Temp_Press_Wait(MS5611_Handle_t *Handle) {
    MS5611_Status_t status;
    // Read the Temperature
    status = MS5611_Get_Temp_Wait(Handle);
    if (status != MS5611_OK) {
        Handle->Status = status;
        return status;
    }
    // Read the pressure
    status = MS5611_Get_Press_Wait(Handle);
    if (status != MS5611_OK) {
        Handle->Status = status;
        return status;
    }
    // Get Altitude
    status = MS5611_Get_Altitude(Handle);
    Handle->Status = status;
    return MS5611_OK;
}

/**
 * @brief Update the Barometer. Must be called with interval 4ms.
 * @param Handle The pointer of the MS5611 Handle.
 * @retval MS5611_Status_t
 */
MS5611_Status_t MS5611_Update(MS5611_Handle_t *Handle) {
    static uint8_t count = 0;
    static uint8_t TempCount = 0;
    MS5611_Status_t status;
    if (count == 0) {
        status = MS5611_Convert_Pressure(Handle);
        if (status != MS5611_OK) {
            Handle->Status = status;
            return status;
        }
        count++;
    } else {
        count++;
        if (count == 4) {
            count = 1;
            if (TempCount < 20) {
                TempCount++;
                status = MS5611_Get_Pressure(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
                status = MS5611_Convert_Pressure(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
            } else if (TempCount == 20) {
                TempCount++;
                status = MS5611_Get_Pressure(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
                status = MS5611_Convert_Temperature(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
            } else {
                TempCount = 0;
                status = MS56111_Get_Temperature(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
                status = MS5611_Convert_Pressure(Handle);
                if (status != MS5611_OK) {
                    Handle->Status = status;
                    return status;
                }
            }
        }
    }
    Handle->Status = MS5611_OK;
    return MS5611_OK;
}
