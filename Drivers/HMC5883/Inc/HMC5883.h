/*
File:                HMC5883.h
Written by:             Sang Truong Tan
Date Written:           02/13/2022
Description:
References:

*Copyright (C) 2022 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#ifndef _HCM5883_H__
#define _HMC5883_H__

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdbool.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum HMC5883L_Status_t {
   HMC5883L_OK = 0,
   HMC5883L_ERROR = 1,
   HMC5883L_BUSY = 2,
   HMC5883L_TIMEOUT = 3,
} HMC5883L_Status_t;

typedef enum HMC5883L_State_t {
   HMC5883L_OK_STATE = 0,
   HMC5883L_UN_IDENTIFY_STATE,
   HMC5883L_ERROR_STATE,
} HMC5883L_State_t;

typedef struct HMC5883L_Init_t {
   uint8_t Average;
   uint8_t OutputRate;
   uint8_t Bias;
   uint8_t Gain;
   uint8_t Mode;
} HMC5883L_Init_t;

typedef struct HMC5883L_Raw_t {
   int16_t x;
   int16_t y;
   int16_t z;
} HMC5883L_Raw_t;

typedef struct HMC5883L_Scaled_t {
   float x;
   float y;
   float z;
} HMC5883L_Scaled_t;

typedef struct HMC5883L_Handle_t {
   I2C_HandleTypeDef hi2c;
   HMC5883L_Status_t Status;
   HMC5883L_State_t State;
   HMC5883L_Init_t Init;
   HMC5883L_Raw_t Raw;
   HMC5883L_Scaled_t Scaled;
   float Resolution;
} HMC5883L_Handle_t;

/* Private define ------------------------------------------------------------*/
#define HMC5883L_ADDRESS            ((uint8_t)0x1E << 1) // this device only has one address

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    5
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       2
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       0
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        (0x00 << HMC5883L_CRA_AVERAGE_BIT)
#define HMC5883L_AVERAGING_2        (0x01 << HMC5883L_CRA_AVERAGE_BIT)
#define HMC5883L_AVERAGING_4        (0x02 << HMC5883L_CRA_AVERAGE_BIT)
#define HMC5883L_AVERAGING_8        (0x03 << HMC5883L_CRA_AVERAGE_BIT)

#define HMC5883L_RATE_0P75          (0x00 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_1P5           (0x01 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_3             (0x02 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_7P5           (0x03 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_15            (0x04 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_30            (0x05 << HMC5883L_CRA_RATE_BIT)
#define HMC5883L_RATE_75            (0x06 << HMC5883L_CRA_RATE_BIT)

#define HMC5883L_BIAS_NORMAL        (0x00 << HMC5883L_CRA_BIAS_BIT)
#define HMC5883L_BIAS_POSITIVE      (0x01 << HMC5883L_CRA_BIAS_BIT)
#define HMC5883L_BIAS_NEGATIVE      (0x02 << HMC5883L_CRA_BIAS_BIT)

#define HMC5883L_CRB_GAIN_BIT       5
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          (0x00 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_1090          (0x01 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_820           (0x02 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_660           (0x03 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_440           (0x04 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_390           (0x05 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_330           (0x06 << HMC5883L_CRB_GAIN_BIT)
#define HMC5883L_GAIN_230           (0x07 << HMC5883L_CRB_GAIN_BIT)

#define HMC5883L_MODEREG_BIT        0
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

#define HMC5883L_IDENTIFY_A         0x48
#define HMC5883L_IDENTIFY_B         0x34
#define HMC5883L_IDENTIFY_C         0x33
//HMC5883L Resolution
#define HMC5883L_RESOLUTION_1370    (float) 0.73
#define HMC5883L_RESOLUTION_1090    (float) 0.92      //Default
#define HMC5883L_RESOLUTION_820     (float) 1.22
#define HMC5883L_RESOLUTION_660     (float) 1.52
#define HMC5883L_RESOLUTION_440     (float) 2.27
#define HMC5883L_RESOLUTION_390     (float) 2.56
#define HMC5883L_RESOLUTION_330     (float) 3.03
#define HMC5883L_RESOLUTION_230     (float) 4.35

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  The function read data from device's register.
  * @param Handle The handle of the hmc5883l ic.
  * @param Address The Address of the register.
  * @param DataR The buffer's pointer to read the data.
  * @param size The size of the data that needs to read.
  * @retval HMC5883_Status_t
  */
HMC5883L_Status_t HMC5883L_Read (HMC5883L_Handle_t *Handle,
                                        uint8_t Address, uint8_t *DataR, uint8_t size);

/**
  * @brief  The function write data to device's register.
  * @param Handle The handle type of the hmc5883L ic.
  * @param Address The Address of the register.
  * @param DataW The buffer's pointer to write the data.
  * @param size The size of the data that needs to write.
  * @retval HMC5883_State_t
  */
HMC5883L_Status_t HMC5883L_Write (HMC5883L_Handle_t *Handle,
                                         uint8_t Address, uint8_t *DataW, uint8_t size);

/**
 * @brief Find the Resolution rely on the Gain configuration.
 * @param GainConfig The configuration of the gain attribute.
 * @retval float The gain at float type.
 */
float HMC5883L_Get_Gain (uint8_t GainConfig);

/**
 * @brief Check the DRY header. Must be check before read the value.
 * @param Handle The Handle type of the library.
 * @retval bool true means ready, false means not ready.
 */
bool HMC5883L_Check_DRY (HMC5883L_Handle_t *Handle);

/**
 * @brief Read the raw data at three axis.
 * @param Handle The pointer of the handle type.
 * @retval HMC5883L_Status_t The status of the HMC5883.
 */
HMC5883L_Status_t HMC5883L_Get_Raw (HMC5883L_Handle_t *Handle);

/**
 * @brief Read the scaled data at three axis in mg.
 * @param Handle The pointer of the handle type.
 * @retval HMC5883L_Status_t The status of the HMC5883.
 */
HMC5883L_Status_t HMC5883L_Get_Scaled (HMC5883L_Handle_t *Handle);

/**
 * @brief Initialize the HMC5883L.
 * @param Handle The handle of the initialization.
 * @param I2c The handle type of the I2C.
 * @retval HMC5883L_State_t The state of the IC after initialize.
 */
HMC5883L_State_t HMC5883L_Init (HMC5883L_Handle_t *Handle, I2C_HandleTypeDef *I2c);

#endif      /* End of File */
