/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : UbloxGps.c
 * @brief          : The source file of the library
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
#include "UbloxGps.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function definations ----------------------------------------------*/
/**
 * @brief Parse the string with the delimiter.
 * @param TempBuffer The temporary buffer.
 * @param Source The Source of the String.
 * @param Deli the delimiter.
 * @retval int The number of split string.
 */
int Gps_Parse(uint8_t (*TempBuffer)[COLUMNS], uint8_t *Source,
              const char Deli) {
    int length = 0;
    char *Head = malloc(32);
    Head = strtok((char *)Source, &Deli);
    while (Head != NULL) {
        strcpy((char *)&(TempBuffer)[length], Head);
        length++;
        Head = strtok(NULL, &Deli);
    }
    free(Head);
    return length;
}

/**
 * @brief Convert the Raw data of the Latitude or the Longitude to the float
 * value.
 * @param Buffer The buffer data.
 * @retval float
 */
float Convert_Position_Data(const char *Buffer) {
    float val = atof(Buffer);
    float DegreeAndMinute = val / 100.0;
    float truncRound = trunc(DegreeAndMinute);
    float minute = (DegreeAndMinute - truncRound) * 100;
    float Result = truncRound + minute / 60.0;
    return Result;
}

/**
 * @brief Decode the GGA data.
 * @param GpsData The pointer of the Gps data struct.
 * @param Buffer The Buffer store the GGA data.
 * @retval GpsStatus
 */
GpsStatus_t Gps_Decode_GGA(GpsData_t *GpsData, uint8_t (*Buffer)[COLUMNS]) {
    // Check the title is GPGGA or not.
    if (strcmp((const char *)&Buffer[0], GNGGA) != 0) return GPS_ERROR;
    // Check Time
    if (strlen((const char *)&Buffer[1]) < 6) return GPS_NOTIME;
    int TimeRaw = atoi((const char *)Buffer[1]);
    GpsData->Time.Hour = TimeRaw / 10000;
    GpsData->Time.Minute = TimeRaw / 100 % 100;
    GpsData->Time.Seconds = TimeRaw % 100;
    // CheckFix
    if (Buffer[6][0] != '1') {
        GpsData->Fixed = false;
        return GPS_NOTFIXED;
    }
    GpsData->Fixed = true;
    // Get Latitude and Longitude
    if (strlen((const char *)&Buffer[2]) < 6 ||
        strlen((const char *)&Buffer[4]) < 6)
        return GPS_NODATA;
    // Calculate the Latitude data
    GpsData->Latitude = Convert_Position_Data((const char *)&Buffer[2]);
    GpsData->Longitude = Convert_Position_Data((const char *)&Buffer[4]);
    GpsData->Lat_North = Buffer[3][0];
    GpsData->Lat_East = Buffer[5][0];
    GpsData->Sattellites = atoi((const char *)&Buffer[7]);
    return GPS_OK;
}

/**
 * @brief Gps processing the data.
 * @param GpsHandler The pointer of the Gps Handler.
 * @retval GpsStatus_t
 */
bool Gps_Process(GpsHandler_t *GpsHandler) {
    GpsStatus_t Status = GPS_WAIT_BUFFER;
    uint8_t StructData[ROWS][COLUMNS];
    uint8_t *DataBuffer;
    if(Detect_Char(GpsHandler->Serial, '\n') == true) {
        DataBuffer = malloc(128);
        while (Get_String_NonBlocking(GpsHandler->Serial, DataBuffer, '\n') > 0) {
            int length = Gps_Parse(StructData, DataBuffer, ',');
            if (length) {
                if (strcmp((const char *)&StructData[0], GNGGA) == 0) {
                    Status = Gps_Decode_GGA(&GpsHandler->Position, StructData);
                }
            }
            memset(DataBuffer, '\0', 128);
        }
        free(DataBuffer);
    }
    return Status;
}
