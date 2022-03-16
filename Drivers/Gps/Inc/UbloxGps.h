/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UbloxGps.h
  * @brief          : Header for UbloxGps.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __UBLOXGPS_H_
#define __UBLOXGPS_H_

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "uartRingBufDMA.h"
#include "stdbool.h"
#include "math.h"
/* Exported types ------------------------------------------------------------*/
typedef struct Gpstime_t {
  uint8_t Hour;
  uint8_t Minute;
  uint8_t Seconds;
} Gpstime_t;

typedef enum GpsStatus_t {
  GPS_OK = 0,
  GPS_NOTFIXED = 1,
  GPS_NOTIME = 2,
  GPS_NODATA = 3,
  GPS_ERROR = 4,
  GPS_WAIT_BUFFER = 5,
} GpsStatus_t;

typedef struct GpsData_t {
  float Longitude;
  float Latitude;
  bool Fixed;
  char Lat_East;
  char Lat_North;
  Gpstime_t Time;
  int Sattellites;
} GpsData_t;

typedef struct GpsHandler_t {
  RingHandler_t *Serial;
  GpsData_t Position;
  GpsStatus_t Status;
} GpsHandler_t;

/* Exported constants --------------------------------------------------------*/
#define ROWS 20
#define COLUMNS 32

#define GNGGA (const char *)"$GNGGA"
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

bool Gps_Process(GpsHandler_t *GpsHandler);
/* Private defines -----------------------------------------------------------*/
#endif  /* __UBLOXGPS_H_ */
