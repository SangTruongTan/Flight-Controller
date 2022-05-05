/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : RemoteControl.h
 * @brief          : Header for RemoteControl.c file.
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
#ifndef __REMOTECONTROL_H_
#define __REMOTECONTROL_H_

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "sensors.h"
#include "uartRingBufDMA.h"

/* Exported types ------------------------------------------------------------*/
typedef enum ControlStatus_t {
    CONTROL_OK = 0,
    CONTROL_ERROR_SERIAL = 1,
    CONTROL_ERROR = 2,
} ControlStatus_t;

typedef struct Joystick_t {
    uint16_t Thrust;
    int Roll;
    int Pitch;
    int Yaw;
} Joystick_t;

typedef struct Pid_t {
    float P;
    float I;
    float D;
    float MaxPID;
} Pid_t;

typedef enum ControlMode_t {
    BLOCK_MODE = 0,
    MANUAL_MODE = 1,
    ALTITUDE_HOLD_MODE = 2,
    GPS_HOLD_MODE = 3,
    RETURN_HOME_MODE = 4,
    LOST_MODE = 5,
} ControlMode_t;

typedef struct ControlPid_t {
    Pid_t Roll;
    Pid_t Pitch;
    Pid_t Yaw;
    Pid_t Altitude;
    Pid_t Gps;
} ControlPid_t;

typedef struct ControlInfo_t {
    Joystick_t JoyStick;
    ControlPid_t ControlPid;
    bool HeadingLock;
} ControlInfo_t;

typedef struct ControlInit_t {
    RingHandler_t *Serial;
    ControlPid_t ControlPid;
    ControlMode_t Mode;
    Joystick_t Joystick;
    SensorParameters_t *SensorsParameter;
} ControlInit_t;

typedef struct ControlHandler_t {
    ControlInit_t Init;
    RingHandler_t *Serial;
    ControlMode_t Mode;
    ControlInfo_t Control;
    ControlStatus_t Status;
    SensorParameters_t *SensorsParameter;
} ControlHandler_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Control_Init(ControlHandler_t *Handler, ControlInit_t Init);
ControlStatus_t Control_Process(void);
void Control_Feedback_VBat(void);
void Control_Feedback_Mode(void);
void Control_Feedback_Angle(void);

/* Exported defines
 * -----------------------------------------------------------*/

#endif /* __REMOTECONTROL_H_ */
