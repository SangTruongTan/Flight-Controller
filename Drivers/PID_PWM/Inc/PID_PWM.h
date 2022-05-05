/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : PID_PWM.h
 * @brief          : Header for PID_PWM.c file.
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
#ifndef __PID_PWM_H_
#define __PID_PWM_H_

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include "RemoteControl.h"
#include "ST_MPU6050.h"
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct PIDControl_t {
    int32_t Roll;
    int32_t Pitch;
    int32_t Yaw;
} PIDControl_t;

typedef struct PIDSetpoint_t {
    float Roll;
    float Pitch;
    float Yaw;
} PIDSetpoint_t;

typedef struct PIDOutput_t {
    float Error;
    float *Input;
    float *Setpoint;
    Pid_t *Pid;
    float LastError;
    float IMem;
    float Output;
} PIDOutput_t;

typedef struct PWMMOTOR_t {
    int16_t Throttle;
    int16_t Esc_1;
    int16_t Esc_2;
    int16_t Esc_3;
    int16_t Esc_4;
} PWMMOTOR_t;

typedef struct Heading_t {
    float HeadingLockDeviation;
    float LockHeading;
} Heading_t;

typedef struct PIDPWMHandle_t {
    TIM_HandleTypeDef *htim;
    ControlInfo_t *Control;
    MPU6050_handle_t *Angle;
    PIDControl_t SetPointBase;
    PIDSetpoint_t SetPoint;
    PIDOutput_t PIDRoll;
    PIDOutput_t PIDPitch;
    PIDOutput_t PIDYaw;
    PWMMOTOR_t Motors;
    Heading_t Heading;
    ControlMode_t *Mode;
} PIDPWMHandle_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void PIDPWM_Init(PIDPWMHandle_t *HandleInit);
void PIDPWM_Process();
/* Private defines -----------------------------------------------------------*/
#endif /* __PID_PWM_H */
