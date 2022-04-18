/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : PID_PWM.c
 * @brief          : Main program body
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
#include "PID_PWM.h"

/* Private includes ----------------------------------------------------------*/
#include "main.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PIDPWMHandle_t *Handle;

/* Private function prototypes -----------------------------------------------*/
void PID_Calculate(PIDOutput_t *PID);
/* Private user code ---------------------------------------------------------*/
void PIDPWM_Init(PIDPWMHandle_t *HandleInit) {
    Handle = HandleInit;
    Handle->PIDPitch.Pid = &Handle->Control->ControlPid.Pitch;
    Handle->PIDRoll.Pid = &Handle->Control->ControlPid.Roll;
    Handle->PIDYaw.Pid = &Handle->Control->ControlPid.Yaw;
    Handle->PIDPitch.Input = &Handle->Angle->GyroInput.pitch;
    Handle->PIDPitch.Setpoint = &Handle->SetPoint.Pitch;
    Handle->PIDRoll.Input = &Handle->Angle->GyroInput.roll;
    Handle->PIDRoll.Setpoint = &Handle->SetPoint.Roll;
    Handle->PIDYaw.Input = &Handle->Angle->GyroInput.yaw;
    Handle->PIDYaw.Setpoint = &Handle->SetPoint.Yaw;
}

void PIDPWM_Process() {
    // Check LOST connection Mode
    if (*Handle->Mode == LOST_MODE) {
        Handle->SetPointBase.Pitch = 0;
        Handle->SetPointBase.Roll = 0;
    } else {
        Handle->SetPointBase.Pitch = Handle->Control->JoyStick.Pitch;
        Handle->SetPointBase.Roll = Handle->Control->JoyStick.Roll;
    }

    // Put heading lock here
    // Put the GPS control here
    // Limit the range
    if (Handle->SetPointBase.Pitch > 2000)
        Handle->SetPointBase.Pitch = 2000;
    else if (Handle->SetPointBase.Pitch < 1000)
        Handle->SetPointBase.Pitch = 1000;
    if (Handle->SetPointBase.Roll > 2000)
        Handle->SetPointBase.Roll = 2000;
    else if (Handle->SetPointBase.Roll < 1000)
        Handle->SetPointBase.Roll = 1000;
    // Calculate the set point
    // The PID set point in degrees per second is determined by the roll
    // receiver input.
    // In the case of deviding by 3 the max roll rate is approx 164 degrees per
    // second ( (500-8)/3 = 164d/s ).
    Handle->SetPoint.Roll = 0;
    if (Handle->SetPointBase.Roll > 1508)
        Handle->SetPoint.Roll = Handle->SetPointBase.Roll - 1508;
    else if (Handle->SetPointBase.Roll < 1492)
        Handle->SetPoint.Roll = Handle->SetPointBase.Roll - 1492;
    // Subtract the angle correction
    Handle->SetPoint.Roll -= Handle->Angle->AngleAjust.roll;
    Handle->SetPoint.Roll /= 3.0;

    // The PID set point in degrees per second is determined by the pitch
    // receiver input.
    // In the case of deviding by 3 the max pitch rate is approx 164 degrees per
    // second ( (500-8)/3 = 164d/s ).
    if (Handle->SetPointBase.Pitch > 1508)
        Handle->SetPoint.Pitch = Handle->SetPointBase.Pitch - 1508;
    else if (Handle->SetPointBase.Pitch < 1492)
        Handle->SetPoint.Pitch = Handle->SetPointBase.Pitch - 1492;
    // Subtract the angle correction
    Handle->SetPoint.Pitch -= Handle->Angle->AngleAjust.pitch;
    Handle->SetPoint.Pitch /= 3.0;

    // The PID set point in degrees per second is determined by the yaw
    // receiver input.
    // In the case of deviding by 3 the max yaw rate is approx 164 degrees per
    // second ( (500-8)/3 = 164d/s ).
    Handle->SetPoint.Yaw = 0;
    // Do not calculate Yaw when the motor is off.
    if (Handle->Control->JoyStick.Thrust > 1050) {
        if (Handle->Control->JoyStick.Yaw > 1508)
            Handle->SetPoint.Yaw = (Handle->Control->JoyStick.Yaw - 1508) / 3.0;
        else if (Handle->Control->JoyStick.Yaw < 1492)
            Handle->SetPoint.Yaw = (Handle->Control->JoyStick.Yaw - 1492) / 3.0;
    }
    // Roll PID's calculation.
    PID_Calculate(&Handle->PIDRoll);
    // Pitch PID's calculation.
    PID_Calculate(&Handle->PIDPitch);
    // Yaw PID's calculation.
    PID_Calculate(&Handle->PIDYaw);
    // Check for I controller when the Drone isn't start yet.
    if (Handle->Motors.Throttle < 1010) {
        Handle->PIDRoll.IMem = 0;
        Handle->PIDPitch.IMem = 0;
        Handle->PIDYaw.IMem = 0;
    }
    // Process for PWM
    // Detect the Drone Crash
    if ((abs(Handle->Angle->GyroAxis.pitch) > 75 ||
         abs(Handle->Angle->GyroAxis.roll) > 75) &&
        *Handle->Mode == LOST_MODE)
        *Handle->Mode = BLOCK_MODE;
    // Put the Throttle calculation here
    // Check LOST connection Mode
    if (*Handle->Mode == LOST_MODE) {
        Handle->Motors.Throttle = 1080;
    }

    else
        Handle->Motors.Throttle = Handle->Control->JoyStick.Thrust;

    if (*Handle->Mode != BLOCK_MODE &&
        Handle->Control->JoyStick.Thrust > 1010) {
        if (Handle->Motors.Throttle > 1800) Handle->Motors.Throttle = 1800;
        Handle->Motors.Esc_1 = Handle->Motors.Throttle -
                               Handle->PIDPitch.Output +
                               Handle->PIDRoll.Output - Handle->PIDYaw.Output;
        Handle->Motors.Esc_2 = Handle->Motors.Throttle +
                               Handle->PIDPitch.Output +
                               Handle->PIDRoll.Output + Handle->PIDYaw.Output;
        Handle->Motors.Esc_3 = Handle->Motors.Throttle +
                               Handle->PIDPitch.Output -
                               Handle->PIDRoll.Output - Handle->PIDYaw.Output;
        Handle->Motors.Esc_4 = Handle->Motors.Throttle -
                               Handle->PIDPitch.Output -
                               Handle->PIDRoll.Output + Handle->PIDYaw.Output;
        if (Handle->Motors.Esc_1 < 1000) {
            Handle->Motors.Esc_1 = 1000;
        } else if (Handle->Motors.Esc_1 > 2000) {
            Handle->Motors.Esc_1 = 2000;
        }
        if (Handle->Motors.Esc_2 < 1000) {
            Handle->Motors.Esc_2 = 1000;
        } else if (Handle->Motors.Esc_2 > 2000) {
            Handle->Motors.Esc_2 = 2000;
        }
        if (Handle->Motors.Esc_3 < 1000) {
            Handle->Motors.Esc_3 = 1000;
        } else if (Handle->Motors.Esc_3 > 2000) {
            Handle->Motors.Esc_3 = 2000;
        }
        if (Handle->Motors.Esc_4 < 1000) {
            Handle->Motors.Esc_4 = 1000;
        } else if (Handle->Motors.Esc_4 > 2000) {
            Handle->Motors.Esc_4 = 2000;
        }
        if (Handle->Control->JoyStick.Thrust > 1010) {
            if (Handle->Motors.Esc_1 < 1100) Handle->Motors.Esc_1 = 1100;
            if (Handle->Motors.Esc_2 < 1100) Handle->Motors.Esc_2 = 1100;
            if (Handle->Motors.Esc_3 < 1100) Handle->Motors.Esc_3 = 1100;
            if (Handle->Motors.Esc_4 < 1100) Handle->Motors.Esc_4 = 1100;
        }
    } else {
        Handle->Motors.Esc_1 = 1000;
        Handle->Motors.Esc_2 = 1000;
        Handle->Motors.Esc_3 = 1000;
        Handle->Motors.Esc_4 = 1000;
    }
// Put the value to the timer output PWM
#if MOTOR_CONFIG == 0
    Handle->htim->Instance->CCR1 = Handle->Motors.Esc_1;
    Handle->htim->Instance->CCR2 = Handle->Motors.Esc_2;
    Handle->htim->Instance->CCR3 = Handle->Motors.Esc_3;
    Handle->htim->Instance->CCR4 = Handle->Motors.Esc_4;
#elif MOTOR_CONFIG == 1
    if (Handle->Control->JoyStick.Thrust > 1700) {
        Handle->Control->JoyStick.Thrust = 2000;
    } else if (Handle->Control->JoyStick.Thrust < 1030) {
        Handle->Control->JoyStick.Thrust = 1000;
    }
    Handle->htim->Instance->CCR1 = Handle->Control->JoyStick.Thrust;
    Handle->htim->Instance->CCR2 = Handle->Control->JoyStick.Thrust;
    Handle->htim->Instance->CCR3 = Handle->Control->JoyStick.Thrust;
    Handle->htim->Instance->CCR4 = Handle->Control->JoyStick.Thrust;
#endif
    // Handle->htim->Instance->CNT = 5000;
}

void PID_Calculate(PIDOutput_t *PID) {
    PID->Error = *PID->Input - *PID->Setpoint;
    PID->IMem += PID->Pid->I * PID->Error;
    if (PID->IMem > PID->Pid->MaxPID)
        PID->IMem = PID->Pid->MaxPID;
    else if (PID->IMem < PID->Pid->MaxPID * -1)
        PID->IMem = PID->Pid->MaxPID * -1;
    PID->Output = PID->Pid->P * PID->Error + PID->IMem +
                  PID->Pid->D * (PID->Error - PID->LastError);
    if (PID->Output > PID->Pid->MaxPID)
        PID->Output = PID->Pid->MaxPID;
    else if (PID->Output < PID->Pid->MaxPID * -1)
        PID->Output = PID->Pid->MaxPID * -1;
    PID->LastError = PID->Error;
}