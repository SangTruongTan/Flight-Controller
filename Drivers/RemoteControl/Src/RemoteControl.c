/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : RemoteControl.c
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
#include "RemoteControl.h"

/* Private includes ----------------------------------------------------------*/
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CONTROL_ROWS 12
#define CONTROL_COLUMNS 16
#define CONTROL_CMD_MAX_LENGTH 64
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ControlHandler_t *ControlHandler;

/* Private function prototypes -----------------------------------------------*/
int Control_Parse(uint8_t (*TempBuffer)[CONTROL_COLUMNS], uint8_t *Source,
                  const char Deli);

ControlStatus_t Decode_CMD(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[CONTROL_COLUMNS]);

ControlStatus_t Decode_PID(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[CONTROL_COLUMNS]);

ControlStatus_t Decode_Mode(ControlMode_t *Mode,
                            uint8_t (*StructBuffer)[CONTROL_COLUMNS]);

/* Private function definations ----------------------------------------------*/
/**
 * @brief Parse the string with the delimiter.
 * @param TempBuffer The temporary buffer.
 * @param Source The Source of the String.
 * @param Deli the delimiter.
 * @retval int The number of split string.
 */
int Control_Parse(uint8_t (*TempBuffer)[CONTROL_COLUMNS], uint8_t *Source,
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
 * @brief Decode the CMD message.
 * @param Control The Control Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_CMD(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[CONTROL_COLUMNS]) {
    int index = 1;
    if (strcmp((const char *)&StructBuffer[0], "CMD") != 0)
        return CONTROL_ERROR;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "T") == 0) {
            int Thrust = atoi((const char *)&StructBuffer[index + 1]);
            if (Thrust >= 1000 && Thrust <= 2000)
                Control->JoyStick.Thrust = Thrust;
        } else if (strcmp((const char *)&StructBuffer[index], "R") == 0) {
            int Roll = atoi((const char *)&StructBuffer[index + 1]);
            if (Roll >= 1000 && Roll <= 2000) Control->JoyStick.Roll = Roll;
        } else if (strcmp((const char *)&StructBuffer[index], "P") == 0) {
            int Pitch = atoi((const char *)&StructBuffer[index + 1]);
            if (Pitch >= 1000 && Pitch <= 2000) Control->JoyStick.Pitch = Pitch;
        } else if (strcmp((const char *)&StructBuffer[index], "Y") == 0) {
            int Yaw = atoi((const char *)&StructBuffer[index + 1]);
            if (Yaw >= 1000 && Yaw <= 2000) Control->JoyStick.Yaw = Yaw;
        } else if (strcmp((const char *)&StructBuffer[index], "H") == 0) {
            if (StructBuffer[index + 1][0] == 'T') {
                Control->HeadingLock = true;
            } else {
                Control->HeadingLock = false;
            }
        }
        index += 2;
    }
    // Detect Start Stop State
    if (Control->JoyStick.Thrust < 1030 && Control->JoyStick.Yaw < 1080 &&
        ControlHandler->Mode == BLOCK_MODE) {
        ControlHandler->Mode = MANUAL_MODE;
        const char *Buffer = "MOD,MANUAL\n";
        HAL_UART_Transmit(ControlHandler->Serial->huart, (uint8_t *)Buffer,
                          strlen(Buffer), 100);
    } else if (Control->JoyStick.Thrust < 1030 &&
               Control->JoyStick.Yaw > 1900 &&
               ControlHandler->Mode != BLOCK_MODE) {
        ControlHandler->Mode = BLOCK_MODE;
        const char *Buffer1 = "MOD,BLOCK\n";
        HAL_UART_Transmit(ControlHandler->Serial->huart, (uint8_t *)Buffer1,
                          strlen(Buffer1), 100);
    }
    return CONTROL_OK;
}

/**
 * @brief Decode the PID message.
 * @param Control The Control Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_PID(ControlInfo_t *Control,
                           uint8_t (*StructBuffer)[CONTROL_COLUMNS]) {
    int index = 1;
    if (strcmp((const char *)&StructBuffer[0], "PID") != 0)
        return CONTROL_ERROR;
    while (StructBuffer[index][0] != '\0') {
        if (strcmp((const char *)&StructBuffer[index], "PR") == 0) {
            Control->ControlPid.Roll.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IR") == 0) {
            Control->ControlPid.Roll.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DR") == 0) {
            Control->ControlPid.Roll.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PP") == 0) {
            Control->ControlPid.Pitch.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IP") == 0) {
            Control->ControlPid.Pitch.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DP") == 0) {
            Control->ControlPid.Pitch.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PY") == 0) {
            Control->ControlPid.Yaw.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IY") == 0) {
            Control->ControlPid.Yaw.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DY") == 0) {
            Control->ControlPid.Yaw.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PA") == 0) {
            Control->ControlPid.Altitude.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IA") == 0) {
            Control->ControlPid.Altitude.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DA") == 0) {
            Control->ControlPid.Altitude.D =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "PG") == 0) {
            Control->ControlPid.Gps.P =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "IG") == 0) {
            Control->ControlPid.Gps.I =
                atof((const char *)&StructBuffer[index + 1]);
        } else if (strcmp((const char *)&StructBuffer[index], "DG") == 0) {
            Control->ControlPid.Gps.D =
                atof((const char *)&StructBuffer[index + 1]);
        }
        index += 2;
    }
    return CONTROL_OK;
}

/**
 * @brief Decode the Mode message.
 * @param Mode The Mode Pointer.
 * @param StructBuffer The Structural Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Decode_Mode(ControlMode_t *Mode,
                            uint8_t (*StructBuffer)[CONTROL_COLUMNS]) {
    if (strcmp((const char *)&StructBuffer[0], "MOD") != 0)
        return CONTROL_ERROR;
    if (strcmp((const char *)&StructBuffer[1], "MANUAL") == 0) {
        *Mode = MANUAL_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "ALTITUDE") == 0) {
        *Mode = ALTITUDE_HOLD_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "GPS") == 0) {
        *Mode = GPS_HOLD_MODE;
    } else if (strcmp((const char *)&StructBuffer[1], "HOME") == 0) {
        *Mode = RETURN_HOME_MODE;
    }
    return CONTROL_OK;
}

/* Function definations _-----------------------------------------------------*/
/**
 * @brief The Initialize for the Remote Control.
 * @param Handler The pointer of the Control Handler.
 * @param Init The Init struct.
 * @retval void
 */
void Control_Init(ControlHandler_t *Handler, ControlInit_t Init) {
    // Copy the user Init struct to the Init struct of the Handler.
    memcpy(&Handler->Init, &Init, sizeof(Init));
    // Copy the Parameter
    memcpy(&Handler->Mode, &Init.Mode, sizeof(Init.Mode));
    memcpy(&Handler->Control.ControlPid, &Init.ControlPid,
           sizeof(Init.ControlPid));
    memcpy(&Handler->Control.JoyStick, &Init.Joystick, sizeof(Init.Joystick));
    Handler->Serial = Init.Serial;
    // Copy the sensors parameter attribute
    Handler->SensorsParameter = Init.SensorsParameter;
    // Assign the Handler to the Handler of the file
    ControlHandler = Handler;
}

/**
 * @brief Process the data in the Ring Buffer.
 * @retval ControlStatus_t
 */
ControlStatus_t Control_Process(void) {
    uint8_t *Buffer = NULL;
    uint8_t *Buffer1 = NULL;
    uint8_t StructBuffer[CONTROL_ROWS][CONTROL_COLUMNS];
    ControlStatus_t Status = CONTROL_ERROR;
    if (Detect_Char(ControlHandler->Serial, '\n')) {
        Buffer = malloc(128);
        Buffer1 = malloc(128);
        memset(Buffer1, '\0', 128);
        memset(Buffer, '\0', 128);
        int Length =
            Get_String_NonBlocking(ControlHandler->Serial, Buffer, '\n');
        if (Length > 0 && Length < CONTROL_CMD_MAX_LENGTH) {
            memmove(Buffer1, Buffer, Length);
            strcpy((char *)Buffer1, (char *)Buffer);
            Control_Parse(StructBuffer, Buffer, ',');
            if (strcmp((const char *)&StructBuffer[0], "CMD") == 0) {
                Status = Decode_CMD(&ControlHandler->Control, StructBuffer);
            }
            if (strcmp((const char *)&StructBuffer[0], "PID") == 0) {
                Status = Decode_PID(&ControlHandler->Control, StructBuffer);
                Buffer1[strlen((char *)Buffer1)] = '\n';
                HAL_UART_Transmit(ControlHandler->Serial->huart, Buffer1,
                                  strlen((char *)Buffer1), 100);
            }
            if (strcmp((const char *)&StructBuffer[0], "MOD") == 0) {
                Status = Decode_Mode(&ControlHandler->Mode, StructBuffer);
                Buffer1[strlen((char *)Buffer1)] = '\n';
                HAL_UART_Transmit(ControlHandler->Serial->huart, Buffer1,
                                  strlen((char *)Buffer1), 100);
            }
            ControlHandler->Status = Status;
        }
        // Put the string want to forward
        free(Buffer);
        free(Buffer1);
    }
    return Status;
}

void Control_Feedback_VBat(void) {
    char *Buffer;
    Buffer = malloc(32);
    sprintf(Buffer, "ACK,B,%04.2f\n", ControlHandler->SensorsParameter->VBat);
    HAL_UART_Transmit(ControlHandler->Serial->huart, (uint8_t *)Buffer,
                      strlen(Buffer), 100);
    free(Buffer);
}

void Control_Feedback_Angle(void) {
    char *Buffer;
    Buffer = malloc(32);
    sprintf(Buffer, "ACK,R,%04.1f,P,%04.1f,Y,%04.1f\n",
            ControlHandler->SensorsParameter->Roll,
            ControlHandler->SensorsParameter->Pitch,
            ControlHandler->SensorsParameter->Yaw);
    HAL_UART_Transmit(ControlHandler->Serial->huart, (uint8_t *)Buffer,
                      strlen(Buffer), 100);
    free(Buffer);
}

void Control_Feedback_Mode(void) {
    char *Buffer;
    char Mode[10];
    Buffer = malloc(32);
    switch (ControlHandler->Mode) {
        case BLOCK_MODE:
            sprintf(Mode, "BLOCK");
            break;
        case MANUAL_MODE:
            sprintf(Mode, "MANUAL");
            break;
        case ALTITUDE_HOLD_MODE:
            sprintf(Mode, "ALTITUDE");
            break;
        case GPS_HOLD_MODE:
            sprintf(Mode, "GPS");
            break;
        case RETURN_HOME_MODE:
            sprintf(Mode, "HOME");
            break;
        case LOST_MODE:
            sprintf(Mode, "LOST");
            break;
        default:
            break;
    }
    sprintf(Buffer, "MOD,%s\n", Mode);
    HAL_UART_Transmit(ControlHandler->Serial->huart, (uint8_t *)Buffer,
                      strlen(Buffer), 100);
    free(Buffer);
}
