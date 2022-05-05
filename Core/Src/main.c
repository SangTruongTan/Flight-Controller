/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RemoteControl.h"
// FreeRTOS
#include "FreeRTOS.h"
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "stdio.h"
#include "task.h"
#include "timers.h"
// MEMS
#include "PID_PWM.h"
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define dt 0.004f
#define TUNE_PID 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t count = 0;

TaskHandle_t DefaultTask;
TaskHandle_t LoopTask;
TaskHandle_t GpsTask;
TaskHandle_t ControlTask;
TaskHandle_t FeedbackTask;
TaskHandle_t CalibrationTask;

RingBuffer_t Ring;
Sensor_handle_t sensors;
ControlHandler_t Control;
ControlInit_t ControlInit;
PIDPWMHandle_t PID;

uint16_t AdcBatValue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
int _write(int file, char *outgoing, int len);
void Default_task(void *pvParameters);
void Loop_task(void *pvParameters);
void GPS_task(void *pvParameters);
void Control_task(void *pvParameters);
void Feedback_task(void *pvParameters);
void Calibration_task(void *pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();
    /* USER CODE BEGIN 2 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&AdcBatValue, 1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
#if MOTOR_CONFIG == 0
    htim5.Instance->CCR1 = 1000;
    htim5.Instance->CCR2 = 1000;
    htim5.Instance->CCR3 = 1000;
    htim5.Instance->CCR4 = 1000;
#else
    htim5.Instance->CCR1 = 2000;
    htim5.Instance->CCR2 = 2000;
    htim5.Instance->CCR3 = 2000;
    htim5.Instance->CCR4 = 2000;
#endif
    HAL_Delay(200);
    // Init for Ring Buffer. Must be called first
    Ring.Ring1.enable = true;
    Ring.Ring2.enable = true;
    Ring.Ring3.enable = true;
    Ring.Ring1.hdma = &hdma_usart1_rx;
    Ring.Ring1.huart = &huart1;
    Ring.Ring2.hdma = &hdma_usart3_rx;
    Ring.Ring2.huart = &huart3;
    Ring.Ring3.hdma = &hdma_uart5_rx;
    Ring.Ring3.huart = &huart5;
    Ring.GetTime = xTaskGetTickCount;
    // Init for sensors
    sensors.hmchi2c = &hi2c2;
    sensors.mpuhi2c = &hi2c1;
    sensors.mshi2c = &hi2c1;
    sensors.gpsRing = &Ring.Ring3;
    sensors.msHandler.wait = vTaskDelay;
    // Init compass calibration
    // sensors.hmcHandler.OffsetScale.offset_x = -128;
    // sensors.hmcHandler.OffsetScale.offset_y = -160;
    // sensors.hmcHandler.OffsetScale.offset_z = 117;
    // sensors.hmcHandler.OffsetScale.scale_y = 1.051408;
    // sensors.hmcHandler.OffsetScale.scale_z = 1.149933;
    sensors.hmcHandler.OffsetScale.compass_cal_values[0] = -302;
    sensors.hmcHandler.OffsetScale.compass_cal_values[1] = 557;
    sensors.hmcHandler.OffsetScale.compass_cal_values[2] = -256;
    sensors.hmcHandler.OffsetScale.compass_cal_values[3] = 561;
    sensors.hmcHandler.OffsetScale.compass_cal_values[4] = -476;
    sensors.hmcHandler.OffsetScale.compass_cal_values[5] = 271;
    sensors.hmcHandler.OffsetScale.declination = 0;
    // Calibration value for gyroscope
    sensors.mpuHandler.GyroOffset.x = 114;
    sensors.mpuHandler.GyroOffset.y = 10;
    sensors.mpuHandler.GyroOffset.z = -45;
    // Init for calibration Task
    sensors.Calibration.GetTime = &xTaskGetTickCount;
    sensors.Calibration.wait = &vTaskDelay;
    sensors.Calibration.waitUntil = &vTaskDelayUntil;
    sensors.Calibration.ThrustChannel = &Control.Control.JoyStick.Thrust;
    // Init for ADC Battery Voltage reading
    sensors.AdcBat = &AdcBatValue;
// Init for Remote Control
#if MOTOR_CONFIG == 0
    ControlInit.Mode = BLOCK_MODE;
#else
    ControlInit.Mode = MANUAL_MODE;
#endif
    ControlInit.SensorsParameter = &sensors.Angle;
    ControlInit.Serial = &Ring.Ring2;
    ControlInit.Joystick.Pitch = 1500;
    ControlInit.Joystick.Roll = 1500;
    ControlInit.Joystick.Yaw = 1500;
    ControlInit.Joystick.Thrust = 1000;
#if TUNE_PID == 0
    ControlInit.ControlPid.Pitch.P = 1.3;
    ControlInit.ControlPid.Pitch.I = 0.008;
    ControlInit.ControlPid.Pitch.D = 20.0;
    ControlInit.ControlPid.Roll.P = 1.3;
    ControlInit.ControlPid.Roll.I = 0.008;
    ControlInit.ControlPid.Roll.D = 20.0;
    ControlInit.ControlPid.Yaw.P = 4;
    ControlInit.ControlPid.Yaw.I = 0.02;
    ControlInit.ControlPid.Yaw.D = 0.0;
#else
#endif
    ControlInit.ControlPid.Pitch.MaxPID = 400;
    ControlInit.ControlPid.Roll.MaxPID = 400;
    ControlInit.ControlPid.Yaw.MaxPID = 400;
    // Initialize for PID
    PID.htim = &htim5;
    PID.Control = &Control.Control;
    PID.Angle = &sensors.mpuHandler;
    PID.Mode = &Control.Mode;
    // Calibration value Initialize
    Ring_Init(&Ring);
    // The Remote Control Function must be called after the Ring Init
    Control_Init(&Control, ControlInit);
    // Init function for the sensors
    sensors_init(&sensors);
    // Init PID
    PIDPWM_Init(&PID);

    if (sensors.status != SENSOR_OK) {
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
        printf("Sensors error:%d\r\n", sensors.status);
        while (1)
            ;
    }
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
    xTaskCreate(&Default_task, "Default", 512, NULL, 1, &DefaultTask);
    // Start Scheduler
    vTaskStartScheduler();
    /* USER CODE END 2 */

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 128;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Configure the global features of the ADC (Clock, Resolution, Data
     * Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in
     * the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {
    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 64;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 20000;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
    HAL_TIM_MspPostInit(&htim5);
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {
    /* USER CODE BEGIN UART5_Init 0 */

    /* USER CODE END UART5_Init 0 */

    /* USER CODE BEGIN UART5_Init 1 */

    /* USER CODE END UART5_Init 1 */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 9600;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART5_Init 2 */

    /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin,
                      GPIO_PIN_SET);

    /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
    GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : BTN1_Pin BTN2_Pin PC8_Pin PC9_Pin */
    GPIO_InitStruct.Pin = BTN1_Pin | BTN2_Pin | PC8_Pin | PC9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *outgoing, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)outgoing, len, 100);
    return len;
}

void Default_task(void *pvParameters) {
    printf("*** Welcome to Flight Controller Software ***\r\n");
    // xTaskCreate(&GPS_task, "Gps", 512, NULL, 2, &GpsTask);
    xTaskCreate(&Loop_task, "Loop", 256, NULL, 3, &LoopTask);
    xTaskCreate(&Control_task, "Control", 256, NULL, 2, &ControlTask);
    xTaskCreate(&Feedback_task, "FeedBack", 256, NULL, 2, &FeedbackTask);
    vTaskDelete(DefaultTask);
    for (;;) {
    }
}

void Loop_task(void *pvParameters) {
    TickType_t StartTask = xTaskGetTickCount();
    uint32_t count = 0;
    for (;;) {
        count++;
        sensors_update(&sensors);
        PIDPWM_Process();
        if (sensors.status != SENSOR_OK) {
            printf("The Sensors error:%d\r\n", sensors.status);
        } else {
            if (count > 20) {
                // printf("Heading=%f\r\n",
                //        sensors.hmcHandler.CompassData.actual_heading);
                printf("y=%f,x=%f,head=%f\r\n", sensors.hmcHandler.CompassData.y_horizontal,
                                        sensors.hmcHandler.CompassData.x_horizontal,
                                        sensors.Angle.Yaw);
                count = 0;
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            } else {
            }
        }
        // Detect calibration
        if (Control.Control.JoyStick.Thrust < 1020) {
            if (Control.Control.JoyStick.Yaw > 1900 &&
                Control.Control.JoyStick.Roll < 1080 &&
                Control.Control.JoyStick.Pitch < 1080) {
                xTaskCreate(&Calibration_task, "Calibration", 256, NULL, 3,
                            &CalibrationTask);
                vTaskSuspend(NULL);
                // Wait Until the calibration task resume this task again
                StartTask = xTaskGetTickCount();
            }
        }
        vTaskDelayUntil(&StartTask, 4);
    }
}

void GPS_task(void *pvParameters) {
    TickType_t StartTask = xTaskGetTickCount();
    for (;;) {
        if (Sensor_Gps_Update(&sensors) == SENSOR_OK) {
            GpsHandler_t GpsHandler = sensors.gpsHandler;
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            printf("Time:%dh %dm %ds\r\n", GpsHandler.Position.Time.Hour,
                   GpsHandler.Position.Time.Minute,
                   GpsHandler.Position.Time.Seconds);
            printf("Lat:%f, Long:%f, Fixed:%d, Satellites:%d\r\n",
                   GpsHandler.Position.Latitude, GpsHandler.Position.Longitude,
                   GpsHandler.Position.Fixed, GpsHandler.Position.Sattellites);
        } else if (sensors.gpsHandler.Status == GPS_NOTFIXED) {
            GpsHandler_t GpsHandler = sensors.gpsHandler;
            printf("Time:%dh %dm %ds\r\n", GpsHandler.Position.Time.Hour,
                   GpsHandler.Position.Time.Minute,
                   GpsHandler.Position.Time.Seconds);
        } else {
            printf("Error:%d\r\n", sensors.gpsHandler.Status);
        }
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        vTaskDelayUntil(&StartTask, 1000);
    }
}

void Control_task(void *pvParameters) {
    static int count = 0;
    TickType_t StartTask = xTaskGetTickCount();
    static int LostMonitoring = 0;
    for (;;) {
        ControlStatus_t Status = Control_Process();
        LostMonitoring++;
        if (Status == CONTROL_OK) {
            count++;
            if (count > 10) {
                count = 0;
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            }
            LostMonitoring = 0;
        }
        // Lost Connection decide
        if (LostMonitoring > 100 && Control.Mode != BLOCK_MODE) {
            Control.Mode = LOST_MODE;
        }
        vTaskDelayUntil(&StartTask, 10);
    }
}

void Feedback_task(void *pvParameters) {
    uint8_t AckId = 0;
    TickType_t StartTask = xTaskGetTickCount();
    vTaskDelay(200);
    for (;;) {
        if (AckId == 0) {
            Control_Feedback_Mode();
            AckId++;
        } else if (AckId == 1) {
            Control_Feedback_VBat();
            AckId++;
        } else if (AckId == 2) {
            Control_Feedback_Angle();
            AckId = 0;
        }
        vTaskDelayUntil(&StartTask, 200);
    }
}

void Calibration_task(void *pvParameters) {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin | LED4_Pin,
                      GPIO_PIN_SET);
    for (int i = 0; i < 10; i++) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        vTaskDelay(500);
    }
    for (;;) {
        if (Control.Control.JoyStick.Thrust > 1800 &&
            Control.Control.JoyStick.Yaw < 1060) {  // Yaw right
            sensors.Calibration.StartTask = xTaskGetTickCount();
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin,
                              GPIO_PIN_SET);
            Sensor_Gyro_Calibration(&sensors);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin,
                              GPIO_PIN_SET);
            while (Control.Control.JoyStick.Thrust >= 1030) {
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                vTaskDelay(500);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                vTaskDelay(200);
            }
        }
        if (Control.Control.JoyStick.Thrust > 1800 &&
            Control.Control.JoyStick.Yaw > 1900) {  // Yaw left
            sensors.Calibration.StartTask = xTaskGetTickCount();
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin,
                              GPIO_PIN_SET);
            Sensor_Compass_Calibration(&sensors);
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin | LED4_Pin,
                              GPIO_PIN_SET);
            printf("Compass:sy=%f,sz=%f,ox=%d,oy=%d,oz=%d\r\n",
                   sensors.hmcHandler.OffsetScale.scale_y,
                   sensors.hmcHandler.OffsetScale.scale_z,
                   sensors.hmcHandler.OffsetScale.offset_x,
                   sensors.hmcHandler.OffsetScale.offset_y,
                   sensors.hmcHandler.OffsetScale.offset_z);
            while (Control.Control.JoyStick.Thrust >= 1030) {
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
                vTaskDelay(1000);
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
                vTaskDelay(500);
            }
        }
        if (Control.Control.JoyStick.Thrust < 1030 &&
            Control.Control.JoyStick.Yaw > 1900) {  // Yaw left
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin,
                              GPIO_PIN_SET);
            vTaskResume(LoopTask);
            vTaskDelete(CalibrationTask);
        }
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin | LED3_Pin);
        vTaskDelay(200);
    }
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
