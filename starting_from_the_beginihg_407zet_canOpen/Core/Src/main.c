/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
  /* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CO_app_STM32.h"
#include "../CANopenNode_STM32/OD.h"
#include "../CANopenNode/301/CO_driver.h"
#include "../CANopenNode/301/CO_SDOserver.h"
#include "../CANopenNode/301/CO_SDOclient.h"
#include "../CANopenNode/301/CO_NMT_Heartbeat.h"
#include "../CANopenNode/301/CO_PDO.h"
#include "../CANopenNode/301/CO_HBconsumer.h"
#include "../CANopenNode/CANopen.h"
#include "string.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MASTER_NODE_ID 66
#define WHEEL_NODE_ID 0x16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

CO_SDOclient_t SDO_C;
CO_NMT_t NMT;
CO_CANmodule_t CANmodule;
uint8_t heartBeatData[8];
CO_CANrx_t CANrxArray[16]; // Массив буферов приема CAN сообщений
CO_CANtx_t CANtxArray[16]; // Массив буферов передачи CAN сообщений

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Функция обработки сообщений CAN
void CAN_receive(void* object, void* message) {
    CO_CANrxMsg_t* msg = (CO_CANrxMsg_t*)message;
    uint16_t canID = msg->ident;

    // Проверка на Heartbeat сообщение
    if (canID == (0x700 + WHEEL_NODE_ID)) {
        memcpy(heartBeatData, msg->data, 8);
        // Отправка данных через UART
        HAL_UART_Transmit(&huart2, (uint8_t*)"Heartbeat message received: ", strlen("Heartbeat message received: "), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, heartBeatData, 8, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
    else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Received CAN message with ID: ", strlen("Received CAN message with ID: "), HAL_MAX_DELAY);
        char idStr[10];
        sprintf(idStr, "0x%X", canID);
        HAL_UART_Transmit(&huart2, (uint8_t*)idStr, strlen(idStr), HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim14) {
        canopen_app_interrupt();
    }
}

void disable_CAN_transmission(CO_CANmodule_t* CANmodule) {
    CANmodule->CANerrorStatus |= CO_CAN_ERRTX_BUS_OFF;
    // Дополнительно можно вызвать функцию, которая обработает это состояние
    CO_CANmodule_disable(CANmodule);
}

// Инициализация CANopen
void CANopen_init() {
    CO_ReturnError_t err;

    // Инициализация CAN модуля
    HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing CAN module...\r\n", strlen("Initializing CAN module...\r\n"), HAL_MAX_DELAY);
    err = CO_CANmodule_init(&CANmodule, CAN1, CANrxArray, sizeof(CANrxArray) / sizeof(CANrxArray[0]), CANtxArray, sizeof(CANtxArray) / sizeof(CANtxArray[0]), 125);
    if (err != CO_ERROR_NO) {
        // Обработка ошибки
        HAL_UART_Transmit(&huart2, (uint8_t*)"CAN module initialization failed\r\n", strlen("CAN module initialization failed\r\n"), HAL_MAX_DELAY);
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"CAN module initialized successfully\r\n", strlen("CAN module initialized successfully\r\n"), HAL_MAX_DELAY);

    // Настройка функции обработки сообщений CAN
    HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing CAN receive buffer...\r\n", strlen("Initializing CAN receive buffer...\r\n"), HAL_MAX_DELAY);
    CO_CANrxBufferInit(&CANmodule, 0, 0x700 + WHEEL_NODE_ID, 0x7FF, 0, NULL, CAN_receive);

    OD_entry_t* OD_1017_ProducerHbTime = OD_ENTRY_H1017;
    OD_entry_t* OD_1280_SDOcliPar = OD_ENTRY_H1280;

    uint32_t errInfo = 0;

    // Инициализация NMT и Heartbeat
    HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing NMT and Heartbeat...\r\n", strlen("Initializing NMT and Heartbeat...\r\n"), HAL_MAX_DELAY);
    err = CO_NMT_init(&NMT, OD_1017_ProducerHbTime, NULL, MASTER_NODE_ID, 0, 0, &CANmodule, 0, 0x700 + MASTER_NODE_ID, &CANmodule, 0, 0x700 + MASTER_NODE_ID, &errInfo);
    if (err != CO_ERROR_NO) {
        // Обработка ошибки
        HAL_UART_Transmit(&huart2, (uint8_t*)"NMT initialization failed\r\n", strlen("NMT initialization failed\r\n"), HAL_MAX_DELAY);
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"NMT and Heartbeat initialized successfully\r\n", strlen("NMT and Heartbeat initialized successfully\r\n"), HAL_MAX_DELAY);

    // Инициализация SDO client
    HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing SDO client...\r\n", strlen("Initializing SDO client...\r\n"), HAL_MAX_DELAY);
    err = CO_SDOclient_init(&SDO_C, OD, OD_1280_SDOcliPar, MASTER_NODE_ID, &CANmodule, 0, &CANmodule, 0, NULL);
    if (err != CO_ERROR_NO) {
        // Обработка ошибки
        HAL_UART_Transmit(&huart2, (uint8_t*)"SDO client initialization failed\r\n", strlen("SDO client initialization failed\r\n"), HAL_MAX_DELAY);
        return;
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)"SDO client initialized successfully\r\n", strlen("SDO client initialized successfully\r\n"), HAL_MAX_DELAY);
}

// Пример функции чтения SDO
CO_SDO_abortCode_t read_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId,
    uint16_t index, uint8_t subIndex,
    uint8_t* buf, size_t bufSize, size_t* readSize)
{
    CO_SDO_return_t SDO_ret;

    // Настройка клиента
    SDO_ret = CO_SDOclient_setup(SDO_C, CO_CAN_ID_SDO_CLI + nodeId, CO_CAN_ID_SDO_SRV + nodeId, nodeId);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // Инициация чтения
    SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, 1000, 0);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // Чтение данных
    size_t nRead;
    do {
        uint32_t timeDifference_us = 10000;
        CO_SDO_abortCode_t abortCode;

        SDO_ret = CO_SDOclientUpload(SDO_C, timeDifference_us, false, false, &abortCode, NULL, NULL);
        if (SDO_ret < 0) {
            return abortCode;
        }
    } while (SDO_ret > 0);

    // Копирование данных
    nRead = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
    if (readSize) {
        *readSize = nRead;
    }

    return CO_SDO_AB_NONE;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_TIM14_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_UART_Transmit(&huart2, (uint8_t*)"Starting initialization\r\n", strlen("Starting initialization\r\n"), HAL_MAX_DELAY);

    CANopenNodeSTM32 canOpenNodeSTM32;
    canOpenNodeSTM32.CANHandle = &hcan1;
    canOpenNodeSTM32.HWInitFunction = MX_CAN1_Init;
    canOpenNodeSTM32.timerHandle = &htim14;
    canOpenNodeSTM32.desiredNodeID = MASTER_NODE_ID;
    canOpenNodeSTM32.baudrate = 500;
    canopen_app_init(&canOpenNodeSTM32);

    HAL_UART_Transmit(&huart2, (uint8_t*)"CAN module initialization\r\n", strlen("CAN module initialization\r\n"), HAL_MAX_DELAY);

    CANopen_init();

    // Пример чтения SDO
    uint8_t buffer[8];
    size_t readSize;
    CO_SDO_abortCode_t sdoAbortCode = read_SDO(&SDO_C, WHEEL_NODE_ID, 0x1018, 0x00, buffer, sizeof(buffer), &readSize);
    if (sdoAbortCode == CO_SDO_AB_NONE) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"SDO read successful\r\n", strlen("SDO read successful\r\n"), HAL_MAX_DELAY);
    }
    else {
        HAL_UART_Transmit(&huart2, (uint8_t*)"SDO read failed\r\n", strlen("SDO read failed\r\n"), HAL_MAX_DELAY);
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    while (1)
    {
        canopen_app_process();
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */

    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 21;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

    /* USER CODE BEGIN TIM14_Init 0 */

    /* USER CODE END TIM14_Init 0 */

    /* USER CODE BEGIN TIM14_Init 1 */

    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 63;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 1000;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */

    /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

      /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOF, led2_Pin | led1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : led2_Pin led1_Pin */
    GPIO_InitStruct.Pin = led2_Pin | led1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
