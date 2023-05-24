/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATUS_UNKNOWN = 0,
	STATUS_SYSTEM_READY,
	STATUS_DC_ON,
	STATUS_QUIT_DC_ON,
	STATUS_INVERTER_ON,
	STATUS_QUIT_INVERTER_ON, //Running status
	STATUS_DERATING,
	STATUS_ERROR,
	STATUS_RUNNING,
} AMK_Status;

typedef enum {
	CONTROL_UNKNOWN = 0,
	CONTROL_DC_ON,
	CONTROL_ENABLE,
	CONTROL_INVERTER_ON,
	CONTROL_TS_READY,
	CONTROL_ERROR_RESET_RIGHT,
	CONTROL_ERROR_RESET_LEFT,
	CONTROL_RUNNING,
} AMK_Control;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

/* Definitions for controllerStart */
osThreadId_t controllerStartHandle;
const osThreadAttr_t controllerStart_attributes = {
  .name = "controllerStart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef AMK_TxHeader_R;
CAN_TxHeaderTypeDef AMK_TxHeader_L;
CAN_RxHeaderTypeDef RxHeader;
CAN_RxHeaderTypeDef RxHeader_2;
uint8_t AMK_TxData_R[8];
uint8_t AMK_TxData_L[8];
uint8_t RxData[8];
uint8_t RxData_2[8];
uint32_t TxMailbox;
AMK_Status MotorStatus_R = 0;
AMK_Status MotorStatus_L = 0;
AMK_Control ControlStatus = 0;
_Bool TsOn_n;

uint16_t APPS1_VAL;
uint16_t APPS2_VAL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN3_Init(void);
void Start_FRT_controller(void *argument);
void Start_AMK(void *argument);

/* USER CODE BEGIN PFP */
static void CAN_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float APPS1_ADC_Percent(void) {
	uint16_t ADC_VAL;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (float)ADC_VAL/4095; //returns ADC percentage ranges from 0-1
}

float APPS2_ADC_Percent(void) {
	uint16_t ADC_VAL;

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	ADC_VAL = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);
	return (float)ADC_VAL/4095;
}

/* USER CODE END 0 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN3_Init();
  /* USER CODE BEGIN 2 */
  	CAN_Config();

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	  Error_Handler();
	}
	// Initialize TxHeader and TxData
	AMK_TxHeader_R.DLC = 8;
	AMK_TxHeader_R.IDE = CAN_ID_STD;
	AMK_TxHeader_R.RTR = CAN_RTR_DATA;
	AMK_TxHeader_R.StdId = 0x186;
	AMK_TxHeader_R.TransmitGlobalTime = DISABLE;

	AMK_TxData_R[0] = 0x00;
	AMK_TxData_R[1] = 0x00;
	AMK_TxData_R[2] = 0x00;
	AMK_TxData_R[3] = 0x00;
	AMK_TxData_R[4] = 0x00;
	AMK_TxData_R[5] = 0x00;
	AMK_TxData_R[6] = 0x00;
	AMK_TxData_R[7] = 0x00;

	AMK_TxHeader_L.DLC = 8;
	AMK_TxHeader_L.IDE = CAN_ID_STD;
	AMK_TxHeader_L.RTR = CAN_RTR_DATA;
	AMK_TxHeader_L.StdId = 0x185;
	AMK_TxHeader_L.TransmitGlobalTime = DISABLE;

	AMK_TxData_L[0] = 0x00;
	AMK_TxData_L[1] = 0x00;
	AMK_TxData_L[2] = 0x00;
	AMK_TxData_L[3] = 0x00;
	AMK_TxData_L[4] = 0x00;
	AMK_TxData_L[5] = 0x00;
	AMK_TxData_L[6] = 0x00;
	AMK_TxData_L[7] = 0x00;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controllerStart */
  controllerStartHandle = osThreadNew(Start_FRT_controller, NULL, &controllerStart_attributes);

  /* creation of motorControl */
  motorControlHandle = osThreadNew(Start_AMK, NULL, &motorControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hcan1.Init.Prescaler = 1;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
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
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 1;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = ENABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_LED_Pin|START_BTN_LED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_LED_Pin START_BTN_LED_EN_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin|START_BTN_LED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BRAKE_LIGHT_EN_Pin RTDS_EN_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_EN_Pin|RTDS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : START_BTN_n_Pin */
  GPIO_InitStruct.Pin = START_BTN_n_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(START_BTN_n_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void CAN_Config(void)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterBank = 13;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x284<<5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFE<<5; //Only ID 0x284 and 0x285 can pass through
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan3) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//Get Rx message
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
		Error_Handler();
	}

	if (RxHeader.StdId == 0x285) {
		if ((RxData[1] >> 1) & 1) {
			MotorStatus_R = STATUS_ERROR;
			return;
		} else if (RxData[1] >> 7 & 1){
			MotorStatus_R = STATUS_DERATING;
			return;
		} else {
			switch(RxData[1]) {
				case 0x01:
					MotorStatus_R = STATUS_SYSTEM_READY;
					break;
				case 0x11:
					MotorStatus_R = STATUS_DC_ON;
					break;
				case 0x19:
					MotorStatus_R = STATUS_QUIT_DC_ON;
					break;
				case 0x59:
					MotorStatus_R = STATUS_INVERTER_ON;
					break;
				case 0x79:
					MotorStatus_R = STATUS_QUIT_INVERTER_ON;
					break;
				default:
					MotorStatus_R = STATUS_UNKNOWN;
			}
		}
	}
	else if (RxHeader.StdId == 0x284) {
		if ((RxData[1] >> 1) & 1) {
			MotorStatus_L = STATUS_ERROR;
			return;
		} else if (RxData[1] >> 7 & 1){
			MotorStatus_L = STATUS_DERATING;
			return;
		} else {
			switch(RxData[1]) {
				case 0x01:
					MotorStatus_L = STATUS_SYSTEM_READY;
					break;
				case 0x11:
					MotorStatus_L = STATUS_DC_ON;
					break;
				case 0x19:
					MotorStatus_L = STATUS_QUIT_DC_ON;
					break;
				case 0x59:
					MotorStatus_L = STATUS_INVERTER_ON;
					break;
				case 0x79:
					MotorStatus_L = STATUS_QUIT_INVERTER_ON;
					break;
				default:
					MotorStatus_L = STATUS_UNKNOWN;
			}
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_FRT_controller */
/**
  * @brief  Function implementing the controllerStart thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_FRT_controller */
void Start_FRT_controller(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
    if (HAL_GPIO_ReadPin(START_BTN_n_GPIO_Port, START_BTN_n_Pin) == 0) {
    	//Read Ready to Drive button
    	TsOn_n = 1;
    }
    if (TsOn_n == 1) {
    	HAL_GPIO_WritePin(RTDS_EN_GPIO_Port, RTDS_EN_Pin, GPIO_PIN_SET);
    	osDelay(3000);
    	HAL_GPIO_WritePin(RTDS_EN_GPIO_Port, RTDS_EN_Pin, GPIO_PIN_RESET);
    	osThreadTerminate(controllerStartHandle);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_AMK */
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_AMK */
void Start_AMK(void *argument)
{
  /* USER CODE BEGIN Start_AMK */
  /* Infinite loop */
  for(;;)
  {

    osDelay(5);
    APPS2_VAL = APPS2_ADC_Percent()*500;

	//HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(RTDS_EN_GPIO_Port, RTDS_EN_Pin, GPIO_PIN_SET);

    if ((MotorStatus_R == STATUS_SYSTEM_READY) && (MotorStatus_L == STATUS_SYSTEM_READY)) {
    	AMK_TxData_R[1] = 0x02;
    	AMK_TxData_L[1] = 0x02;
		ControlStatus = CONTROL_DC_ON;
    } else if ((MotorStatus_R == STATUS_QUIT_DC_ON) && (MotorStatus_L == STATUS_QUIT_DC_ON)) {
		AMK_TxData_L[1] = 0x07;
		AMK_TxData_R[1] = 0x07;
		memset(&AMK_TxData_R[2],0x00, 2*sizeof(uint8_t));
		memset(&AMK_TxData_R[4],0x00, 2*sizeof(uint8_t));
		memset(&AMK_TxData_L[2],0x00, 2*sizeof(uint8_t));
		memset(&AMK_TxData_L[4],0x00, 2*sizeof(uint8_t));
		ControlStatus = CONTROL_ENABLE;
    } else if ((MotorStatus_R == STATUS_INVERTER_ON) && (MotorStatus_L == STATUS_INVERTER_ON)) {
    	AMK_TxData_L[1] = 0x07;
    	AMK_TxData_R[1] = 0x07;
    	memset(&AMK_TxData_R[2],0x00, 2*sizeof(uint8_t));
    	memset(&AMK_TxData_R[4],0x00, 2*sizeof(uint8_t));
    	memset(&AMK_TxData_L[2],0x00, 2*sizeof(uint8_t));
    	memset(&AMK_TxData_L[4],0x00, 2*sizeof(uint8_t));
    	ControlStatus = CONTROL_INVERTER_ON;
    } else if ((MotorStatus_R == STATUS_QUIT_INVERTER_ON) && (MotorStatus_L == STATUS_QUIT_INVERTER_ON)) {
    	HAL_GPIO_WritePin(BRAKE_LIGHT_EN_GPIO_Port, BRAKE_LIGHT_EN_Pin, GPIO_PIN_SET);
    	if (TsOn_n) {
    		AMK_TxData_R[1] = 0x07;
			AMK_TxData_R[2] = APPS2_VAL & 0xFF;
			AMK_TxData_R[3] = (APPS2_VAL >> 8) & 0xFF;
			AMK_TxData_R[4] = 0x32; //set positive torque request to 50

			AMK_TxData_L[1] = 0x07;
			AMK_TxData_L[2] = APPS2_VAL & 0xFF;
			AMK_TxData_L[3] = (APPS2_VAL >> 8) & 0xFF;
			AMK_TxData_L[4] = 0x32;
			if (RxData[1] != 0x79) {
				memset(&AMK_TxData_R[2],0x00, 4*sizeof(uint8_t));
				memset(&AMK_TxData_L[2],0x00, 4*sizeof(uint8_t));
			}
			ControlStatus = CONTROL_RUNNING;
    	} else {
    		AMK_TxData_R[1] = 0x07;
			AMK_TxData_L[1] = 0x07;
			memset(&AMK_TxData_R[2],0x00, 2*sizeof(uint8_t));
			memset(&AMK_TxData_R[4],0x00, 2*sizeof(uint8_t));
			memset(&AMK_TxData_L[2],0x00, 2*sizeof(uint8_t));
			memset(&AMK_TxData_L[4],0x00, 2*sizeof(uint8_t));
    		ControlStatus = CONTROL_TS_READY;
    	}
    } else if (MotorStatus_R == STATUS_ERROR) {
    	AMK_TxData_R[1] = 0x08;
		ControlStatus = CONTROL_ERROR_RESET_RIGHT;
    } else if (MotorStatus_L == STATUS_ERROR) {
    	AMK_TxData_L[1] = 0x08;
		ControlStatus = CONTROL_ERROR_RESET_LEFT;
    } else {
    	ControlStatus = CONTROL_UNKNOWN;
    }

	HAL_CAN_AddTxMessage(&hcan1, &AMK_TxHeader_R, AMK_TxData_R, &TxMailbox);
	HAL_CAN_AddTxMessage(&hcan1, &AMK_TxHeader_L, AMK_TxData_L, &TxMailbox);
	memset(&AMK_TxData_R[0],0x00, 8*sizeof(uint8_t));
	memset(&AMK_TxData_L[0],0x00, 8*sizeof(uint8_t));
  }
  /* USER CODE END Start_AMK */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, 1);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
