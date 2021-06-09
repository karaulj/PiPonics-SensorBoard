/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "ezo_sensor.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
// Pi UART
#define PI_UART_BUFFER_SZ		(uint8_t) 6
#define CMD_IDX					(uint8_t) 0
#define ITEM_IDX				(uint8_t) 1
#define PAYLOAD_START_IDX		(uint8_t) 2		// payload is 4 bytes
// Buzzer
#define BUZZER_DELAY_MS			5000U
#define BUZZER_ON_MS			250U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for receiveUART */
osThreadId_t receiveUARTHandle;
const osThreadAttr_t receiveUART_attributes = {
  .name = "receiveUART",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for detectOverflow */
osThreadId_t detectOverflowHandle;
const osThreadAttr_t detectOverflow_attributes = {
  .name = "detectOverflow",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UARTSemaphore */
osSemaphoreId_t UARTSemaphoreHandle;
const osSemaphoreAttr_t UARTSemaphore_attributes = {
  .name = "UARTSemaphore"
};
/* Definitions for overflowSemaphore */
osSemaphoreId_t overflowSemaphoreHandle;
const osSemaphoreAttr_t overflowSemaphore_attributes = {
  .name = "overflowSemaphore"
};
/* Definitions for I2CSemaphore */
osSemaphoreId_t I2CSemaphoreHandle;
const osSemaphoreAttr_t I2CSemaphore_attributes = {
  .name = "I2CSemaphore"
};
/* USER CODE BEGIN PV */

// UART buffer for communication w/ Raspberry Pi
uint8_t piUartBuffer[PI_UART_BUFFER_SZ];

enum piCmds {
	Echo	= 0xF0,
	Drive	= 0x0F,
	Read	= 0x88
} piCmd;

typedef enum {
	TankLights			= 0,
	GrowBedWaterPump	= 1,
	// test case
	Ret0				= 0xFF,
	Ret1				= 0xFE
} Actuator;

typedef enum {
	TankTemp			= 0,
	GrowBedWaterLevel	= 1,
	// test case
	RetPi				= 0xFF
} Sensor;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartReceiveUART(void *argument);
void StartDetectOverflow(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("START\n\r");

  // Pi UART Rx DMA init
  if (HAL_UART_Receive_DMA(&huart1, &piUartBuffer[0], PI_UART_BUFFER_SZ) != HAL_OK) {
	Error_Handler();
  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of UARTSemaphore */
  UARTSemaphoreHandle = osSemaphoreNew(1, 1, &UARTSemaphore_attributes);

  /* creation of overflowSemaphore */
  overflowSemaphoreHandle = osSemaphoreNew(1, 1, &overflowSemaphore_attributes);

  /* creation of I2CSemaphore */
  I2CSemaphoreHandle = osSemaphoreNew(1, 1, &I2CSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreAcquire(UARTSemaphoreHandle, 0);
  osSemaphoreAcquire(overflowSemaphoreHandle, 0);
  osSemaphoreAcquire(I2CSemaphoreHandle, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of receiveUART */
  receiveUARTHandle = osThreadNew(StartReceiveUART, NULL, &receiveUART_attributes);

  /* creation of detectOverflow */
  detectOverflowHandle = osThreadNew(StartDetectOverflow, NULL, &detectOverflow_attributes);

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Pump_Relay_GPIO_Port, Pump_Relay_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|LD3_Pin|Tank_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Pump_Relay_Pin */
  GPIO_InitStruct.Pin = Pump_Relay_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Pump_Relay_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LD3_Pin Tank_LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LD3_Pin|Tank_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Float_Switch_Pin */
  GPIO_InitStruct.Pin = Float_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Float_Switch_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
	/* write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1);
	return ch;
}

/* Pi UART Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		osSemaphoreRelease(UARTSemaphoreHandle);
	}
}

/* Float Switch Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Float_Switch_Pin) {
		osSemaphoreRelease(overflowSemaphoreHandle);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		osSemaphoreRelease(I2CSemaphoreHandle);
	}
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->ErrorCode == HAL_I2C_ERROR_AF) {
		printf("I2C AF err\r\n");
	}
}


void turnOnWaterPump() {
	HAL_GPIO_WritePin(Pump_Relay_GPIO_Port, Pump_Relay_Pin, GPIO_PIN_SET);
}
void turnOffWaterPump() {
	HAL_GPIO_WritePin(Pump_Relay_GPIO_Port, Pump_Relay_Pin, GPIO_PIN_RESET);
}
void turnOnTankLights() {
	HAL_GPIO_WritePin(Tank_LED_GPIO_Port, Tank_LED_Pin, GPIO_PIN_SET);
}
void turnOffTankLights() {
	HAL_GPIO_WritePin(Tank_LED_GPIO_Port, Tank_LED_Pin, GPIO_PIN_RESET);
}
void turnOnBuzzer() {
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
}
void turnOffBuzzer() {
	HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
}


uint32_t driveActuator(Actuator actuator, uint32_t driveVal) {
	printf("id:  %d\n\r", (int)actuator);
	printf("val: %d\n\r", (int)driveVal);
	uint32_t retVal;

	switch(actuator) {

	case TankLights:			// actuator: sys1.tank1_lights
		(driveVal > 0) ? turnOnTankLights() : turnOffTankLights();
		retVal = 0;
		break;

	case GrowBedWaterPump:		// actuator: sys1.growbed1_waterpump
		(driveVal > 0) ? turnOnWaterPump() : turnOffWaterPump();
		retVal = 0;
		break;

	case Ret0:		// test case: return 0
		retVal = 0;
		break;

	case Ret1:		// test case: return 1
		retVal = 1;
		break;

	default:
		retVal = 0xFFFFFFFF;
		break;
	}
	printf("ret: %d\n\r", (int)retVal);
	return retVal;
}

/* Return a sensor reading from a provided sensor ID */
float getReading(Sensor sensor) {
	printf("id:  %d\n\r", (int) sensor);
	float retVal;

	switch(sensor) {

	case TankTemp:;
		// request reading from EZO RTD sensor over I2C
		HAL_I2C_Master_Transmit_IT(&hi2c1, RTD_I2C_ADDR_DEFAULT<<1, (uint8_t*)RTD_I2C_Cmd_Read, sizeof(RTD_I2C_Cmd_Read));
		// delay for EZO RTD processing
		osDelay(RTD_I2C_LONG_DELAY_MS);
		// start I2C Rx interrupt
		HAL_I2C_Master_Receive_IT(&hi2c1, RTD_I2C_ADDR_DEFAULT<<1, &RTD_I2C_resBuf[0], RTD_I2C_BUFFER_MAX_SZ);
		// semaphore released when response is received; should be instantaneous due to proc delay
		osSemaphoreAcquire(I2CSemaphoreHandle, 10);
		// get temp reading as float
		retVal = (float)getRTDReading(RTD_I2C_resBuf);
		// reset I2C buffer
		memset(&RTD_I2C_resBuf[0], '\0', RTD_I2C_BUFFER_MAX_SZ);
		break;

	case RetPi:	// test case: return pi (3.14)
		retVal = 3.14;
		break;

	default:
		retVal = 0xFFFFFFFF;
		break;
	}
	printf("ret: %f\n\r", retVal);
	return retVal;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	//printf("%d", uxTaskGetStackHighWaterMark( NULL ));
  /* Infinite loop */
	for(;;)
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		osDelay(1);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReceiveUART */
/**
* @brief Function implementing the receiveUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveUART */
void StartReceiveUART(void *argument)
{
  /* USER CODE BEGIN StartReceiveUART */
  /* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(UARTSemaphoreHandle, osWaitForever);	// wait for UART frame

		uint8_t retBuffer[PI_UART_BUFFER_SZ] = {
				piUartBuffer[CMD_IDX],			// copy command byte
				piUartBuffer[ITEM_IDX]			// copy sensor/actuator id byte
		};

		piCmd = piUartBuffer[CMD_IDX];
		switch(piCmd) {

		case Echo :
			printf("ECHO\n\r");
			// copy input val to output
			memcpy(&retBuffer[PAYLOAD_START_IDX], &piUartBuffer[PAYLOAD_START_IDX], sizeof(uint32_t));
			break;

		case Drive :
			printf("DRIVE\n\r");
			// get drive val
			uint32_t driveVal;
			memcpy(&driveVal, &piUartBuffer[PAYLOAD_START_IDX], sizeof(uint32_t));	// note: relies on little-endian repr.
			// drive actuator w/ drive val, get return val
			uint32_t retVal = driveActuator(piUartBuffer[ITEM_IDX], driveVal);
			// assign actuator return val to output buffer
			memcpy(&retBuffer[PAYLOAD_START_IDX], &retVal, sizeof(uint32_t));
			break;

		case Read :
			printf("READ\n\r");
			// get reading as return val
			float reading = getReading(piUartBuffer[ITEM_IDX]);
			// assign sensor reading to output buffer
			memcpy(&retBuffer[PAYLOAD_START_IDX], &reading, sizeof(float));			// relies on little-endian repr.
			break;

		default:
			printf("UNKNOWN CMD: %d\n\r", piCmd);
			break;
		}
		// transmit response frame to pi
		if (HAL_UART_Transmit_DMA(&huart1, &retBuffer[0], PI_UART_BUFFER_SZ) != HAL_OK) {
			Error_Handler();
		}
		// UART ready again
		if (HAL_UART_Receive_DMA(&huart1, &piUartBuffer[0], PI_UART_BUFFER_SZ) != HAL_OK) {
			Error_Handler();
		}
  }
  /* USER CODE END StartReceiveUART */
}

/* USER CODE BEGIN Header_StartDetectOverflow */
/**
* @brief Function implementing the detectOverflow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetectOverflow */
void StartDetectOverflow(void *argument)
{
  /* USER CODE BEGIN StartDetectOverflow */
  /* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire(overflowSemaphoreHandle, osWaitForever);	// wait for emergency detect trigger

		// Float switch triggered (grow bed overflow)
		turnOffWaterPump();
		// infinite loop: sound buzzer. Stop by resetting MCU
		while (1) {
			printf("OVERFLOW\n\r");
			turnOnBuzzer();
			osDelay(BUZZER_ON_MS);		// depends on 1000 Hz RTOS tick rate
			turnOffBuzzer();
			osDelay(BUZZER_DELAY_MS);
		}
	}
  /* USER CODE END StartDetectOverflow */
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
  printf("Error\n\r");

  while (1)
  {

	  for (int i=0; i<3; i++) {
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		  HAL_Delay(250);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		  HAL_Delay(250);
	  }
	  HAL_Delay(2000);

	  /*
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
