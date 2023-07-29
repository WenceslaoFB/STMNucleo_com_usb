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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
    struct{
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    }bit;
    uint8_t byte;
}flag;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const char UNER[]="UNER";

#define CMD_ALIVE 0xD2
#define CMD_SENSORS 0xA0
#define CMD_MAG_SENSOR 0xA1
#define CMD_RFID_SENSOR 0xA2
#define CMD_EMERGENCY_STOP 0xB0

#define ALIVERECIVE flag1.bit.b0 //RECIBIO alive
#define SENSORS_RECIVE flag1.bit.b1 //RECIBIO COMANDO PARA ENVIAR VALORES SENSORES CADA CIERTO TIEMPO
#define FLAG2 flag1.bit.b2 //FLAG2
#define FLAG3 flag1.bit.b3 //FLAG3

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

volatile flag flag1;

volatile uint8_t buffer_tx[256];

volatile uint8_t buffer_rx[256];

volatile uint8_t indR_rx = 0;
volatile uint8_t indW_rx = 0;

volatile uint8_t indR_tx = 0;
volatile uint8_t indW_tx = 0;
volatile uint8_t cksTX = 0;

volatile uint8_t listoSend = 0;

uint8_t timeoutSENSORS = 0;
uint16_t size=0;


uint32_t cmdUNERprotocol;
uint8_t bytesUNERprotocol;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void DecodeQT();
void DecodeCommands(uint8_t *buffer, uint8_t indexCMD);
void SendData(uint8_t cmd);
void uart();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //funcion para saber cuando una transferencia se completo

	indW_rx++;
	HAL_UART_Receive_IT(&huart2, (uint8_t *) &buffer_rx[indW_rx], 1);
	//HAL_UART_Transmit_IT(&huart2, buffer_tx, strlen((char*)buffer_tx));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	//HAL_UART_Transmit_IT(&huart2, buffer_tx, strlen((char*)buffer_tx));
	if(SENSORS_RECIVE && timeoutSENSORS){
		timeoutSENSORS--;
	}
}

void uart(){

		if((huart2.Instance->SR & UART_FLAG_TXE)==UART_FLAG_TXE){
			huart2.Instance->DR=buffer_tx[indR_tx];
			indR_tx++;
		}
/*
	if(indW_tx > indR_tx){
			size = indW_tx - indR_tx;
	}else{
			size = 256 - indR_tx;
	}
	HAL_UART_Transmit_IT(&huart2, (uint8_t *) &buffer_tx, size);
	indR_tx++;*/
}

void DecodeQT(){
	static uint8_t i=0,step=0,cksQT,counter=1,cmdPosInBuff;

	switch(step){
		case 0:
			if(buffer_rx[indR_rx]==UNER[i]){
				i++;
				if(i==4){
					step++;
					i=0;
					//cksQT='U'^'N'^'E'^'R';
				}
			}else
				i=0;

			break;
		case 1:
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			bytesUNERprotocol=buffer_rx[indR_rx];
			step++;
			//cksQT^=bytesUNERprotocol;
			break;
		case 2:
			if(buffer_rx[indR_rx]== 0x00){
				step++;
			}else{
				step=0;
			}
			break;
		case 3:
			if(buffer_rx[indR_rx]==':'){
				cksQT^='U'^'N'^'E'^'R'^bytesUNERprotocol^0x00^':';
				step++;
				//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}else{
				step=0;
			}
			break;
		case 4:
			if(counter==1)
				cmdPosInBuff = indR_rx;
			if(counter<bytesUNERprotocol){
				cksQT^=buffer_rx[indR_rx];
				counter++;
			}else{
				if(cksQT==buffer_rx[indR_rx]){
					DecodeCommands((uint8_t*)&buffer_rx, cmdPosInBuff);
					//HAL_UART_Transmit_IT(&huart2, (uint8_t *) &buffer_tx, strlen((char*)buffer_tx));
					//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				}
				step=0;
				counter=1;
			}
			break;
	}
	indR_rx++;
}

void DecodeCommands(uint8_t *buffer, uint8_t indexCMD){

	switch(buffer[indexCMD]){
		case 0xA2://Enviar datos de sensor RFID a pantalla

		break;
		case 0xA1: //Enviar datos de sensor mag a pantalla

		break;
		case 0xA0:	//Enviar datos de sensores a pantalla
			timeoutSENSORS = 4;
			SENSORS_RECIVE = 1;
		break;
		case CMD_ALIVE: //ALIVE
			SendData(CMD_ALIVE);
		break;

		case 0xB0://Parada de emergencia
		break;
	}
}
void SendData(uint8_t cmd){

		buffer_tx[indW_tx++]='U';
		buffer_tx[indW_tx++]='N';
		buffer_tx[indW_tx++]='E';
		buffer_tx[indW_tx++]='R';

		switch(cmd){
			case CMD_ALIVE:
				buffer_tx[indW_tx++] = 0x02;
				buffer_tx[indW_tx++] = 0x00;
				buffer_tx[indW_tx++] = ':';
				buffer_tx[indW_tx++] = cmd;
			break;
			case CMD_MAG_SENSOR:
				buffer_tx[indW_tx++] = 0x08;
				buffer_tx[indW_tx++] = 0x00;
				buffer_tx[indW_tx++] = ':';
				buffer_tx[indW_tx++] = cmd;
				buffer_tx[indW_tx++] = 0x05; //1
				buffer_tx[indW_tx++] = 0x05; //2
				buffer_tx[indW_tx++] = 0x05; //3
				buffer_tx[indW_tx++] = 0x05; //4
				buffer_tx[indW_tx++] = 0x05; //5
				buffer_tx[indW_tx++] = 0x05; //6
			break;
			case CMD_RFID_SENSOR:
				buffer_tx[indW_tx++] = 0x08;
				buffer_tx[indW_tx++] = 0x00;
				buffer_tx[indW_tx++] = ':';
				buffer_tx[indW_tx++] = cmd;
				buffer_tx[indW_tx++] = 0x05; //1
				buffer_tx[indW_tx++] = 0x05; //2
				buffer_tx[indW_tx++] = 0x05; //3
				buffer_tx[indW_tx++] = 0x05; //4
				buffer_tx[indW_tx++] = 0x05; //5
				buffer_tx[indW_tx++] = 0x05; //6
			break;
			default:
			break;
		}


		cksTX=0;
		for(uint8_t i=0; i<indW_tx; i++) {
			cksTX^=buffer_tx[i];
			//pc.printf("%d - %x - %d   v: %d \n",i,cks,cks,tx[i]);
		}
		if(cksTX>0)
			buffer_tx[indW_tx++]=cksTX;

		//listoSend = 1;
	//HAL_UART_Transmit_IT(&huart2, (uint8_t *) &buffer_tx, size);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &buffer_rx[indW_rx], 1);
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Transmit_IT(&huart2, buffer_tx, strlen((char*)buffer_tx));
	  //HAL_Delay(1000);

	  if(indR_rx != indW_rx){
		  DecodeQT();
	  }


	  if(indR_tx != indW_tx){
	  	  uart();
	  }


	  if(!timeoutSENSORS){
		  SendData(CMD_MAG_SENSOR);
		  SendData(CMD_RFID_SENSOR);
		  timeoutSENSORS = 4;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8400;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
