/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include <stdio.h>
#include <stdlib.h>   // malloc
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRIGGER_THRESHOLD 100
#define SAMPLES_IN_EVENT 1000
#define WF_PRE_SAMPLES 50		  // Samples to include before trigger
#define ADC_BUFFER_SAMPLES 3 * SAMPLES_IN_EVENT
#define FIFO_NUMWF 4


#define ADC_HALF_BUFFER_SAMPLES ADC_BUFFER_SAMPLES/2
/* Packet footer (32 bytes)
 * 4 byte uint32: event/trigger number
 * 1 byte int: fifo slot number (for debugging)
 * 19 byte ASCII string: yymmdd_hh:mm:ss.qqq, with 1 Jan 2000 = device startup
 * 8 byte magic terminator: !~O_o~! + null character
 */
#define DATE_AND_TERMINATOR_LENGTH 19 + 8
#define FOOTER_LEN 4 + 1 + DATE_AND_TERMINATOR_LENGTH
#define BYTES_PER_SAMPLE 2
#define EVENT_SIZE (SAMPLES_IN_EVENT * BYTES_PER_SAMPLE + FOOTER_LEN)

#define SLOT_FREE 0      // memory will be old junk, don't assume it's 0
#define SLOT_WRITING 1
#define SLOT_FULL 2
#define SLOT_TRANSMITTING 3

#define NO_ACTIVE_TRIGGER -1
#define NOT_TRANSMITTING -1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

uint16_t adc_buf[ADC_BUFFER_SAMPLES];

// Index in adc_buf of the trigger currently being processed
int16_t active_trigger = NO_ACTIVE_TRIGGER;

// Buffer holding waveforms before transmission
unsigned char fifo[FIFO_NUMWF][EVENT_SIZE];
int fifo_status[FIFO_NUMWF];            // See SLOT_XXX defines. Could enum...
uint32_t fifo_event_number[FIFO_NUMWF]; // valid if SLOT_FULL, junk otherwise

uint32_t event_number;   // Number for the next observed event

// Fifo slot that is being sent over UART
int transmitting_slot = NOT_TRANSMITTING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void fill_tx_buffer();

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
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFFER_SAMPLES);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (transmitting_slot != NOT_TRANSMITTING) continue;

		// Find earliest untransmitted waveform in buffer
		int earliest_slot = -1;
		uint32_t earliest_event_number = 0;
		for (int i = 0; i < FIFO_NUMWF; i++){
			if ((fifo_status[i] == SLOT_FULL)
					&& ((earliest_slot == -1)
						|| fifo_event_number[i] <= earliest_event_number)){
				earliest_slot = i;
				earliest_event_number = fifo_event_number[i];
			}
		}
		if (earliest_slot == -1) continue;


		transmitting_slot = earliest_slot;
		fifo_status[transmitting_slot] = SLOT_TRANSMITTING;
		HAL_UART_Transmit_DMA(
				&huart2,
				(uint8_t*)fifo[transmitting_slot],
				EVENT_SIZE);

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart){
	fifo_status[transmitting_slot] = SLOT_FREE;
	transmitting_slot = NOT_TRANSMITTING;
}


void write_RTC_datetime(unsigned char *datetime_str){
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	snprintf((char *) datetime_str,
			 DATE_AND_TERMINATOR_LENGTH,
             "%02d%02d%02d_%02d:%02d:%02d.%03ld!~O_o~!",
 			 sDate.Year, sDate.Month, sDate.Date,
 			 sTime.Hours, sTime.Minutes, sTime.Seconds,
			 (1000 - (1000 * sTime.SubSeconds)/sTime.SecondFraction) );
}

void copy_wf_to_fifo (){
	event_number += 1;

	// Find a free slot to store the waveform
	int free_slot_i = -1;
	for (int i = 0; i < FIFO_NUMWF; i++){
		if (fifo_status[i] == SLOT_FREE){
			free_slot_i = i;
			break;
		}
	}
	if (free_slot_i == -1) {
		// Can't save this trigger, no free slot
		active_trigger = NO_ACTIVE_TRIGGER;
		return;
	}

	fifo_status[free_slot_i] = SLOT_WRITING;
	fifo_event_number[free_slot_i] = event_number;

	// Index in adc_buf where the event starts
	int16_t event_start = active_trigger - WF_PRE_SAMPLES;
	if (event_start < 0)
		event_start = ADC_BUFFER_SAMPLES + event_start;

	// Form a waveform packet inside our assigned fifo slot
	// pos tracks our position / bytes written
	unsigned char* slot = fifo[free_slot_i];
	uint16_t pos = 0;

	// Copy over the data (BYTES_PER_SAMPLE * SAMPLES_IN_EVENT bytes)
	if (event_start > (ADC_BUFFER_SAMPLES - SAMPLES_IN_EVENT)){
		// The event wraps around the buffer
		// could in-line this... maybe compiler is nice enough to do this for us?
		uint16_t samples_near_end = ADC_BUFFER_SAMPLES - event_start;
		uint16_t samples_near_start = SAMPLES_IN_EVENT - samples_near_end;
		memcpy(&slot[pos],
               &adc_buf[event_start],
               samples_near_end * BYTES_PER_SAMPLE);
		pos += samples_near_end * BYTES_PER_SAMPLE;
		memcpy(&slot[pos],
			   &adc_buf[0],
			   samples_near_start * BYTES_PER_SAMPLE);
		pos += samples_near_start * BYTES_PER_SAMPLE;
	}
	else {
		// The event is in sequential memory
		memcpy(&slot[pos],
			   &adc_buf[event_start],
			   SAMPLES_IN_EVENT * BYTES_PER_SAMPLE);
		pos += SAMPLES_IN_EVENT * BYTES_PER_SAMPLE;
	}


	// Event number
	memcpy(&slot[pos], &event_number, sizeof(event_number));   // TODO: sizeof(var) is clearer, try it
	pos += sizeof(event_number);

	// FIFO slot number
	slot[pos] = (unsigned char)free_slot_i;
	pos += 1;

	// Datetime string
	write_RTC_datetime(&slot[pos]);
	pos += DATE_AND_TERMINATOR_LENGTH;

	active_trigger = NO_ACTIVE_TRIGGER;
	fifo_status[free_slot_i] = SLOT_FULL;
}


void check_half_buffer_for_trigger(int start_i){
	if (active_trigger != NO_ACTIVE_TRIGGER){
		/* There was a trigger in the previous half buffer,
		 * but it needed data from this half to complete the waveform */
		copy_wf_to_fifo();

	} else {
		/* look for a trigger in the half full buffer of new data */
		for (int i=start_i; i < start_i + ADC_HALF_BUFFER_SAMPLES; i=i+1){
			if (adc_buf[i] > TRIGGER_THRESHOLD) {
				active_trigger = i;
				break;
			}
		}
	}

	if (active_trigger != NO_ACTIVE_TRIGGER
			&& (active_trigger < start_i + ADC_HALF_BUFFER_SAMPLES - (SAMPLES_IN_EVENT - WF_PRE_SAMPLES))) {
		// The entire event we found lies in this half-buffer, so just copy it out */
		copy_wf_to_fifo();
	}
}


void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc){
	check_half_buffer_for_trigger(0);
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	check_half_buffer_for_trigger(ADC_HALF_BUFFER_SAMPLES);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
