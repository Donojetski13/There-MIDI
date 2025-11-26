/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "vl53l7cx_api.h"
//#include "vs1053.h"
#include "MIDI_Player.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ToF_W 0x52
#define ToF_R 0x53
#define _4x4 ((uint8_t) 16U)
#define _8x8 ((uint8_t) 64U)
#define continus 1U
#define Auto 3U
#define RIGHT_CENTER_1 19
#define RIGHT_CENTER_2 20
#define CENTER_ZONE_1 27
#define CENTER_ZONE_2 28
#define CENTER_ZONE_3 35
#define CENTER_ZONE_4 36
#define LEFT_CENTER_1 43
#define LEFT_CENTER_2 44
#define NUM_CHANS 2U

#define OCTAVE 12U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
uint8_t resolution=_8x8, ranging_frequency=13, sharpener_percent=14;
uint16_t integration_time=27;
uint8_t data_to_transfer=0;
uint16_t Tof_values_1[64], Tof_values_2[64]; // 64(8x8) max size

ADC_ChannelConfTypeDef ADC_Chan_Config = {0};
uint32_t ADC_vals[NUM_CHANS], ADC_Channels[NUM_CHANS] = {ADC_CHANNEL_1, ADC_CHANNEL_4};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// void printGrid(uint8_t* vals, int res) {
//	 printf("Zone %d distance: %d mm", 0, valConcat(Tof_values_1,0));
// }

 uint8_t sensorInit(VL53L7CX_Configuration* Dev, int port, uint8_t* isAlive) {
	 printf("VL53L7CX sensor %d Initialization start\r\n", port);
	 Dev->platform.address = ToF_W;
	 if (port == 1) Dev->platform.i2c = hi2c1;
	 else if (port == 2) Dev->platform.i2c = hi2c2;

	 uint8_t status = vl53l7cx_is_alive(Dev, isAlive);
	   	if(!(*isAlive) || status)
	   	{
	   		printf("VL53L7CX sensor %d not detected at requested address\r\n", port);
	   		return status;
	   	}

	status |= vl53l7cx_init(Dev);
	status |= vl53l7cx_set_ranging_mode(Dev, Auto);
	status |= vl53l7cx_set_resolution(Dev, resolution);
	status |= vl53l7cx_set_ranging_frequency_hz(Dev, ranging_frequency);
	status |= vl53l7cx_set_integration_time_ms(Dev, integration_time);
	status |= vl53l7cx_set_sharpener_percent(Dev, sharpener_percent);
	status |= vl53l7cx_start_ranging(Dev);

	return status;
 };

 // Comparison function for integers (ascending order)
 uint16_t compareIntegers(const void *a, const void *b) {
	 uint16_t valA = *(const uint16_t *)a;
	 uint16_t valB = *(const uint16_t *)b;
     return valA - valB; // For descending order, return valB - valA;
 }

 // get average of the 2 lowest or highest zones out of 4 based on the sensor
 uint8_t _4ZonesVal(uint16_t* Values, int sensor, int ZONE_1, int ZONE_2, int ZONE_3, int ZONE_4) {
	 uint16_t x [4] = {Values[ZONE_1], Values[ZONE_2], Values[ZONE_3], Values[ZONE_4]};

	 qsort (x, 4, 2, compareIntegers);

	 if (sensor == 1) return (x[2] + x[3]) / 2;
	 return  (x[0] + x[1]) / 2;
 }

void Get_ADC_Vals(uint32_t* tempo, uint32_t* treble_bass)
{
	for (int i = 0; i < NUM_CHANS; i++)
	{
		ADC_Chan_Config.Channel = ADC_Channels[i];
		if (HAL_ADC_ConfigChannel(&hadc1, &ADC_Chan_Config) != HAL_OK)
		{
		Error_Handler();
		}
		HAL_ADC_Start(&hadc1);//start conversion
		HAL_ADC_PollForConversion(&hadc1, 10);//wait for conversion to finish
		ADC_vals[i] = HAL_ADC_GetValue(&hadc1);//retrieve value
	}
	*tempo = ADC_vals[0];
	*treble_bass = ADC_vals[1];
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t status_1, isAlive_1, status_2, isAlive_2;			// 1 = volume  | 2 = pitch
	VL53L7CX_Configuration Dev_1, Dev_2; /* Sensor configuration */
	VL53L7CX_ResultsData 	Results_1, Results_2;  // Sensor read out info
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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  	/* Init VL53L7CX sensor */
	status_1 = sensorInit(&Dev_1, 1, &isAlive_1);
	status_2 = sensorInit(&Dev_2, 2, &isAlive_2);
	printf("MIDI Board initialization\r\n");
	if(midi_Init())
	{
		printf("MIDI Board failed\r\n");
		return 13;
	}
  	if(status_1)
  	{
  		printf("Volume VL53L7CX ULD Loading failed (Volume)\r\n");
  		ToFSensor_failure();
  		return status_1;
  	}
  	if(status_2)
	{
		printf("Pitch VL53L7CX ULD Loading failed (pitch)\r\n");
		ToFSensor_failure();
		return status_2;
	}
  	printf("VL53L7CX ULD ready ! (Version : %s)\r\n",VL53L7CX_API_REVISION);
  	ToFSensor_sucess();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	ADC_Chan_Config.Rank = ADC_REGULAR_RANK_1;
	ADC_Chan_Config.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
	ADC_Chan_Config.SingleDiff = ADC_SINGLE_ENDED;
	ADC_Chan_Config.OffsetNumber = ADC_OFFSET_NONE;
	ADC_Chan_Config.Offset = 0;
	uint8_t root = 60, vol_scale = 6, note = 0, vol = 0, fx; // defualt is middle C
	uint32_t tempo = 0, Trb_Bass = 0;
//  int fx_zone = 60;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // pedal effect (tempo & treble mute & bass enhance)
	  Get_ADC_Vals(&tempo, &Trb_Bass);
	  uint8_t bass = Trb_Bass/ 267;
	  int treble = -Trb_Bass/ 500;
	  midi_Treble_Bass(treble, bass);

	  uint8_t data_ready_1 = 0, data_ready_2 = 0;

	  status_1 = vl53l7cx_check_data_ready(&Dev_1, &data_ready_1);
	  status_2 = vl53l7cx_check_data_ready(&Dev_2, &data_ready_2);

	  if (!data_ready_1) {
		  HAL_Delay(1);      // small backoff; frame rate set by sensor
		  continue;
	  }
	  if (!data_ready_2) {
		  HAL_Delay(1);      // small backoff; frame rate set by sensor
		  continue;
	  }

	  // 2) Read the fresh frame
	  data_ready_1 = vl53l7cx_get_ranging_data(&Dev_1, &Results_1);
	  data_ready_2 = vl53l7cx_get_ranging_data(&Dev_2, &Results_2);
	 if (data_ready_1) {
		 printf("Volume: get_ranging_data error: %u\r\n", data_ready_1);
		 HAL_Delay(5);
		 continue;
	 }
	 if (data_ready_2) {
		 printf("Pitch: get_ranging_data error: %u\r\n", data_ready_2);
		 HAL_Delay(5);
		 continue;
	 }


	 // 4) Copy/pack only VALID distances (status 5) for display
	 for (int zone = 0; zone < resolution; zone++) {
//		 uint8_t nb_P = Results_1.nb_target_detected[zone], nb_V = Results_2.nb_target_detected[zone];
//		 uint8_t st_P = Results_1.target_status[zone], st_V = Results_2.target_status[zone];
		 uint16_t cm_1 = 0, cm_2 = 0;

		cm_1 = (uint16_t)Results_1.distance_mm[zone]/10;
		cm_2 = (uint16_t)Results_2.distance_mm[zone]/10;

		 Tof_values_1[zone] = cm_1/3; // 3 cm levels
		 Tof_values_2[zone] = cm_2/3;
	 }
	 	 // spot to add playing notes //
	 vol = 127 - vol_scale*_4ZonesVal(Tof_values_1, 1, CENTER_ZONE_1, CENTER_ZONE_2, CENTER_ZONE_3, CENTER_ZONE_4); // Close = load & far = quiet
	 uint8_t left_center = _4ZonesVal(Tof_values_2, 2, CENTER_ZONE_3, CENTER_ZONE_4, LEFT_CENTER_1, LEFT_CENTER_2);
	 uint8_t right_center = _4ZonesVal(Tof_values_2, 2, CENTER_ZONE_1, CENTER_ZONE_2, RIGHT_CENTER_1, RIGHT_CENTER_2);
	 midiNoteOff(0, note, 127);
	 if (left_center <= right_center)
	 {
		 note = left_center;
		 fx = 0; 					// no reverb
	 } else
	 {
		 note = right_center;
		 fx = 1;   					//  reverb
	 }
	 if (note > 12) note = 12; 		// stay within 1 octave
	 note += root;					// note  from the base note
	 if (vol < 0 || vol > 127) vol = 0;			// data validation
	 if (note > 127) note = 127;
	 // 5) Example print:
//	 uint8_t Z1=2,Z2=3,Z3=4,Z4=5;
	 printf("Volume: %02u|\t", vol);
	 printf("Note: %02u|\tReverb: %u|\tTempo: %03u|\tTreble&Bass: %02d\t%02u\r\n", note, fx, tempo, treble, bass);
//	 printf("Zone %d value: %u| Zone %d value: %u| Zone %d value: %u| Zone %d value: %u\r\n", Z1, Tof_values_2[Z1], Z2, Tof_values_2[Z2], Z3, Tof_values_2[Z3], Z4, Tof_values_2[Z4]);
	 midi_SetChannelVolume(0, vol);
	 midi_SetChannelReverb(0, fx);
	 midiNoteOn(0, note, 127);
	 HAL_Delay(tempo);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
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
  hi2c1.Init.Timing = 0x00000508;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000508;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
