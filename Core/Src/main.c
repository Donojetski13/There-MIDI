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
#include "stdio.h"
#include <stdint.h>
extern const uint8_t blank_home[];
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
#define ToF_W 0x52
#define ToF_R 0x53
#define _4x4 ((uint8_t) 16U)
#define _8x8 ((uint8_t) 64U)
#define continuous 1U
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
#define LED_MAX 6
#define USE_BRIGHTNESS 1 // 1 = control brightness, 0 = not controlling brightness
#define MAX_PEDAL_VAL 3000U
#define PI 3.14159265
#define TIM_1 ((TIM_TypeDef *) (((0x40000000UL) + 0x00010000UL) + 0x2C00UL))

#define OCTAVE 12U
#define PLAY_ZONE_SIZE 4
#define NOTE_RANGE 18
#define NUM_SCALES 3
#define NUM_INSTRUMENTS 17

#define B1_PORT			GPIOE
#define B1_PIN			GPIO_PIN_10 // last SPI status

#define	B2_PORT			GPIOE
#define	B2_PIN			GPIO_PIN_7 // PB0 reset

#define B3_PORT			GPIOE
#define B3_PIN			GPIO_PIN_8 // command start

#define B4_PORT			GPIOF
#define B4_PIN			GPIO_PIN_15 // Data start

#define B5_PORT			GPIOE
#define B5_PIN			GPIO_PIN_13 // Data start



uint8_t resolution=_8x8, ranging_frequency=15, sharpener_percent=20;
uint16_t integration_time=20;
uint8_t data_to_transfer=0;
uint16_t Tof_values_1[64], Tof_values_2[64]; // 64(8x8) max size

ADC_ChannelConfTypeDef ADC_Chan_Config = {0};
uint32_t ADC_vals[NUM_CHANS], ADC_Channels[NUM_CHANS] = {ADC_CHANNEL_1, ADC_CHANNEL_4};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define TEXT_X 125
#define TEXT_Y 210

#define Oct_text_X 260
#define Oct_text_Y 140

#define Scale_text_X 20
#define Scale_text_Y 120
#define Scale_Spacing 25

#define inst_X 110
#define inst_Y 65

int scale_count = 0;
int inst_counter = 0;

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	HAL_NVIC_SystemReset();
}

char* Intruments[NUM_INSTRUMENTS] = {"Piano", "Vibraphone", "Acoustic Guitar", "Electric Guitar", "Violin", "Trumpet", "Alto Saxophone", "Ocarina", "Sci-Fi", "Timpani", "Goblins", "Helicopter", "Flute",
									 "Choir", "Marimba", "Telephone", "Gunshots"};
char* Notes[29] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B","C", "C#", "D", "D#", "E"};

uint8_t scales[NUM_SCALES][NOTE_RANGE+1] = {{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18},{0,2,4,5,7,9,11,12,14,16,17,19,21,23,24,26,28},{0,2,3,5,7,8,10,12,14,15,17,19,20,22,24,26,27}};
uint8_t instruments[NUM_INSTRUMENTS] = {MEL_GRAND_PIANO, MEL_VIBRAPHONE, MEL_A_GUITAR,
									    MEL_E_GUITAR, MEL_VIOLIN, MEL_TRUMPET, MEL_A_SAX,
									    MEL_OCARINA, MEL_SCIFI, MEL_Timpani, MEL_GOBLINS, MEL_HELICOPTER,
										MEL_FLUTE, MEL_CHOIR, MEL_MARIMBA, MEL_TELEPHONE, MEL_GUNSHOTS};
uint8_t LED_Data_1[LED_MAX][4], LED_Data_2[LED_MAX][4];
uint8_t LED_Mod_1[LED_MAX][4], LED_Mod_2[LED_MAX][4]; // for brightness

int datasentflag_1 = 0; // ensures DMA is not sending another data when first DMA is still being transmitted
int datasentflag_2 = 0;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (  htim->Instance == TIM_1)
	{
		HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
		datasentflag_1 = 1;
	}
	else
	{
		HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
		datasentflag_2 = 1;
	}
}

void Set_LED (uint8_t (*LED)[4], int LEDnum, int Red, int Green, int Blue) {
	LED[LEDnum][0] = LEDnum;
	LED[LEDnum][1] = Green;
	LED[LEDnum][2] = Red;
	LED[LEDnum][3] = Blue;
}

void Set_Brightness(uint8_t (*LED_D)[4], uint8_t (*LED_M)[4], int brightness) // 0 - 45
{
#if USE_BRIGHTNESS
	if (brightness > 45) {
		brightness = 45;
	}

	for (int i = 0; i < LED_MAX; i++) {
		LED_M[i][0] = LED_D[i][0];

		for (int j = 1; j < 4; j++) {
			float angle = 90 - brightness; // in degrees
			angle = angle * PI / 180; // in radians
			LED_M[i][j] = (LED_D[i][j]);
		}
	}
#endif
}

uint32_t pwmData_1[(24*LED_MAX) + 50];
uint32_t pwmData_2[(24*LED_MAX) + 50];

void WS2812_Send (uint32_t* pmw_D, uint8_t (*LED_M)[4], int timer) {
	uint32_t indx = 0;
	uint32_t color;

	for (int i = 0; i < LED_MAX; i++) {
		color = ((LED_M[i][1] << 16) | (LED_M[i][2] << 8) | (LED_M[i][3])); // GREEN RED BLUE

		for (int j = 23; j >= 0; j--) {
			if (color & (1<<j)) {
				pmw_D[indx] = 84; // 2/3 of 149 (reload value)
			}
			else {
				pmw_D[indx] = 42; // 1/3 of 149 (reload value)
			}

			indx++;
		}
	}

	// sending 50 bits of 0 to signify end of data transfer
	for (int i = 0; i < 50; i++) {
		pmw_D[indx] = 0;
		indx++;
	}
	if (timer == 1)
	{
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pmw_D, indx);
		while (!datasentflag_1) {}; // DMA has been stopped and can send another set of data now
		datasentflag_1 = 0;
	} else
	{
		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)pmw_D, indx);
		while (!datasentflag_2) {}; // DMA has been stopped and can send another set of data now
		datasentflag_2 = 0;
	}
}

void send(uint32_t* pmw_D, int Green, int Red, int Blue, int timer) {
	uint32_t data = (Green << 16) | (Red << 8) | Blue;

	for (int i = 23; i >= 0; i--) {
		if (data & (1 << i)) {
			pmw_D[i] = 26;
		}
		else {
			pmw_D[i] = 13;
		}
	}

	if (timer == 1)
	{
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, pmw_D, 24);
	} else
	{
		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pmw_D, 24);
	}
}
 void sensor2LED(uint8_t output, uint8_t sensor) {
	 uint8_t num, light;
	 if (sensor == 1)
	 {
		 num = output / 21;
		 light = (int)(output / 10.5) % 2;
	 }
	 else
	 {
		 num = output / 2;
		 light = output % 2;
		 if (num > 6) num = 6;
	 }

	 // start with all off
	 for (int i = 0; i < LED_MAX; i++)
	 	 {
	 		 if (sensor == 1)Set_LED(LED_Data_2, (LED_MAX-1-i),0,0,0);

	 		 else Set_LED(LED_Data_1, (LED_MAX-1-i),0,0,0);
	 	 }
	 int blue =55, green = 80;
	 for (int i = 0; i < (num+light); i++)
	 {
		 if (sensor == 1)
			 if (light && (i == (num+light)-1)) Set_LED(LED_Data_2, (LED_MAX-1-i),0,0,blue);
			 else Set_LED(LED_Data_2, (LED_MAX-1-i),0,green,blue);

		 else
			 if (light && (i == (num+light)-1)) Set_LED(LED_Data_1, (LED_MAX-1-i),0,0,blue);
			 else Set_LED(LED_Data_1, (LED_MAX-1-i),0,green,blue);
	 }

	 if (sensor == 1)
	 {
		 Set_Brightness(LED_Data_2, LED_Mod_2, 25);
		 WS2812_Send(pwmData_2, LED_Mod_2,2);
	 }
	 else
	 {
		 Set_Brightness(LED_Data_1, LED_Mod_1, 25);
		 WS2812_Send(pwmData_1, LED_Mod_1,1);
	 }


 }
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

	if(*tempo < 50) {
		*tempo = 50;
	}

	*treble_bass = ADC_vals[1];
}

uint16_t Button1[4] = {}, Button2[4] = {}, Button3[4] = {}, Button4[4] = {}, Button5[4] = {}; // Xmin, Xmax, Ymin, Ymax
int zoneClicked (uint16_t x, uint16_t y, uint16_t* button){
	return (((x > button[0]) && (x < button[1])) && ((y > button[2]) && (y < button[3])));
}

//int instr = 0;
void change_instrument(int direction)
{
	int index = inst_counter % NUM_INSTRUMENTS;          // may still be negative
	if (index < 0)
		index += NUM_INSTRUMENTS;

	midi_SetInstrument(0,instruments[(index)]);
	printf("MIDI Index: %d\r\n", index);
}

int root = 60;
void change_octave(int direction)
{
	if (direction)
	{
		root += 12;
		if (root > 120) root = 108;
	}
	else
	{
		root -= 12;
		if (root < 0) root = 0;
	}
}
int scale = 0;
void change_scale(void)
{
	if (++scale == NUM_SCALES) scale = 0;
}

void intToSignedStr(int n, char *out)
{
    if (n > 0) {
        // Add '+' then number
        out[0] = '+';
        sprintf(out + 1, "%d", n);
    }
    else if(n == 0 ) {
    	out[0] = ' ';
    	sprintf(out + 1, "%d", n);
    }
    else {
        // For zero and negatives, sprintf is correct:
        // 0   -> "0"
        // -5  -> "-5"
        sprintf(out, "%d", n);
    }
}

void drawScale() {
	  char * chro = "CHRO";
	  char * major = "MAJOR";
	  char * minor = "MINOR";

	  if(scale_count % 3 == 0) {
		  drawText(Scale_text_X, Scale_text_Y, chro, ILI9488_RED, 2);
		  drawText(Scale_text_X, Scale_text_Y + Scale_Spacing, major, ILI9488_WHITE, 2);
		  drawText(Scale_text_X, Scale_text_Y + (2 * Scale_Spacing), minor, ILI9488_WHITE, 2);
	  }
	  else if(scale_count % 3 == 1) {
		  drawText(Scale_text_X, Scale_text_Y, chro, ILI9488_WHITE, 2);
		  drawText(Scale_text_X, Scale_text_Y + Scale_Spacing, major, ILI9488_RED, 2);
		  drawText(Scale_text_X, Scale_text_Y + (2 * Scale_Spacing), minor, ILI9488_WHITE, 2);
	  }
	  else {
		  drawText(Scale_text_X, Scale_text_Y, chro, ILI9488_WHITE, 2);
		  drawText(Scale_text_X, Scale_text_Y + Scale_Spacing, major, ILI9488_WHITE, 2);
		  drawText(Scale_text_X, Scale_text_Y + (2 * Scale_Spacing), minor, ILI9488_RED, 2);
	  }

	  scale_count++;
}

void draw_inst() {
	int index = inst_counter % NUM_INSTRUMENTS;          // may still be negative
	if (index < 0)
		index += NUM_INSTRUMENTS;
	char * temp = "                      ";
	drawTextBG(inst_X, inst_Y, temp, ILI9488_WHITE, ILI9488_BLACK, 2);
	drawTextBG(inst_X, inst_Y, Intruments[index], ILI9488_WHITE, ILI9488_BLACK, 2);

	printf("Display Index: %d\r\n", index);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  //ILI9488_Init();
  xpt2046_init();
  ILI9488_Init();

  setRotation(2);

  drawImage(blank_home, 0, 0, 320, 480);

  char oct[2];
  int oct_int = 0;
  intToSignedStr(oct_int, oct);
  drawText(Oct_text_X, Oct_text_Y, oct, ILI9488_WHITE, 4);

  drawScale();

  draw_inst();

  xpt2046_spi(&hspi3);
  xpt2046_orientation(2);

  uint16_t x;
  uint16_t y;

  status_1 = sensorInit(&Dev_1, 1, &isAlive_1);
  	status_2 = sensorInit(&Dev_2, 2, &isAlive_2);
  	printf("MIDI Board initialization\r\n");
  	if(midi_Init())
  	{
  		printf("MIDI Board failed\r\n");
  		HAL_NVIC_SystemReset();
  		return 3;
  	}
    	if(status_1)
    	{
    		printf("Volume VL53L7CX ULD Loading failed (Volume)\r\n");
    		ToFSensor_failure();
    		HAL_NVIC_SystemReset();
    		return 1;
    	}
    	if(status_2)
  	{
  		printf("Pitch VL53L7CX ULD Loading failed (pitch)\r\n");
  		ToFSensor_failure();
  		HAL_NVIC_SystemReset();
  		return 2;
  	}
    	printf("VL53L7CX ULD ready ! (Version : %s)\r\n",VL53L7CX_API_REVISION);
    	ToFSensor_sucess();



  ADC_Chan_Config.Rank = ADC_REGULAR_RANK_1;
  	ADC_Chan_Config.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  	ADC_Chan_Config.SingleDiff = ADC_SINGLE_ENDED;
  	ADC_Chan_Config.OffsetNumber = ADC_OFFSET_NONE;
  	ADC_Chan_Config.Offset = 0;
   	uint8_t vol_scale = 6, note = 0, vol = 0, last_note, fx; // defualt is middle C
  	uint32_t tempo = 0, Trb_Bass = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  xpt2046_read_touch(&x, &y);
	  printf("X: %d, Y: %d \n\r", x, y);


	  //octave+
	  if(x != 0 && x > 240 && x < 380 && y != 0 && y < 140) {
		  oct_int++;

		  if(oct_int > 4) {
			  oct_int = 4;
		  }

		  intToSignedStr(oct_int, oct);
		  int temp;
		  if(oct_int > 0) {
			  temp = ILI9488_RED;
		  }
		  else if(oct_int < 0) {
			  temp = ILI9488_BLUE;
		  }
		  else {
			  temp = ILI9488_WHITE;
		  }
		  drawTextBG(Oct_text_X, Oct_text_Y, oct, temp, ILI9488_BLACK, 4);
		  change_octave(1);
	  }
	  //octave -
	  if(x != 0 && x > 380 && x < 460 && y != 0 && y < 140) {
	  		  oct_int--;

	  		if(oct_int < -5) {
				  oct_int = -5;
			  }

	  		  intToSignedStr(oct_int, oct);
	  		  int temp;
	  		  if(oct_int > 0) {
	  			  temp = ILI9488_RED;
	  		  }
	  		  else if(oct_int < 0) {
	  			  temp = ILI9488_BLUE;
	  		  }
	  		  else{
					  temp = ILI9488_WHITE;
				  }
	  		  drawTextBG(Oct_text_X, Oct_text_Y, oct, temp, ILI9488_BLACK, 4);
	  		change_octave(0);
	  	  }

	  //scale
	  if(x != 0 && x > 100 && x < 230 && y != 0 && y < 220 && y > 70) {
		  drawScale();
		  change_scale();
	  }

	  //inst +
	  if(x != 0 && x > 240 && x < 380 && y != 0 && y > 160) {
			inst_counter++;
			draw_inst();
			change_instrument(1);
	  }

	  //inst-
	  if(x != 0 && x > 390 && x < 460 && y != 0 && y > 150) {
		  inst_counter--;
		  draw_inst();
		  change_instrument(0);
	  }

	  last_note = note;
	  	  Get_ADC_Vals(&tempo, &Trb_Bass);
	  	  tempo *= 2; Trb_Bass *= 2; // scale pedal values
	  	  uint8_t bass = Trb_Bass/ (2*MAX_PEDAL_VAL/15); 		// process bass value
	  	  int treble = (Trb_Bass)/ (2*MAX_PEDAL_VAL/8);	// process treble value
	  	  treble *= -1;
	  	  midi_Treble_Bass(treble, bass);
//	  	  if (HAL_GPIO_ReadPin(B1_PORT, B1_PIN)) change_scale();
//	  	  if (HAL_GPIO_ReadPin(B2_PORT, B2_PIN)) change_instrument(0);//down
//	  	  if (HAL_GPIO_ReadPin(B3_PORT, B3_PIN)) change_instrument(1);//up
//	  	  if (HAL_GPIO_ReadPin(B4_PORT, B4_PIN)) change_octave(0);
//	  	  if (HAL_GPIO_ReadPin(B5_PORT, B5_PIN)) change_octave(1);
//
	  	  uint8_t data_ready_1 = 0, data_ready_2 = 0;
//
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

//	  	  // 2) Read the fresh frame
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

//
//	  	 // 4) Copy/pack only VALID distances (status 5) for display
	  	 for (int zone = 0; zone < resolution; zone++) {
	  		 uint16_t cm_1 = 0, cm_2 = 0;

	  		cm_1 = (uint16_t)Results_1.distance_mm[zone]/10;
	  		cm_2 = (uint16_t)Results_2.distance_mm[zone]/10;

	  		 Tof_values_1[zone] = cm_1/PLAY_ZONE_SIZE; // [size] cm levels
	  		 Tof_values_2[zone] = cm_2/PLAY_ZONE_SIZE;
	  	 }
//	  	 	 // spot to add playing notes //
	  	 uint16_t _vol = vol_scale*_4ZonesVal(Tof_values_1, 1, CENTER_ZONE_1, CENTER_ZONE_2, CENTER_ZONE_3, CENTER_ZONE_4);
	  	 if (_vol > 127) _vol = 127;			// limit value
	  	 vol = 127 - _vol; // Close = load & far = quiet
	  	 uint8_t left_center = _4ZonesVal(Tof_values_2, 2, CENTER_ZONE_3, CENTER_ZONE_4, LEFT_CENTER_1, LEFT_CENTER_2);
	  	 uint8_t right_center = _4ZonesVal(Tof_values_2, 2, CENTER_ZONE_1, CENTER_ZONE_2, RIGHT_CENTER_1, RIGHT_CENTER_2);
	  	 if (left_center <= right_center)
	  	 {
	  		 note = left_center;
	  		 fx = 0; 					// no reverb
	  	 } else
	  	 {
	  		 note = right_center;
	  		 fx = 1;   					//  reverb
	  	 }
	  	 if (note > NOTE_RANGE) note = NOTE_RANGE; 		// stay within 1.5 octaves

	  	 sensor2LED(vol, 1);    // volume and pitch LED level
	  	 sensor2LED(note, 2);
	  	 note = scales[scale][note];    // set scale

	  	drawNoteWithSharpBG(TEXT_X, 320-TEXT_Y, Notes[note], ILI9488_BLACK, ILI9488_WHITE, 13, 5);

	  	//drawChar(TEXT_X, 320-TEXT_Y, 'C', ILI9488_BLACK, 13);//middle note

	  	 note += root;					// note  from the base note
	  	 if (note > 127) note = 127;
//	  	 // 5) Example print:
	  //	 uint8_t Z1=2,Z2=3,Z3=4,Z4=5;
	  	// printf("Volume: %02u|\t", vol);
	  	// printf("Note: %02u|\tReverb: %u|\tTempo: %04u|\tTreble&Bass: %d\t%02u|\tInstrument: %d|\tRoot: %d|\tScale: %d\r\n", note, fx, tempo, treble, bass, instr, oct, scale);
	  //	 printf("Zone %d value: %u| Zone %d value: %u| Zone %d value: %u| Zone %d value: %u\r\n", Z1, Tof_values_2[Z1], Z2, Tof_values_2[Z2], Z3, Tof_values_2[Z3], Z4, Tof_values_2[Z4]);
	  	 midi_SetChannelVolume(0, vol);
	  	 midi_SetChannelReverb(0, fx);
	  	 midiNoteOff(0, last_note, 127);
	  	 midiNoteOn(0, note, 127);



	  HAL_Delay(tempo);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
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
  hi2c1.Init.Timing = 0x30A175AB;
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
  hi2c2.Init.Timing = 0x30A175AB;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 149;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 149;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
