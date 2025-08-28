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
#include <stdio.h>
#include <stdlib.h>
#define ARM_MATH_CM4
#include "arm_math.h"
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
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define DMABUFLEN 512
#define BUFLEN 256
#define FFTBUFLEN 2048
volatile static uint16_t dmaBuffer[DMABUFLEN] = {0};
float32_t hannWindow1[BUFLEN];
float32_t hannWindow2[2*BUFLEN];
float32_t hannWindow3[BUFLEN/2];
uint16_t signal0[2*BUFLEN];
float32_t signal0filtered[2*BUFLEN];
uint16_t signalbuffer0[2*BUFLEN];
float32_t signalfilteredbuffer0[2*BUFLEN];
uint16_t maxpp[6];
uint16_t maxpp0 = 0;
uint16_t maxpp1 = 0;
uint16_t maxpp2 = 0;
uint16_t maxpp3 = 0;
uint16_t maxpp4 = 0;
uint16_t maxpp5 = 0;
uint16_t fund[6];
float32_t fundamental0 = 0;
float32_t fundamental1 = 0;
float32_t fundamental2 = 0;
float32_t fundamental3 = 0;
float32_t fundamental4 = 0;
float32_t fundamental5 = 0;
uint8_t signalnumber = 0;
uint32_t print = 0;


uint16_t FreqNotes[47] = {
  78, 82, 87, 93, 98, 104, 110, 116, 124, 130, 139, 147, 155, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 293, 311, 330, 349, 370, 392,
  415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109
};
uint16_t FreqRange[46];
uint16_t NoteNew[6];
uint16_t NoteOld[6];
uint16_t maxppOld[6];
int16_t Delta[6];
uint8_t txBuffer[6];
//uint16_t Treshold[6] = {1000, 1000, 1000, 1000, 1000, 1000};
uint16_t Treshold[6] = {800, 800, 800, 400, 750, 650};

uint8_t FIR_LENGHT = 31;
float32_t fir_coefficients10k[31]  = {-0.0021043, 0.0000019, 0.0046466, -0.0000007, -0.0094219, 0.0000029, 0.0172049, -0.0000020, -0.0297785, 0.0000052, 0.0515218, -0.0000041, -0.0984164, 0.0000048, 0.3156787, 0.4999957, 0.3156787, 0.0000048, -0.0984164, -0.0000041, 0.0515218, 0.0000052, -0.0297785, -0.0000020, 0.0172049, 0.0000029, -0.0094219, -0.0000007, 0.0046466, 0.0000019, -0.0021043};
float32_t fir_coefficients5k[31]  =  {0.0005669, -0.0002630, -0.0022233, -0.0038547, -0.0018613, 0.0053228, 0.0135453, 0.0133131, -0.0027602, -0.0295939, -0.0460559, -0.0251788, 0.0457091, 0.1495368, 0.2430896, 0.2807522, 0.2430896, 0.1495368, 0.0457091, -0.0251788, -0.0460559, -0.0295939, -0.0027602, 0.0133131, 0.0135453, 0.0053228, -0.0018613, -0.0038547, -0.0022233, -0.00026530, 0.0005669};
float32_t fir_coefficients2_5k[31] = {0.0064131, 0.0057191, 0.0051066, 0.0013561, -0.0056292, -0.0145003, -0.0224276, -0.0255772, -0.0201251, -0.0035005, 0.0244877, 0.0610609, 0.1007693, 0.1365633, 0.1614632, 0.1703842, 0.1614632, 0.1365633, 0.1007693, 0.0610609, 0.0244877, -0.0035005, -0.0201251, -0.0255772, -0.0224276, -0.0145003, -0.0056292, 0.0013561, 0.0051066, 0.0057191, 0.0064131};
arm_fir_instance_f32 fir_instance0;
float32_t fir_in_arm0;
float32_t fir_out_arm0;
float32_t fir_state0[31];


// SINE_TABLE_SIZE = Fs / Fsine
#define SINE_TABLE_SIZE 45
uint16_t sine_table[SINE_TABLE_SIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */
void filter(uint16_t inputArray[], uint16_t size, float32_t coefficients[], uint16_t outputArray[], uint16_t lastSamples[]);
void calculateFFT(float32_t inputArray[], uint16_t size, uint16_t* maxpp, float32_t* fundamental, uint16_t fs);
void FundamentalToNote(uint16_t FundamentalNew[], uint16_t NoteNew[6], uint16_t FreqRange[46]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}

void generate_sine_table(void){
	for(int i = 0; i<SINE_TABLE_SIZE; i++){
		float angle = (2.0f * M_PI * i)/ (float)SINE_TABLE_SIZE;
		float sine_value = (sinf(angle) + 1.0f)/2.0f;
		sine_table[i] = (uint16_t)(sine_value * 4095);
	}
}

void calculateFFT(float32_t inputArray[], uint16_t size, uint16_t* maxpp, float32_t* fundamental, uint16_t fs) {
	// Zmiana stanu na pinie PB10 Debug
	HAL_GPIO_TogglePin(CALCOutput_GPIO_Port, CALCOutput_Pin);
	//Przekonwertowanie sygnału z uint16 na float32
    float32_t signal_float32[size];
	for (int i = 0; i < size; i++){
		signal_float32[i] = inputArray[i];
	}
	//Obliczenie wartości średniej sygnału/składowej stałej oraz jej odjęcie
    float32_t mean = 0.0f;
    arm_mean_f32(signal_float32, size,  &mean);
    arm_offset_f32(signal_float32, -mean, signal_float32, size);
    //Nałożenie okna Hanna na sygnał
    float32_t signal_filtered[size];
	if(size==BUFLEN){
		arm_mult_f32(signal_float32, hannWindow1, signal_filtered, size);
	}else if(size==2*BUFLEN)
		arm_mult_f32(signal_float32, hannWindow2, signal_filtered, size);
	else arm_mult_f32(signal_float32, hannWindow3, signal_filtered, size);
	//Znalezienie wartości międzyszczytowych
	float32_t max;
	float32_t min;
	uint32_t maxIndex;
	uint32_t minIndex;
	arm_max_f32(signal_filtered, size, &max, &maxIndex);
	arm_min_f32(signal_filtered, size, &min, &minIndex);

	//Obliczenie FFT
	float32_t signal_fft[FFTBUFLEN] = {0};
	for(int i = 0; i<size; i++)
		signal_fft[i]=signal_filtered[i];

	float32_t fft_Bufor[FFTBUFLEN];
	arm_rfft_fast_instance_f32 rfft_inst;
	arm_rfft_fast_init_f32(&rfft_inst, FFTBUFLEN);

	arm_rfft_fast_f32(&rfft_inst, signal_fft, fft_Bufor, 0);

	//Obliczenie amplitud poszczególnych prążków
	float32_t fft_wynik[FFTBUFLEN/2];
		float32_t fft_max = 0;
		uint16_t fft_max_index = 0;
		for(int i = 12; i < (FFTBUFLEN/2); i++){
			fft_wynik[i]=sqrtf((fft_Bufor[2*i]*fft_Bufor[2*i])+(fft_Bufor[2*i+1]*fft_Bufor[2*i+1]));
			if(fft_wynik[i]>fft_max){
				fft_max=fft_wynik[i];
				fft_max_index = i;
			}
		}
		//Normalizacja wyniku FFT
		for(int i = 0; i < (FFTBUFLEN/2); i++){
				fft_wynik[i]=fft_wynik[i]/fft_max;
			}
//		for(int i = 0; i < (FFTBUFLEN/2); i++){
//					if(fft_wynik[i]< 0.3)
//						fft_wynik[i] = 0;
//				}
		//Odnalezienie indeksu częstotliwości podstawowej
		if(fft_max_index > 0){
			while ((fft_wynik[fft_max_index / 2] > 0.50) || (fft_wynik[fft_max_index / 2 - 1] > 0.50) || (fft_wynik[fft_max_index / 2 + 1] > 0.50)) {
				fft_max_index /= 2;
			}
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index + 1] > fft_wynik[fft_max_index]) {
			fft_max_index++;
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index - 1] > fft_wynik[fft_max_index]) {
			fft_max_index--;
		}
		if(fft_max_index > 0){
			while ((fft_wynik[fft_max_index / 3] > 0.50) || (fft_wynik[fft_max_index / 3 - 1] > 0.50) || (fft_wynik[fft_max_index / 3 + 1] > 0.50)) {
				fft_max_index /= 3;
			}
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index + 1] > fft_wynik[fft_max_index]) {
			fft_max_index++;
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index - 1] > fft_wynik[fft_max_index]) {
			fft_max_index--;
		}
		if(fft_max_index > 0){
			while ((fft_wynik[fft_max_index / 4] > 0.50) || (fft_wynik[fft_max_index / 4 - 1] > 0.50) || (fft_wynik[fft_max_index / 4 + 1] > 0.50)) {
				fft_max_index /= 4;
			}
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index + 1] > fft_wynik[fft_max_index]) {
			fft_max_index++;
		}
		while (fft_max_index > 0
				&& fft_wynik[fft_max_index - 1] > fft_wynik[fft_max_index]) {
			fft_max_index--;
		}
		float32_t fundamental_freq = (((float32_t)fft_max_index) * (float32_t)fs / (float32_t)FFTBUFLEN);

		int16_t peak_to_peak_value = max - min;
		*maxpp = peak_to_peak_value;
		*fundamental = fundamental_freq;

		uint16_t printall = 0;
		if(printall == 1){
			printf("\nRaw signal\n");
			for(int i = 0; i<2*BUFLEN; i++){
				printf("%d, ", inputArray[i]);
			}
			printf("\nSignal filtered\n");
			for(int i = 0; i<2*BUFLEN; i++){
				printf("%f, ", signal_filtered[i]);
			}
			printf("\nFFT\n");
			for(int i = 0; i<FFTBUFLEN/2; i++){
				printf("%f, ", fft_wynik[i]);
			}
		}
}



void FundamentalToNote(uint16_t FundamentalNew[], uint16_t NoteNew[6], uint16_t FreqRange[46]) {
  for (int i = 0; i < 6; i++) {
    int note = 0;
    for (int j = 0; j < 46; j++) {
      if (FundamentalNew[i] >= FreqRange[j] && FundamentalNew[i] < FreqRange[j + 1]) {
        note = j;  // Przypisz odpowiednią notę
        break;  // Przerwij pętlę po znalezieniu odpowiedniego zakresu
      }
    }

    NoteNew[i] = note;
  }
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  arm_fir_init_f32(&fir_instance0, FIR_LENGHT, fir_coefficients10k, fir_state0, 1);

  for(int i = 0; i < 46; i++){
	  FreqRange[i] = (FreqNotes[i] + FreqNotes[i+1])/2;
  }
  for(int i = 0; i < BUFLEN; i++){
	  hannWindow1[i] = (0.5 - (0.5 * cos ( (2.0 * PI * i) / (BUFLEN - 1))));
  }
  for(int i = 0; i < 2*BUFLEN; i++){
  	  hannWindow2[i] = (0.5 - (0.5 * cos ( (2.0 * PI * i) / (2*BUFLEN - 1))));
  }
  for(int i = 0; i < BUFLEN/2; i++){
	  hannWindow3[i] = (0.5 - (0.5 * cos ( (2.0 * PI * i) / (BUFLEN/2 - 1))));
  }
  generate_sine_table();
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)sine_table, SINE_TABLE_SIZE, DAC_ALIGN_12B_R);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dmaBuffer, DMABUFLEN);
    HAL_Delay(10);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  htim2.Init.Prescaler = 59;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DMAOutput_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CALCOutput_GPIO_Port, CALCOutput_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : CALCOutput_Pin */
  GPIO_InitStruct.Pin = CALCOutput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CALCOutput_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DMAOutput_Pin */
  GPIO_InitStruct.Pin = DMAOutput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DMAOutput_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Zmiana stanu na pinie D7
	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);

	//Przepisanie sygnału do tablicy
	for(uint32_t i = 0; i < 2 * BUFLEN; i++){
		signalbuffer0[i] = dmaBuffer[i];
	}

	//Wystartowanie kolejnego cyklu próbkowania
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) dmaBuffer, DMABUFLEN);

	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);

	//Przefiltrowanie sygnałów
	for(int i = 0; i < 2 * BUFLEN; i++){
		fir_in_arm0 = (float32_t) signalbuffer0[i];
		arm_fir_f32(&fir_instance0, &fir_in_arm0, &fir_out_arm0, 1);
		signal0filtered[i] = fir_out_arm0;
	}
	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);


		//Obliczenie częstotliwości podstawowej
			calculateFFT(signal0filtered, 2*BUFLEN, &maxpp0, &fundamental0, 10000);
			fund[0] = (uint16_t)fundamental0;
			maxpp[0] = maxpp0;
			printf("F0 = %d\n", fund[0]);
//			printf("Max = %d\n", maxpp[0]);




	uint16_t data[2];
	data[0] = (uint16_t)fundamental0;
	data[1] = maxpp0;

	HAL_UART_Transmit_DMA(&huart3, (uint8_t *)data, sizeof(data) * 2);
//	HAL_UART_Transmit(&huart3, (uint8_t *)data, sizeof(data) * 2, 10);

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Zmiana stanu na pinie D7
	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);

	//Przesunięcie próbek
	for(int i = 0; i < BUFLEN; i++){
		signalbuffer0[i] = signalbuffer0[i + BUFLEN];
	}

	//Rozdzielenie sygnałów do osobnych tablic
	for(int i = 0; i < BUFLEN; i++){
		signalbuffer0[i + BUFLEN] = dmaBuffer[i];
	}
	//Przefiltrowanie sygnałów
	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);
	for(int i = 0; i < 2 * BUFLEN; i++){
		fir_in_arm0 = (float32_t) signalbuffer0[i];
		arm_fir_f32(&fir_instance0, &fir_in_arm0, &fir_out_arm0, 1);
		signal0filtered[i] = fir_out_arm0;
	}
	HAL_GPIO_TogglePin(DMAOutput_GPIO_Port, DMAOutput_Pin);

		//Obliczenie częstotliwości podstawowej
		calculateFFT(signal0filtered, 2*BUFLEN, &maxpp0, &fundamental0, 10000);
		fund[0] = (uint16_t)fundamental0;
		maxpp[0] = maxpp0;
		printf("F0 = %d\n", fund[0]);
//		printf("Max = %d\n", maxpp[0]);

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
