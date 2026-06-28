/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"
#include "app_touchgfx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <minmea.h>
#include "core_cm4.h"
#include "bms_jk.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SIF_BYTES 12U
#define GPS_LINE_MAX 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile int32_t velocity = 0;
volatile uint8_t faultCode = 0;
volatile uint8_t reverse = 0;
volatile uint8_t fardriver_mode = 0;

volatile uint8_t seinL_val   = 0;
volatile uint8_t seinR_val   = 0;
volatile uint8_t hazard_val  = 0;
volatile uint8_t beam_val    = 0;
volatile uint8_t batt_val    = 0;
volatile uint8_t charge_val  = 0;

/* === ESP32 (VESC via UART) ====================================== */
#define ESP_RX_BUF_SIZE 64
volatile uint8_t  esp_rx_byte;                 /* byte sementara IT      */
volatile char     esp_line[ESP_RX_BUF_SIZE];   /* buffer satu baris      */
volatile uint8_t  esp_line_idx = 0;            /* indeks buffer          */
volatile uint8_t  esp_line_ready = 0;          /* 1 = baris siap parse   */
volatile uint8_t  esp_collecting = 0;          /* 1 = sedang mengumpulkan*/

volatile float    esp_batt_percent = 0.0f;     /* SoC mentah (0..100)    */
volatile int32_t  esp_erpm = 0;                /* eRPM mentah            */
volatile uint32_t esp_rx_count = 0;            /* jumlah baris valid     */
volatile uint8_t  esp_connected = 0;           /* 1 jika ada data <1 s   */
volatile uint32_t esp_last_rx_ms = 0;          /* tick terakhir          */

/* Konversi eRPM -> km/jam (spesifikasi ATV) */
#define ESP_POLE_PAIRS     17.0f
#define ESP_GEAR_REDUCTION 6.0f                /* sproket 60/10          */
#define ESP_WHEEL_RADIUS_M 0.26f
#define ESP_WHEEL_CIRC_M   (2.0f * 3.14159265f * ESP_WHEEL_RADIUS_M)
/* =============================================================== */

volatile uint16_t rpm_raw_dbg = 0;
volatile int32_t rpm_motor_dbg = 0;
volatile uint32_t rpm_time_ms_dbg = 0;
volatile uint32_t rpm_time_us_dbg = 0;
volatile uint32_t rpm_log_tick = 0;
const uint32_t RPM_LOG_PERIOD_MS = 200;


/* Variabel Internal SIF */
//volatile uint8_t rawData[12];
//volatile uint8_t processData[12];


static volatile uint8_t rawData[SIF_BYTES];
static volatile uint8_t frameDataIsr[SIF_BYTES];
static uint8_t processData[SIF_BYTES];
volatile uint8_t debugData[SIF_BYTES];



volatile int bitIndex = -1;
//volatile uint32_t lastTime = 0;
static volatile uint32_t lastCycle = 0;
volatile uint32_t lastDuration = 0;
volatile uint8_t dataReady = 0;

volatile uint32_t debugPacketCounter = 0;
volatile uint32_t crcOkCounter = 0;
volatile uint32_t crcFailCounter = 0;
volatile uint8_t debugCrcCalc = 0;
volatile uint8_t debugCrcRx = 0;

volatile uint32_t latency_start_us = 0;
volatile uint32_t latency_end_us   = 0;
volatile uint32_t latency_us       = 0;

volatile uint32_t latency_min_us = 0xFFFFFFFF;
volatile uint32_t latency_max_us = 0;

volatile uint64_t latency_sum_us = 0;
volatile uint32_t latency_count  = 0;

volatile uint8_t uiNeedsUpdate = 0;

uint32_t lastPacketTime = 0;
const uint32_t PACKET_TIMEOUT = 2000;

//static volatile uint32_t lastCycle = 0;

const float GEAR_RATIO   = 4.77f;
const float WHEEL_CIRC_M = 2.0f * 3.14f * 0.45f;



/*GPS CONFIGURATION//-----------------------------------------------------------*/
static uint8_t gps_rx_byte;
static char gps_line[GPS_LINE_MAX];
static char gps_last_line[GPS_LINE_MAX];

static volatile uint16_t gps_idx = 0;
static volatile uint8_t gps_line_ready = 0;

volatile float gps_lat = 0.0f;
volatile float gps_lon = 0.0f;
volatile float gps_speed_kph = 0.0f;
volatile uint8_t gps_valid = 0;
volatile uint8_t gps_sats = 0;



/*SD CARD MODULE INTEGRATION*/
volatile char gps_date_str[16] = "0000-00-00";
volatile char gps_time_str[16] = "00:00:00";

static FATFS SDFatFS;
static FIL SDFile;
static volatile uint8_t sd_ready = 0;
static uint32_t gps_log_tick = 0;

#define LATENCY_SAMPLES 1000

uint32_t latencyBuffer[LATENCY_SAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */
static void SIF_ProcessIfReady(void);
void SIF_EXTI_FastHandler(void);
static inline void ITM_Print(const char *s);
static inline uint32_t micros_dwt(void);
static void GPS_ProcessLine(const char *line);

//SD CARD CONFIG
static uint8_t SD_MountAndPrepare(void);
static void GPS_LogToSD(void);
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
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */
  Displ_Init(Displ_Orientat_90);		// initialize the display and set the initial display orientation (here is orientaton: 0°) - THIS FUNCTION MUST PRECEED ANY OTHER DISPLAY FUNCTION CALL.
  touchgfxSignalVSync();			// asks TouchGFX to start initial display drawing
  Displ_BackLight('I');  			// initialize backlight and turn it on at init level

//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//  DWT->CYCCNT = 0;
//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  lastCycle = DWT->CYCCNT;
  lastPacketTime = HAL_GetTick();

  ITM_Print("SWV START OK\r\n");
  ITM_Print("Sample,Latency_us\r\n");
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
  ITM_Print("GPS UART START\r\n");

  JK_BMS_Init(&huart3);
  ITM_Print("JK BMS UART START\r\n");

  sd_ready = SD_MountAndPrepare();


//  touchgfx_v_sync();
//
//  HAL_TIM_Base_Start_IT(&htim3);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	      // Cek apakah data dari SIF Interrupt sudah lengkap 12 byte
//	      if (dataReady) {
//	          __disable_irq(); // Kunci interrupt sebentar untuk copy data
//	          memcpy((void*)processData, (const void*)rawData, 12);
//	          dataReady = 0;
//	          __enable_irq();
//
//	          uint8_t crc = 0;
//	          for (int i = 0; i < 11; i++) crc ^= processData[i];
//
//	          // Validasi CRC
//	          if (crc == processData[11]) {
//	              lastPacketTime = HAL_GetTick();
//
//	              faultCode = (processData[3] != 0);
//	              fardriver_mode = (processData[4] & 0x07);
//	              reverse = (processData[5] & 0x04) != 0;
//
//	              // Hitung Kecepatan dari RPM
//	              int32_t rpm_motor = ((uint16_t)processData[7] << 8) | processData[8];
//	              rpm_motor = (int32_t)(rpm_motor * 4.5f);
//
//	              float rpm_wheel = (float)rpm_motor / GEAR_RATIO;
//	              float v = (rpm_wheel * WHEEL_CIRC_M * 60.0f) / 1000.0f;
//
//	              if (v < 0) v = 0;
//	              velocity = (int32_t)(v + 0.5f);
//	          }
//	      }
//
//	      if (Touch_GotATouch(2))
//	      	    	touchgfxSignalVSync();		// asks TouchGFX to handle touch and redraw screen
//	      if ("other events needing a display update")
//	      	    	touchgfxSignalVSync();		// asks TouchGFX to get events and redraw screen*/



	      SIF_ProcessIfReady();


	      JK_BMS_Task();

	      /* === Terima & olah data ESP32 (VESC) === */
	      	      ESP32_ParseLine();
	      	      if ((HAL_GetTick() - esp_last_rx_ms) > 1000U) { esp_connected = 0U; }


	      //read another pin
	      uint8_t newSeinL  = (HAL_GPIO_ReadPin(SEINL_GPIO_Port, SEINL_Pin) == GPIO_PIN_SET);
	      uint8_t newSeinR  = (HAL_GPIO_ReadPin(SEINR_GPIO_Port, SEINR_Pin) == GPIO_PIN_SET);
	      uint8_t newBeam   = (HAL_GPIO_ReadPin(BEAM_GPIO_Port,  BEAM_Pin)  == GPIO_PIN_SET);
	      uint8_t newHazard = (newSeinL && newSeinR);

	      /* update hanya kalau berubah */
	      	      if ((newSeinL != seinL_val) ||
	      	          (newSeinR != seinR_val) ||
	      	          (newBeam  != beam_val)  ||
	      	          (newHazard != hazard_val))
	      	      {
	      	          seinL_val  = newSeinL;
	      	          seinR_val  = newSeinR;
	      	          beam_val   = newBeam;
	      	          hazard_val = newHazard;

	      	          uiNeedsUpdate = 1U;
	      	      }

	      if ((HAL_GetTick() - lastPacketTime) > PACKET_TIMEOUT)
	      {
	    	  velocity = 0;
	    	  rpm_raw_dbg = 0;
	    	  rpm_motor_dbg = 0;
	      }

//	      if (Touch_GotATouch(2))
//	      {
//	          touchgfxSignalVSync();
//	      }
//
//	      if (uiNeedsUpdate)
//	          {
//	              uiNeedsUpdate = 0U;
//	              touchgfxSignalVSync();
//	          }

	      MX_TouchGFX_Process();



	      if (gps_line_ready)
	      {
	          gps_line_ready = 0;
	          GPS_ProcessLine(gps_last_line);

	          char logbuf[160];
	          snprintf(logbuf, sizeof(logbuf),
	                   "GPS valid=%d date=%s time=%s lat=%.6f lon=%.6f speed=%.2f kph sats=%d\r\n",
	                   (int)gps_valid,
	                   gps_date_str,
	                   gps_time_str,
	                   (double)gps_lat,
	                   (double)gps_lon,
	                   (double)gps_speed_kph,
	                   (int)gps_sats);
	          ITM_Print(logbuf);

	      }

	      GPS_LogToSD();

    /* USER CODE END WHILE */

//  MX_TouchGFX_Process();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 166;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISPL_DC_Pin|DISPL_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPL_CS_Pin|TOUCH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SD_GPIO_Port, CS_SD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPL_DC_Pin DISPL_RST_Pin */
  GPIO_InitStruct.Pin = DISPL_DC_Pin|DISPL_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPL_CS_Pin TOUCH_CS_Pin CS_SD_Pin */
  GPIO_InitStruct.Pin = DISPL_CS_Pin|TOUCH_CS_Pin|CS_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BEAM_Pin */
  GPIO_InitStruct.Pin = BEAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BEAM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEINL_Pin SEINR_Pin */
  GPIO_InitStruct.Pin = SEINL_Pin|SEINR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_INT_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//uint32_t micros(void) {
//    return (DWT->CYCCNT / (SystemCoreClock / 1000000U));
//}
//
//// Callback saat Pin SIF mendeteksi Falling Edge (Sinyal Low)
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if (GPIO_Pin == GPIO_PIN_5) {
//	        int val = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5); // Cek status pin saat ini
//	        uint32_t now = micros();
//	        uint32_t duration = now - lastTime;
//	        lastTime = now;
//
////        if (lastDuration > 0) {
////            uint8_t bitComplete = 0;
//
//	        if (val == 0) {
//	                    if (lastDuration > 0) {
//	                        uint8_t bitComplete = 0;
//
//	                        // 1. Deteksi Header (Sesuai logika Arduino kamu)
//	                        if (lastDuration > (duration * 30)) {
//	                            bitIndex = 0;
//	                            memset((void*)rawData, 0, 12);
//	                        }
//	                        // 2. Deteksi Bit 0
//	                        else if ((lastDuration * 10) > (duration * 15)) {
//	                            bitComplete = 1;
//	                        }
//	                        // 3. Deteksi Bit 1
//	                        else if ((duration * 10) > (lastDuration * 15)) {
//	                            if (bitIndex >= 0 && bitIndex < 96) {
//	                                rawData[bitIndex >> 3] |= (1 << (7 - (bitIndex & 7)));
//	                            }
//	                            bitComplete = 1;
//	                        }
//
//	                        if (bitComplete) {
//	                            bitIndex++;
//	                            if (bitIndex >= 96) {
//	                                dataReady = 1;
//	                                bitIndex = -1;
//	                            }
//	                        }
//	                    }
//	                }
//	                lastDuration = duration; // Simpan durasi pulsa terakhir
//	            }
//	        }


static inline uint8_t SIF_PinIsLow(void)
{
    return ((GPIOD->IDR & GPIO_PIN_5) == 0U) ? 1U : 0U;
}

void SIF_EXTI_FastHandler(void)
{
    /* using count; no microsecond
       decoding based on duration */
    uint32_t now = DWT->CYCCNT;
    uint32_t duration = now - lastCycle;
    lastCycle = now;

    if (SIF_PinIsLow())
    {
        if (lastDuration != 0U)
        {
            uint8_t bitComplete = 0U;

            /* header */
            if (lastDuration > (duration * 30U))
            {

                bitIndex = 0;
                memset((void*)rawData, 0, SIF_BYTES);
            }
            /* bit 0 */
            else if ((lastDuration * 10U) > (duration * 15U))
            {
                bitComplete = 1U;
            }
            /* bit 1 */
            else if ((duration * 10U) > (lastDuration * 15U))
            {
                if ((bitIndex >= 0) && (bitIndex < 96))
                {
                    rawData[bitIndex >> 3] |= (uint8_t)(1U << (7U - (bitIndex & 7U)));
                }
                bitComplete = 1U;
            }

            if (bitComplete)
            {
                bitIndex++;
                if (bitIndex >= 96)
                {
                    for (uint32_t i = 0; i < SIF_BYTES; i++)
                    {
                        frameDataIsr[i] = rawData[i];
                    }
                    dataReady = 1U;


                    latency_start_us = micros_dwt();			//PENGUJIAN AWAL DATA LATENCY


                    bitIndex = -1;
                }
            }
        }
    }

    lastDuration = duration;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5)
    {
        SIF_EXTI_FastHandler();
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (gps_rx_byte == '\n')
        {
            gps_line[gps_idx] = '\0';
            strcpy(gps_last_line, gps_line);
            gps_idx = 0;
            gps_line_ready = 1;
        }
        else
        {
            if (gps_idx < (GPS_LINE_MAX - 1))
            {
                gps_line[gps_idx++] = (char)gps_rx_byte;
            }
            else
            {
                gps_idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
    }
    else if (huart->Instance == USART3)
    {
        JK_BMS_UART_RxCpltCallback(huart);
    }
    else if (huart->Instance == USART1)
    {
        char ch = (char)esp_rx_byte;

        if (ch == '$')
        {
            esp_collecting = 1;
            esp_line_idx = 0;
        }
        else if (esp_collecting)
        {
            if (ch == '\n' || ch == '\r')
            {
                esp_line[esp_line_idx] = '\0';
                esp_line_ready = 1;
                esp_collecting = 0;
            }
            else if (esp_line_idx < (ESP_RX_BUF_SIZE - 1))
            {
                esp_line[esp_line_idx++] = ch;
            }
            else
            {
                esp_collecting = 0;
                esp_line_idx = 0;
            }
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&esp_rx_byte, 1);
    }
}

static void GPS_ProcessLine(const char *line)
{
    switch (minmea_sentence_id(line, false))
    {
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line))
            {
                gps_valid = frame.valid ? 1 : 0;
                gps_lat = minmea_tocoord(&frame.latitude);
                gps_lon = minmea_tocoord(&frame.longitude);
                gps_speed_kph = minmea_tofloat(&frame.speed) * 1.852f;

                snprintf((char *)gps_date_str, sizeof(gps_date_str),
                         "20%02d-%02d-%02d",
                         frame.date.year,
                         frame.date.month,
                         frame.date.day);

                snprintf((char *)gps_time_str, sizeof(gps_time_str),
                         "%02d:%02d:%02d",
                         frame.time.hours,
                         frame.time.minutes,
                         frame.time.seconds);
            }
            break;
        }

        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line))
            {
                gps_sats = (uint8_t)frame.satellites_tracked;
            }
            break;
        }

        default:
            break;
    }
}



static uint8_t SD_MountAndPrepare(void)
{
    FRESULT fr;
    UINT bw;

//    MX_FATFS_Init();

    fr = f_mount(&SDFatFS, "", 1);
    if (fr != FR_OK)
    {
        ITM_Print("SD mount failed\r\n");
        return 0;
    }

    fr = f_open(&SDFile, "gps_log.csv", FA_OPEN_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        ITM_Print("SD open failed\r\n");
        return 0;
    }

    if (f_size(&SDFile) == 0)
    {
        const char *header = "date,time,latitude,longitude,sats\r\n";
        fr = f_write(&SDFile, header, strlen(header), &bw);
        if (fr != FR_OK)
        {
            f_close(&SDFile);
            ITM_Print("SD header write failed\r\n");
            return 0;
        }
    }

    f_close(&SDFile);
    ITM_Print("SD ready\r\n");
    return 1;
}

static void GPS_LogToSD(void)
{
    if (!sd_ready) return;
    if (!gps_valid) return;

    if ((HAL_GetTick() - gps_log_tick) < 1000U) return;
    gps_log_tick = HAL_GetTick();

    char line[128];
    UINT bw;
    FRESULT fr;

    int len = snprintf(line, sizeof(line),
                       "%s,%s,%.6f,%.6f,%d\r\n",
                       gps_date_str,
                       gps_time_str,
                       (double)gps_lat,
                       (double)gps_lon,
                       (int)gps_sats);

    fr = f_open(&SDFile, "gps_log.csv", FA_OPEN_APPEND | FA_WRITE);
    if (fr == FR_OK)
    {
        fr = f_write(&SDFile, line, len, &bw);
        f_close(&SDFile);

        if (fr == FR_OK)
        {
            ITM_Print("GPS log saved\r\n");
        }
        else
        {
            ITM_Print("GPS log write failed\r\n");
        }
    }
    else
    {
        ITM_Print("GPS log open failed\r\n");
    }
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//    if (htim->Instance == TIM6)
//    {
//        touchgfxSignalVSync();
//    }
//}


static inline uint32_t micros_dwt(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static inline void ITM_Print(const char *s)
{
    while (*s)
    {
        ITM_SendChar((uint32_t)*s++);
    }
}

static void SIF_ProcessIfReady(void)
{
    if (!dataReady) return;

    __disable_irq();
    memcpy(processData, (const void*)frameDataIsr, SIF_BYTES);
    dataReady = 0U;
    __enable_irq();

    uint8_t crc = 0U;
    for (uint32_t i = 0; i < 11U; i++)
    {
        crc ^= processData[i];
    }

    debugCrcCalc = crc;
    debugCrcRx   = processData[11];

    if (crc != processData[11])
    {
        crcFailCounter++;
        return;
    }

    lastPacketTime = HAL_GetTick();

    uint16_t rpm_raw = ((uint16_t)processData[7] << 8) | processData[8];
    int32_t  rpm_motor = (int32_t)(rpm_raw * 4.5f);

    if (rpm_raw == 0x5555U || rpm_raw == 0xAAAAU || rpm_raw == 0xFFFFU)
    {
        rpm_raw_dbg = 0;
        rpm_motor_dbg = 0;
        return;
    }

    if (rpm_motor > 30000)
    {
        rpm_raw_dbg = 0;
        rpm_motor_dbg = 0;
        return;
    }

    crcOkCounter++;
    memcpy((void*)debugData, processData, SIF_BYTES);
    debugPacketCounter++;


    faultCode      = (processData[3] != 0U);
    fardriver_mode = (processData[4] & 0x07U);
    reverse        = ((processData[5] & 0x04U) != 0U);

    rpm_time_ms_dbg = HAL_GetTick();
    rpm_time_us_dbg = micros_dwt();
    rpm_raw_dbg     = rpm_raw;
    rpm_motor_dbg   = rpm_motor;

    if ((rpm_time_ms_dbg - rpm_log_tick) >= RPM_LOG_PERIOD_MS)
    {
        rpm_log_tick = rpm_time_ms_dbg;

        char logbuf[96];
        snprintf(logbuf, sizeof(logbuf),
                 "%lu ms | %lu us | raw=%u | rpm=%ld\r\n",
                 rpm_time_ms_dbg,
                 rpm_time_us_dbg,
                 rpm_raw_dbg,
                 rpm_motor_dbg);
        ITM_Print(logbuf);
    }

    float rpm_wheel = (float)rpm_motor / GEAR_RATIO;
    float v = (rpm_wheel * WHEEL_CIRC_M * 60.0f) / 1000.0f;

    if (v < 0.0f) v = 0.0f;
    velocity = (int32_t)(v + 0.5f);

    uiNeedsUpdate = 1U;


    latency_end_us = micros_dwt();
    latency_us = latency_end_us - latency_start_us;

    if(latency_us < latency_min_us)
    {
        latency_min_us = latency_us;
    }

    if(latency_us > latency_max_us)
    {
        latency_max_us = latency_us;
    }

    if(latency_count < LATENCY_SAMPLES)
    {
        latencyBuffer[latency_count] = latency_us;

        latency_sum_us += latency_us;
        latency_count++;

        char buf[64];
           snprintf(buf,sizeof(buf),
                    "count=%lu lat=%lu us\r\n",
                    latency_count,
                    latency_us);
           ITM_Print(buf);

        if(latency_count == LATENCY_SAMPLES)
        {
            char buf[128];

            snprintf(buf, sizeof(buf),
                     "AVG=%lu us MIN=%lu us MAX=%lu us\r\n",
                     (uint32_t)(latency_sum_us / LATENCY_SAMPLES),
                     latency_min_us,
                     latency_max_us);

            ITM_Print(buf);
        }
    }
}



/* ===================================================================
 *  ESP32 / USART1 : init, MSP (pin), IRQ handler, dan parser
 *  Ditempatkan di sini supaya TIDAK perlu regenerate CubeMX.
 *  Pin: PA9 = USART1_TX (opsional), PA10 = USART1_RX (dari ESP32).
 * =================================================================== */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  /* MSP: clock + pin PA9/PA10 + NVIC (manual, tanpa CubeMX) */
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;     /* TX=PA9, RX=PA10 */
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/* Parser baris "<persen>,<erpm>" -> simpan ke variabel + (opsional) ke layar */
void ESP32_ParseLine(void)
{
    if (!esp_line_ready) return;
    esp_line_ready = 0;

    char buf[ESP_RX_BUF_SIZE];
    uint8_t i;
    for (i = 0; i < ESP_RX_BUF_SIZE; i++)
    {
        buf[i] = esp_line[i];
        if (esp_line[i] == '\0') break;
    }
    buf[ESP_RX_BUF_SIZE - 1] = '\0';

    char *tok1 = strtok(buf, ",");
    char *tok2 = strtok(NULL, ",");
    if (tok1 == NULL || tok2 == NULL) return;

    float   batt = strtof(tok1, NULL);
    int32_t er   = (int32_t)strtol(tok2, NULL, 10);

    if (batt < 0.0f)   batt = 0.0f;
    if (batt > 100.0f) batt = 100.0f;

    esp_batt_percent = batt;
    esp_erpm = er;
    esp_rx_count++;
    esp_last_rx_ms = HAL_GetTick();
    esp_connected = 1;

    /* Konversi eRPM -> km/jam (pakai nilai mutlak agar tak negatif) */
    float erpm_abs = (er < 0) ? (float)(-er) : (float)er;
    float motor_rpm = erpm_abs / ESP_POLE_PAIRS;
    float wheel_rpm = motor_rpm / ESP_GEAR_REDUCTION;
    float v_kmh = (wheel_rpm * ESP_WHEEL_CIRC_M * 60.0f) / 1000.0f;

    /* ====== INTEGRASI KE TOUCHGFX ======
     * TAHAP 1 (cek Live Expression): biarkan dua baris di bawah TER-COMMENT,
     *         amati esp_batt_percent, esp_erpm, esp_rx_count, esp_connected.
     * TAHAP 2 (tampil ke layar): hapus komentar dua baris di bawah.
     */
    velocity     = (int32_t)(v_kmh + 0.5f);
    extern volatile uint8_t jk_bms_soc;   /* dari bms_jk.c */
    jk_bms_soc   = (uint8_t)(batt + 0.5f);

    (void)v_kmh; /* cegah warning jika dua baris di atas dikomentari */
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
