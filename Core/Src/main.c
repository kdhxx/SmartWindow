/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Simplified for Always-On)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lora_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ====================================================
// [핀 이름 정의] 하드웨어 연결에 따른 이름표
// ====================================================

// 1. 창문 모터 (Window Motor)
#define WINDOW_ACTION_PIN       GPIO_PIN_2   // PB2
#define WINDOW_ACTION_GPIO_Port GPIOB

#define WINDOW_DIR_PIN          GPIO_PIN_15  // PB15
#define WINDOW_DIR_GPIO_Port    GPIOB

// 2. 블라인드 모터 (Blind Motor)
#define BLIND_ACTION_PIN        GPIO_PIN_14  // PB14
#define BLIND_ACTION_GPIO_Port  GPIOB

#define BLIND_DIR_PIN           GPIO_PIN_13  // PB13
#define BLIND_DIR_GPIO_Port     GPIOB

// 3. 센서 및 기타 핀 (기존 유지)
#define RAIN_SENSOR_PIN         GPIO_PIN_10 // PC10
#define RAIN_SENSOR_GPIO_Port   GPIOC

#define DUST_SENSOR_PIN         GPIO_PIN_6  // PC6
#define DUST_SENSOR_GPIO_Port   GPIOC

#define DHT11_PIN               GPIO_PIN_12  // PC12
#define DHT11_PORT              GPIOC

// 3. 로직 설정
#define TEMP_HOT_TO_OPEN    28
#define TEMP_COLD_TO_CLOSE  22
#define HUMI_TO_OPEN        60
#define HUMI_TO_CLOSE       40
#define LIGHT_THRESHOLD     4000
#define DUST_BAD_LEVEL      80
#define LIGHT_BRIGHT_TH 3000
#define LIGHT_DARK_TH   1000

#define SENSOR_CHECK_INTERVAL 3000
#define MAX_OPEN_TIME 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int auto_mode = 0;

int win_moving = 0;
int blind_moving = 0;
int win_dir = 1;
int blind_dir = 0;

volatile uint8_t g_rain_flag = 0;

// --- IR 리모컨 변수 ---
typedef enum {
  IR_STATE_IDLE, IR_STATE_AGC_SPACE, IR_STATE_DATA, IR_STATE_REPEAT_PULSE
} IR_DecoderState_t;

static volatile IR_DecoderState_t ir_state = IR_STATE_IDLE;
static volatile uint32_t last_capture = 0;
static volatile uint8_t  ir_bit_count = 0;
static volatile uint32_t ir_data_raw = 0;
volatile uint8_t  ir_command = 0;
volatile uint32_t ir_full_data = 0;

// [NEW] 미세먼지 센서 변수 (TIM4 사용)
volatile uint32_t dust_start_time = 0;  // 펄스 시작 시간
volatile uint32_t dust_pulse_width = 0; // 펄스 길이 (Low 구간)
volatile uint8_t  dust_ready_flag = 0;  // 계산 완료 깃발

// --- 센서 데이터 변수 ---
/*uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
float Temperature = 0.0f;
float Humidity = 0.0f;*/
int Val_Temp = 0;
int Val_Humi = 0;
int Val_Dust = 0;
int light_value = 0;

// 현재 창문 위치 (0: 닫힘 ~ 5000: 완전 열림)
// [주의] 보드를 처음 켤 때는 창문을 꽉 닫아놓고 켜야 합니다! (0에서 시작하니까)
int32_t g_current_pos = 0;

// 수동 조작 시작 시간 기억용
uint32_t g_manual_start_tick = 0;

int g_dir = 0; // 0: 정지, 1: 열림(Open) 방향, -1: 닫힘(Close) 방향
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Window_Action_Open(void);
void Window_Action_Close(void);
void Window_Action_Stop(void);

void Handle_Periodic_Sensor_Task(void);

void delay_us(uint16_t us);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void DHT11_Start(void);
int8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);
void Blind_Action_Up(void);
void Blind_Action_Down(void);
void Blind_Action_Stop(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
  return len;
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  printf("Smart Window.\r\n");

  Window_Action_Stop();

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // IR
  HAL_TIM_Base_Start(&htim6);                 // Delay
  HAL_TIM_Base_Start_IT(&htim3);              // LoRa
  HAL_TIM_Base_Start(&htim4);                 // 미세먼지 타이머 켜기!

  // 3. [추가] LoRa 초기화
  LoRa_Init_User();

  uint32_t last_sensor_tick = 0;
  uint32_t current_tick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  current_tick = HAL_GetTick();
	  // 1. 주기적 센서 체크 (5초마다) -> 여기에 모든 기능이 다 들어감!
	  if (current_tick - last_sensor_tick >= SENSOR_CHECK_INTERVAL)
	  {
		  last_sensor_tick = current_tick;

	  // 이 함수 하나로 측정, 계산, 전송, 제어
	      Handle_Periodic_Sensor_Task();
	  }

    /* 2. IR 리모컨 처리 */
    if (ir_command != 0)
    {
        printf("[IR] Command: 0x%02X\n", ir_command);
        switch (ir_command)
        {
            case 0x0C: //1
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Window_Action_Open();
            	win_moving = 1;
                printf("Open Window\n");
                break;
            case 0x18: //2
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Window_Action_Stop();
            	win_moving = 0;
                printf("Stop Window\n");
                break;
            case 0x5E: //3
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Window_Action_Close();
            	win_moving = 1;
            	printf("Close Window\n");
            	break;
            case 0x08: //4
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Blind_Action_Up();
            	blind_moving = 1;
            	printf("Up Blind\n");
            	break;
            case 0x1C: //5
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Blind_Action_Stop();
            	blind_moving = 0;
            	printf("Stop Blind\n");
            	break;
            case 0x5A: //6
            	if(auto_mode == 1)
            	{
            		printf("Ignored: Currently Auto Mode\n");
            		break;
            	}
            	Blind_Action_Down();
            	blind_moving = 1;
            	printf("Down Blind\n");
            	break;
            case 0x42: //7
            	auto_mode = 0;
            	printf(">> Auto Mode OFF\n");
            	break;
            case 0x52: //8
            	auto_mode = 1;
            	printf(">> Auto Mode ON\n");
            	break;
            default:
                printf("Unknown Command\n");
                break;
        }
        ir_command = 0;
    }

    /* 3. 루프 딜레이 */
    HAL_Delay(100); // 딜레이를 너무 길게(100ms) 주면 LoRa 수신 반응이 느려질 수 있음. 10ms 추천.

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 49;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 49;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 49;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 49;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RADIO_ANT_SWITCH_Pin|LED_2_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_RESET_GPIO_Port, RADIO_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WINDOW_ACTION_Pin|BLIND_ACTION_Pin|WINDOW_DIR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BLIND_DIR_Pin|RADIO_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_ANT_SWITCH_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_ANT_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_RESET_Pin LED_2_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = RADIO_RESET_Pin|LED_2_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : WINDOW_ACTION_Pin BLIND_DIR_Pin BLIND_ACTION_Pin WINDOW_DIR_Pin
                           RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = WINDOW_ACTION_Pin|BLIND_DIR_Pin|BLIND_ACTION_Pin|WINDOW_DIR_Pin
                          |RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DUST_SENSOR_Pin RAIN_SENSOR_Pin */
  GPIO_InitStruct.Pin = DUST_SENSOR_Pin|RAIN_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_0_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_1_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RADIO_DIO_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- 모터 제어 함수 ---
void Window_Action_Open(void) {
	printf("win_open\r\n");
    HAL_GPIO_WritePin(WINDOW_DIR_GPIO_Port, WINDOW_DIR_Pin, GPIO_PIN_RESET);//0 ->
    HAL_GPIO_WritePin(WINDOW_ACTION_GPIO_Port, WINDOW_ACTION_Pin, GPIO_PIN_RESET);
    win_dir = 0;
}
void Window_Action_Close(void) {

    HAL_GPIO_WritePin(WINDOW_DIR_GPIO_Port, WINDOW_DIR_Pin, GPIO_PIN_SET); //1 <-
    HAL_GPIO_WritePin(WINDOW_ACTION_GPIO_Port, WINDOW_ACTION_Pin, GPIO_PIN_RESET); // 0 동작
    win_dir = 1;
}
void Window_Action_Stop(void) {

    HAL_GPIO_WritePin(WINDOW_ACTION_GPIO_Port, WINDOW_ACTION_Pin,GPIO_PIN_SET); // 1 멈춤
}

void Blind_Action_Up(void) {

	HAL_GPIO_WritePin(BLIND_DIR_GPIO_Port, BLIND_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BLIND_ACTION_GPIO_Port, BLIND_ACTION_Pin, GPIO_PIN_RESET);
	blind_dir = 0;
}

void Blind_Action_Down(void) {

	HAL_GPIO_WritePin(BLIND_DIR_GPIO_Port, BLIND_DIR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLIND_ACTION_GPIO_Port, BLIND_ACTION_Pin, GPIO_PIN_RESET);
	blind_dir = 1;
}

void Blind_Action_Stop(void) {

	HAL_GPIO_WritePin(BLIND_ACTION_GPIO_Port, BLIND_ACTION_Pin, GPIO_PIN_SET);
}

// --- DHT11 및 유틸리티 ---
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < us);
}
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start(void) {
    Set_Pin_Output(DHT11_PORT, DHT11_PIN);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    delay_us(18000);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(20);
    Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

int8_t DHT11_Check_Response(void) {
    int8_t response = 0;
    delay_us(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        delay_us(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) response = 1;
        else response = -1;
    }
    uint32_t timeout = 0;
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        timeout++;
        if(timeout > 10000) return -1;
    }
    return response;
}

uint8_t DHT11_Read(void) {
    uint8_t i = 0, j;
    for (j = 0; j < 8; j++) {
        uint32_t timeout = 0;
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
             if(timeout++ > 10000) return 0;
        }
        delay_us(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
            i &= ~(1 << (7 - j));
        }
        else {
            i |= (1 << (7 - j));
            timeout = 0;
            while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
                 if(timeout++ > 10000) return 0;
            }
        }
    }
    return i;
}

// [NEW] 미세먼지 펄스 길이를 직접 재는 함수 (인터럽트 X)
// 핀이 Low인 시간을 마이크로초(us) 단위로 반환
uint32_t Measure_Dust_Pulse_Blocking(void) {
    uint32_t timeout = 0;

    // 1. 현재 신호가 Low라면 High가 될 때까지 기다림 (측정 시작점 맞추기)
    while (HAL_GPIO_ReadPin(DUST_SENSOR_GPIO_Port, DUST_SENSOR_Pin) == GPIO_PIN_RESET) {
        if (timeout++ > 100000) return 0; // 너무 오래 걸리면 탈출 (에러 방지)
    }

    // 2. 신호가 Low로 떨어질 때까지 대기 (펄스 시작!)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DUST_SENSOR_GPIO_Port, DUST_SENSOR_Pin) == GPIO_PIN_SET) {
        if (timeout++ > 100000) return 0;
    }

    // 3. Low 구간 시간 측정 시작 (타이머4 사용)
    __HAL_TIM_SET_COUNTER(&htim4, 0); // 타이머 0으로 초기화

    // 4. 신호가 다시 High로 올라갈 때까지 대기 (펄스 끝!)
    while (HAL_GPIO_ReadPin(DUST_SENSOR_GPIO_Port, DUST_SENSOR_Pin) == GPIO_PIN_RESET) {
        // 기다리는 중...
        if (__HAL_TIM_GET_COUNTER(&htim4) > 60000) break; // 너무 길면 탈출
    }

    // 5. 측정된 시간 반환
    return __HAL_TIM_GET_COUNTER(&htim4);
}

// --- 태스크 처리 함수 ---

void Handle_Periodic_Sensor_Task(void)
{

    // ==========================================================
    // 1. DHT11 온습도 읽기 (체크섬 확인 포함)
    // ==========================================================
    DHT11_Start();
    if (DHT11_Check_Response() == 1) {
        uint8_t rh_i = DHT11_Read(); // 습도 정수
        uint8_t rh_d = DHT11_Read(); // 습도 소수 (무시)
        uint8_t t_i  = DHT11_Read(); // 온도 정수
        uint8_t t_d  = DHT11_Read(); // 온도 소수 (무시)
        uint8_t sum  = DHT11_Read();

        // 데이터 무결성 검사
        if (sum == (rh_i + rh_d + t_i + t_d)) {
            Val_Humi = rh_i; // 전역 변수 업데이트
            Val_Temp = t_i;  // 전역 변수 업데이트
        } else {
             printf("  [Error] DHT11 Checksum Fail\r\n");
        }
    } else {
        printf("  [Error] DHT11 Not Responding\r\n");
    }

    // ==========================================================
    // 2. 미세먼지 측정
    // ==========================================================

    uint32_t measured_pulse = 0;

    if (HAL_GPIO_ReadPin(DUST_SENSOR_GPIO_Port, DUST_SENSOR_Pin) == GPIO_PIN_RESET)
    {
         __HAL_TIM_SET_COUNTER(&htim4, 0); // 스톱워치 0으로 리셋
         // 다시 High가 될 때까지 대기 (먼지 측정 중...)
         while (HAL_GPIO_ReadPin(DUST_SENSOR_GPIO_Port, DUST_SENSOR_Pin) == GPIO_PIN_RESET)
         {
             // 안전장치: 너무 오래 걸리면(약 65ms 이상) 탈출
             if (__HAL_TIM_GET_COUNTER(&htim4) > 60000)
             {
                 break;
             }
         }
         measured_pulse = __HAL_TIM_GET_COUNTER(&htim4);
    }
    else
    {
        // 핀이 High 상태라면 지금은 먼지가 감지되지 않는 구간임 -> 0 처리
        measured_pulse = 0;
    }

    // (C) 값 변환 (이 부분이 함수 밖으로 쫓겨나 있었음! 이제 안으로 들어옴)
    if (measured_pulse > 1400)
    {
        Val_Dust = (measured_pulse - 1400) / 14;
    }
    else
    {
        Val_Dust = 0;
    }

    // 조도 센서

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) light_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);


    // 3. 통합 데이터 출력 (정수만 출력)
    printf("  >> T:%d, H:%d | Dust: %d ug/m3 Light: %d \r\n", Val_Temp, Val_Humi, Val_Dust, light_value);

    // 4. LoRa 전송 (정수 데이터만 전송)
    // lora_app.c 함수가 인자 7개를 요구하므로, 소수점 자리에 0을 채워 보냅니다.
    int rain_st = (HAL_GPIO_ReadPin(RAIN_SENSOR_GPIO_Port, RAIN_SENSOR_PIN) == GPIO_PIN_RESET) ? 1 : 0;

    // [Temp, 0, Humi, 0, Dust, 0, Rain] 순서

    LoRa_Send_SensorData(Val_Temp, Val_Humi, Val_Dust, rain_st, light_value, win_dir, blind_dir);

    // ==========================================================
    // 5. 자동 제어 로직 (요청하신 우선순위 적용)
    // 우선순위: 미세먼지(건강) > 온도(쾌적) > 습도
    // ==========================================================

    if (auto_mode == 1)
    {
//    	if (win_moving == 0) {

    		// (0) 비오면 닫기 (최우선) - rain_st 활용
    		if (rain_st == 1) {
                 printf("   => [Auto] Rain! Closing.\r\n");
    			Window_Action_Close();
    			win_moving = 1;
            }
            // (1) 미세먼지가 나쁘면 무조건 닫기
            else if (Val_Dust > DUST_BAD_LEVEL) {
                 printf("   => [Auto] Dust Bad! Closing.\r\n");
            	Window_Action_Close();
            	win_moving = 1;
            }
            // (2) 온도가 너무 낮으면 닫기 (추위)
            else if (Val_Temp < TEMP_COLD_TO_CLOSE) {
                printf("   => [Auto] Cold! Closing.\r\n");
            	Window_Action_Close();
            	win_moving = 1;
            }
            // (3) 온도가 너무 높으면 열기 (더위)
            else if (Val_Temp > TEMP_HOT_TO_OPEN) {
                printf("   => [Auto] Hot! Opening.\r\n");
            	Window_Action_Open();
            	win_moving = 1;
            }
            // (4) 온도/먼지가 괜찮으면 습도 체크
            else {
                if (Val_Humi > HUMI_TO_OPEN) {
                    printf("   => [Auto] Humid! Opening.\r\n");
                	Window_Action_Open();
                	win_moving = 1;
                } else if (Val_Humi < HUMI_TO_CLOSE) {
                    printf("   => [Auto] Dry! Closing.\r\n");
                	Window_Action_Close();
                	win_moving = 1;
                }
            }
    		if (light_value > LIGHT_BRIGHT_TH) {
    			printf("   => [Auto] Blind up.\r\n");
    			Blind_Action_Up();
    		}
    		else if (light_value < LIGHT_DARK_TH) {
    			printf("   => [Auto] Blind down.\r\n");
    			Blind_Action_Down();
    		}
    }
}

// --- 인터럽트 콜백 ---

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RADIO_DIO_0_Pin && DIO0_IrqHandler != NULL) {
    	DIO0_IrqHandler(DIO0_Context);
//    	SX1272OnDio0Irq(); // 전송/수신 완료 신호 처리
    }
    else if (GPIO_Pin == RADIO_DIO_1_Pin && DIO1_IrqHandler != NULL) {
    	DIO1_IrqHandler(DIO1_Context);
//    	SX1272OnDio1Irq(); // 타임아웃 신호 처리
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;
  uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  uint32_t pulse_width_us = (uint32_t)(current_capture - last_capture);
  last_capture = current_capture;

  if (pulse_width_us > 15000) { ir_state = IR_STATE_IDLE; return; }

  switch (ir_state) {
    case IR_STATE_IDLE:
      if (pulse_width_us > 10000 && pulse_width_us < 11500) ir_state = IR_STATE_AGC_SPACE;
      break;
    case IR_STATE_AGC_SPACE:
      if (pulse_width_us > 4000 && pulse_width_us < 5000) {
        ir_state = IR_STATE_DATA; ir_data_raw = 0; ir_bit_count = 0;
      } else if (pulse_width_us > 2000 && pulse_width_us < 2500) {
        ir_state = IR_STATE_REPEAT_PULSE;
      } else { ir_state = IR_STATE_IDLE; }
      break;
    case IR_STATE_REPEAT_PULSE: ir_state = IR_STATE_IDLE; break;
    case IR_STATE_DATA:
      if (ir_bit_count % 2 == 1) {
        uint8_t idx = ir_bit_count / 2;
        if (pulse_width_us > 1500) ir_data_raw |= (1UL << idx);
      }
      ir_bit_count++;
      if (ir_bit_count == 64) {
        ir_full_data = ir_data_raw;
        uint8_t addr = (ir_full_data >> 0) & 0xFF;
        uint8_t not_addr = (ir_full_data >> 8) & 0xFF;
        uint8_t cmd = (ir_full_data >> 16) & 0xFF;
        uint8_t not_cmd = (ir_full_data >> 24) & 0xFF;

        if (addr == (uint8_t)~not_addr && cmd == (uint8_t)~not_cmd) {
            ir_command = cmd;
        }
        ir_state = IR_STATE_IDLE;
      }
      break;
  }
}

// [LoRa] 타이머 인터럽트 핸들러 연결 (TIM3 사용)
// 이 함수가 없으면 LoRa 내부 시간이 안 가서 통신이 멈춤
extern void TimerIrqHandler(void); // 드라이버 함수 가져오기

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM3가 1ms마다 울릴 때만 실행
    if (htim->Instance == TIM3) {
        TimerIrqHandler();
    }
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
