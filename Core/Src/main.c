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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t LED_Volt_AD_Val;
	uint16_t BattVolt_AD_Val;
	uint16_t HeatTemp_AD_Val;
	uint16_t CPU_Temp_AD_Val;
	uint16_t AD_IntRef_AD_Val;
} ADC1_VAL_t;

typedef struct
{
	uint16_t LED_Curr_AD_Val;
} ADC2_VAL_t;

typedef struct
{
	int32_t target;
	int32_t now;
} PID_Ctrl_t;

typedef struct
{
	uint32_t StartPID : 1;	   // PID処理の開始
	uint32_t StartProcADC : 1; // 各種ADC取得データの処理開始
	uint32_t SW_now : 1;	   // 起動SWの現在値(100ms処理)
	uint32_t SW_befor : 1;	   // 起動SWの前回値(100ms処理)
	uint32_t SW_push : 1;	   // 起動SWが押された(ソフトクリア)
	uint32_t Failsafe : 1;	   // フェールセーフ状態
	uint32_t TIM_10ms : 1;
	uint32_t TIM_100ms : 1;
	uint32_t TIM_1s : 1;
	uint32_t OLED_Update : 1;
} FLGBITS_t;

typedef struct
{
	uint8_t LED_OverVolt : 1;
	uint8_t L_OverCurr : 1;
	uint8_t Bat_UnderVold : 1;
	uint8_t LED_OverHeat : 1;
	uint8_t CPU_OverHeat : 1;
} FAILSAFE_STATUS_BITS_t;

typedef union
{
	uint8_t BYTEs;
	FAILSAFE_STATUS_BITS_t BITs;
} FAILSAFE_STATUS_BYTE_t;

typedef struct
{
	FAILSAFE_STATUS_BYTE_t now;
	FAILSAFE_STATUS_BYTE_t befor;
	FAILSAFE_STATUS_BYTE_t record;
} FAILSAFE_STATUSES_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADC1_VAL_t ADC1_Value = {0};
ADC2_VAL_t ADC2_Value = {0};
FLGBITS_t FLGbits = {0};
FAILSAFE_STATUSES_t FailsafeStatuses = {0};

PID_Ctrl_t LED_Curr_PID = {0};
PIDController_t PID_CurrCtrl = {0};
PID_Ctrl_t LED_Power_PID = {0};
PIDController_t PID_LedPowerCtrl = {0};

const uint16_t PWM_FULL = 1440 - 1;
const uint16_t PWM_HARF = 720;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
static void mainProc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		// 1ms
		static uint16_t cnt_1s = 0;
		cnt_1s++;

		if (cnt_1s % 10 == 0)
		{
			// 10ms
			FLGbits.TIM_10ms = 1;
		}
		if (cnt_1s % 100 == 0)
		{
			// 100ms
			FLGbits.TIM_100ms = 1;
		}
		if (cnt_1s % 50 == 0)
		{
			// 200ms
			FLGbits.OLED_Update = 1;
		}

		if (cnt_1s >= 1000)
		{
			// 1s
			cnt_1s = 0;
			FLGbits.TIM_1s = 1;
			// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		FLGbits.StartProcADC = 1;
	}
	if (hadc == &hadc2)
	{
		ADC2_Value.LED_Curr_AD_Val = HAL_ADC_GetValue(&hadc2);
		FLGbits.StartPID = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LED_VoltComp_Pin)
	{
		FailsafeStatuses.now.BITs.LED_OverVolt = 1;
	}
	if (GPIO_Pin == L_CurrComp_Pin)
	{
		FailsafeStatuses.now.BITs.L_OverCurr = 1;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c == &hi2c2)
	{
		__NOP();
	}
}

static void mainProc(void)
{

	if (FLGbits.StartPID)
	{
		FLGbits.StartPID = 0;

		//  ADC=1.2V/A
		//  Vref=3.3V
		//  1.2V=1489d
		LED_Curr_PID.now = ADC2_Value.LED_Curr_AD_Val;

		fixedPoint32_t proc = PID_Proc(&PID_CurrCtrl, LED_Curr_PID.target, LED_Curr_PID.now);
		proc = fixed_point_mul(proc, PWM_FULL);
		proc = (proc <= 0) ? 0 : proc;
		const int32_t PID_Limit = 900;
		proc = (proc > PID_Limit) ? PID_Limit : proc;
		uint16_t pwm = (uint16_t)proc;

		if (FLGbits.Failsafe)
		{
			// LED_Curr_PID.target = 0;
			pwm = 0;
			FLGbits.Failsafe = 0;
		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM_HARF);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm / 2 + 1);
	}

	if (FLGbits.StartProcADC)
	{
		FLGbits.StartProcADC = 0;

		// ヒートシンク冷却ファンPWMデューティー演算
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1200);
	}

	if (FLGbits.SW_push)
	{
		FLGbits.SW_push = 0;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}

	if (FLGbits.TIM_10ms)
	{
		FLGbits.TIM_10ms = 0;

		// LED投入電力計算
		float led_curr = LED_Curr_PID.now;
		led_curr *= 33.0f / 32768.0f;
		float led_volt = ADC1_Value.LED_Volt_AD_Val;
		led_volt *= 11.0f / 1024.0f;
		LED_Power_PID.now = (int32_t)(led_curr * led_volt);

		// LED電力PID制御
		fixedPoint32_t proc = PID_Proc(&PID_LedPowerCtrl, LED_Power_PID.target, LED_Power_PID.now);
		proc = fixed_point_mul(proc, 4000);
		proc = (proc <= 0) ? 0 : proc;
		const int32_t PID_Limit = 3800;
		proc = (proc > PID_Limit) ? PID_Limit : proc;
		LED_Curr_PID.target = proc;

		// フェールセーフ状態か判定
		if (
			(FailsafeStatuses.now.BYTEs > 0) &&
			(FailsafeStatuses.now.BYTEs != FailsafeStatuses.befor.BYTEs))
		{
			FailsafeStatuses.record.BYTEs |= FailsafeStatuses.now.BYTEs;
			FailsafeStatuses.now.BYTEs = 0;
			FLGbits.Failsafe = 1;
		}
		FailsafeStatuses.befor.BYTEs = FailsafeStatuses.now.BYTEs;

		// OLED表示データ生成
		char str[20] = {0};
		intToString((int16_t)LED_Curr_PID.target, str);
		OLED_SetCursor(0, 1);
		OLED_WriteString(str);
		OLED_WriteString(" ");

		intToString((int16_t)LED_Curr_PID.now, str);
		OLED_SetCursor(30, 1);
		OLED_WriteString(str);
		OLED_WriteString("     ");

		intToString(ADC1_Value.LED_Volt_AD_Val, str);
		OLED_SetCursor(0, 2);
		OLED_WriteString(str);
		OLED_WriteString(" ");

		intToString(ADC1_Value.BattVolt_AD_Val, str);
		OLED_SetCursor(30, 2);
		OLED_WriteString(str);
		OLED_WriteString(" ");

		intToString(ADC1_Value.HeatTemp_AD_Val, str);
		OLED_SetCursor(60, 2);
		OLED_WriteString(str);
		OLED_WriteString(" ");

		intToString(LED_Power_PID.now, str);
		OLED_SetCursor(90, 2);
		OLED_WriteString(str);
		OLED_WriteString("    ");

		intToString(ADC1_Value.CPU_Temp_AD_Val, str);
		OLED_SetCursor(0, 3);
		OLED_WriteString(str);
		OLED_WriteString(" ");

		intToString(ADC1_Value.AD_IntRef_AD_Val, str);
		OLED_SetCursor(30, 3);
		OLED_WriteString(str);
		OLED_WriteString("    ");
	}

	if (FLGbits.TIM_100ms)
	{
		FLGbits.TIM_100ms = 0;

		FLGbits.SW_now = (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET) ? 1 : 0; // SW押されてたら1
		if (
			(FLGbits.SW_now == 1) &&
			(FLGbits.SW_now != FLGbits.SW_befor)) // SW押されたエッジを検出
		{
			FLGbits.SW_push = 1;
		}
		FLGbits.SW_befor = FLGbits.SW_now; // 今回値を前回値にする
	}

	if (FLGbits.TIM_1s)
	{
		FLGbits.TIM_1s = 0;
		if(LED_Power_PID.target>30)
		{
			LED_Power_PID.target=10;
		}else{
			LED_Power_PID.target=100;
		}
	}

	if (FLGbits.OLED_Update)
	{
		FLGbits.OLED_Update = 0;
		OLED_Update(&hi2c2);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	// TIM2動作開始＆割り込み有効
	HAL_TIM_Base_Start_IT(&htim2);
	// TIM3、TIM4のPWNとOC動作開始＆割り込み有効
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_4);
	// ADC1、ADC2動作開始＆割り込み有効
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&ADC1_Value, 5); // 5chの変換を行う 格納先変数アドレスはuint32で受けるため、uint16の構造体だがキャストする
	HAL_ADC_Start_IT(&hadc2);
	// OLED初期化
	OLED_Init(&hi2c2);
	OLED_Clear();
	// PID初期化
	PID_Init(
		&PID_CurrCtrl,
		fixed_point_val(0.5f),
		fixed_point_val(0.3f),
		fixed_point_val(0.05f),
		fixed_point_val(-1.0f),
		fixed_point_val(1.0f));
	PID_Init(
		&PID_LedPowerCtrl,
		fixed_point_val(30.0f),
		fixed_point_val(15.0f),
		fixed_point_val(1.0f),
		fixed_point_val(-1.0f),
		fixed_point_val(1.0f));

	// 負電源生成開始
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_HARF);
	// 昇圧コイル電流リミット値設定
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1309); // リミット8A
	// LED過電圧リミット値設定
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1309); // リミット40V

	// フェールセーフフラグクリア
	// ポート初期化でプルアップが無効になり、ポート初期設定後にフェールセーフモードに入っているため
	FLGbits.Failsafe = 0;

	// LED電流設定
	//LED_Curr_PID.target = 500;

	// LED電力設定
	LED_Power_PID.target=10;

	//電源の自己保持開始
	HAL_GPIO_WritePin(SelfPowerON_GPIO_Port,SelfPowerON_Pin,GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		mainProc();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
