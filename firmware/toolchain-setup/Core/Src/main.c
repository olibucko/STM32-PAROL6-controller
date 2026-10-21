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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc5160.h"
#include <stdio.h>
#include <math.h>                        // [CHANGE 1] sqrtf for the ramp

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* [CHANGE 2] Unit constants moved up from USER CODE 4 so everything below
 * (accel setter, ramp) can use them. */
#define STEPS_PER_REV     51200.0f                 // 200 full x 256 usteps
#define STEPS_PER_DEG     (STEPS_PER_REV / 360.0f)
#define STEPS_PER_RAD     (STEPS_PER_REV / 6.2831853f)

/* [CHANGE 3] Ramp/timer constants */
#define TIM_TICK_HZ       1000000.0f   // TIM2 tick: 90 MHz / (PSC 89 + 1)
#define STEP_PULSE_TICKS  5u           // STEP high time = 5 us (fixed width)
#define V_START_HZ        500.0f       // first & last step rate; must be > 0

#define RUN_MSCNT_TEST    1            // [CHANGE 9] set 0 once verified

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t val;

volatile int32_t  g_position_steps  = 0;  // absolute position, signed, in steps
volatile uint32_t g_steps_remaining = 0;  // steps left in current move (debug/info)
volatile int8_t   g_step_dir        = 1;  // +1 or -1, applied per step

/* [CHANGE 3] Ramp state */
volatile uint32_t g_steps_total = 0;      // |n| for current move
volatile uint32_t g_steps_done  = 0;      // pulses launched so far
volatile uint8_t  g_moving      = 0;      // 1 until the timer has actually stopped

static float g_vmax  = 0.0f;              // cruise rate, steps/s (per move)
static float g_two_a = 0.0f;              // 2 x accel, steps/s^2
static const float g_vstart2 = V_START_HZ * V_START_HZ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void motor_set_dir(uint8_t fwd);
/* [CHANGE 8] motor_set_step_hz / motor_set_rpm removed (free-running, uncounted) */

void set_accel_rpm_s(float rpm_per_s);

void move_steps(int32_t n, float rpm);
void move_to_deg(float deg, float rpm);
void move_to_rad(float rad, float rpm);
float get_angle_deg(void);
float get_angle_rad(void);

void move_wait(void);
uint8_t move_busy(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	static void dbg(const char *tag, uint32_t v)
	{
		char buf[48];
		int n = snprintf(buf, sizeof(buf), "%-10s 0x%08lX\r\n", tag, (unsigned long)v);
		HAL_UART_Transmit(&huart2, (uint8_t *)buf, n, HAL_MAX_DELAY);
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
  MX_SPI2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  tmc5160_write(TMC_GSTAT, 0x07);
  tmc5160_write(TMC_GCONF,        0x00000000);   // SpreadCycle
  tmc5160_write(TMC_GLOBALSCALER, 128);
  tmc5160_write(TMC_CHOPCONF,     0x000100C3);   // TOFF=3, MRES=256
  tmc5160_write(TMC_IHOLD_IRUN,   0x00081810);
  tmc5160_write(TMC_TPOWERDOWN,   0x0000000A);

  set_accel_rpm_s(120.0f);   // [CHANGE 7] 0 -> 30 rpm in 0.25 s (~22.5 deg of travel)

#if RUN_MSCNT_TEST
  /* [CHANGE 9] Step-count exactness check against the chip itself.
   * 12345 mod 1024 = 57. Expect d = 57 (or 967 if polarity counts down). */
  {
      uint32_t m0 = tmc5160_read(TMC_MSCNT) & 0x3FF;
      move_steps(12345, 30.0f);  move_wait();
      HAL_Delay(50);
      uint32_t m1 = tmc5160_read(TMC_MSCNT) & 0x3FF;
      dbg("MSCNT d", (m1 - m0) & 0x3FF);
      move_steps(-12345, 30.0f); move_wait();        // back to 0
      HAL_Delay(500);
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  move_to_deg(90.0f, 30.0f);   move_wait();   HAL_Delay(500);
	  move_to_deg(180.0f, 30.0f);  move_wait();   HAL_Delay(500);
	  move_to_deg(90.0f, 30.0f);   move_wait();   HAL_Delay(500);
	  move_to_deg(0.0f, 30.0f);    move_wait();   HAL_Delay(500);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;   // [CHANGE 4] set in .ioc too
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
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* [CHANGE 4] CCR1 preload. HAL_TIM_PWM_ConfigChannel already sets OC1PE;
   * stated explicitly because the ramp depends on it. */
  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_1);

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
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin PA5 */
  GPIO_InitStruct.Pin = DIR_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TMC_CS_Pin */
  GPIO_InitStruct.Pin = TMC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TMC_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

	void motor_set_dir(uint8_t fwd)
	{
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, fwd ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

/* RPM -> step frequency (Hz). Internal helper. */
static uint32_t rpm_to_hz(float rpm)
{
    return (uint32_t)(rpm * STEPS_PER_REV / 60.0f);
}

/* [CHANGE 7] Acceleration in rpm/s -> 2a in steps/s^2. Call only while !g_moving. */
void set_accel_rpm_s(float rpm_per_s)
{
    g_two_a = 2.0f * rpm_per_s * STEPS_PER_REV / 60.0f;
}

/* [CHANGE 5] Ramp: step rate for a pulse `from_start` steps into the move,
 * with `to_end` steps still to follow it.
 * Constant accel => v^2 = v0^2 + 2*a*s. min(accel limb, decel limb, vmax)
 * is the trapezoid; short moves become triangles automatically. */
static uint32_t ramp_arr(uint32_t from_start, uint32_t to_end)
{
    float v_up = sqrtf(g_vstart2 + g_two_a * (float)from_start);
    float v_dn = sqrtf(g_vstart2 + g_two_a * (float)to_end);
    float v    = (v_up < v_dn) ? v_up : v_dn;
    if (v > g_vmax) v = g_vmax;

    uint32_t arr = (uint32_t)(TIM_TICK_HZ / v + 0.5f) - 1u;
    if (arr < 2u * STEP_PULSE_TICKS) arr = 2u * STEP_PULSE_TICKS;  // keep a low gap
    return arr;
}

/* [CHANGE 5] Write preload regs for the period AFTER the one now running. */
static void load_next_period(void)
{
    if (g_steps_done >= g_steps_total) {
        /* All pulses launched: next period is blank (CCR=0 -> pin stays low).
         * Its update event is where the ISR stops the timer. */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    } else {
        uint32_t from_start = g_steps_done;
        uint32_t to_end     = g_steps_total - g_steps_done - 1u;
        __HAL_TIM_SET_AUTORELOAD(&htim2, ramp_arr(from_start, to_end));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, STEP_PULSE_TICKS);
    }
}

/* [CHANGE 6] ISR: fires at the start of every period, after preload -> shadow. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;

    if (g_steps_done >= g_steps_total) {          // blank period began -> done
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        g_moving = 0;
        return;
    }

    /* Hardware has just launched the next STEP pulse. */
    g_steps_done++;
    g_steps_remaining = g_steps_total - g_steps_done;
    g_position_steps += g_step_dir;

    load_next_period();
}

/* [CHANGE 6] Relative move: n steps signed, cruise speed in RPM. */
void move_steps(int32_t n, float rpm)
{
    uint32_t hz = rpm_to_hz(rpm);
    if (n == 0 || hz == 0 || g_moving) return;

    g_step_dir = (n > 0) ? 1 : -1;
    motor_set_dir(n > 0);

    g_steps_total = (uint32_t)(n > 0 ? n : -n);
    g_vmax        = (float)hz;
    g_moving      = 1;

    /* Period 1: write preload, UG copies it to shadow and zeroes CNT. */
    __HAL_TIM_SET_AUTORELOAD(&htim2, ramp_arr(0, g_steps_total - 1u));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, STEP_PULSE_TICKS);
    htim2.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);   // UG sets UIF; discard

    /* PWM1: pin goes high the instant the counter runs (CNT=0 < CCR),
     * so pulse 1 is launched by the start itself — count it here. */
    g_steps_done      = 1;
    g_steps_remaining = g_steps_total - 1u;
    g_position_steps += g_step_dir;
    load_next_period();                              // preload period 2 (or blank)

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/* Absolute move to an angle in DEGREES, speed in RPM. */
void move_to_deg(float deg, float rpm)
{
    int32_t target = (int32_t)(deg * STEPS_PER_DEG);
    move_steps(target - g_position_steps, rpm);
}

/* Absolute move in RADIANS — kept for the kinematics layer. */
void move_to_rad(float rad, float rpm)
{
    int32_t target = (int32_t)(rad * STEPS_PER_RAD);
    move_steps(target - g_position_steps, rpm);
}

float get_angle_deg(void) { return g_position_steps / STEPS_PER_DEG; }
float get_angle_rad(void) { return g_position_steps / STEPS_PER_RAD; }

/* [CHANGE 6] Block until the timer has actually stopped. */
void move_wait(void)
{
    while (g_moving) { __NOP(); }
}

/* [CHANGE 6] True while a move is in progress. Non-blocking. */
uint8_t move_busy(void)
{
    return g_moving;
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
