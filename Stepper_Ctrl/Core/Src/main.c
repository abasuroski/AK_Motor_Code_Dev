/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Stepper motor control via TMC2209 (Step/Dir mode)
  *
  * Trapezoidal motion profile: accelerates to max velocity, cruises,
  * then decelerates to stop at target position.
  *
  * Pins:
  *   PA6  = STEP  (toggled in TIM3 ISR)  — CN10 pin 13 (D12)
  *   PA7  = DIR   (high = CW, low = CCW) — CN10 pin 15 (D11)
  *   PB6  = EN    (active low)           — CN10 pin 17 (D10)
  *   PA5  = LD2   (onboard LED, toggles each move)
  *   PA2  = UART2 TX (feedback to PC at 115200 via ST-Link)
  *
  * Timer: TIM3 with 1 MHz tick (PSC=83), variable ARR for step frequency.
  *        Each TIM3 update interrupt = one step pulse.
  *
  * Hardware: STM32F446RE Nucleo + TMC2209 V1.3
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ============================================================
//  MOTION PARAMETERS — edit these
// ============================================================
#define TARGET_STEPS      3200        // target position (steps from 0)
#define MAX_VELOCITY      800         // max speed (steps/sec)
#define ACCELERATION      1600        // acceleration (steps/sec^2)

// ============================================================
//  TIMER CONFIG
// ============================================================
#define TIM_CLOCK_FREQ    1000000UL   // 1 MHz after prescaler (84MHz / 84)
#define MIN_PERIOD        (TIM_CLOCK_FREQ / MAX_VELOCITY)

// ============================================================
//  MOTION STATE (modified by ISR)
// ============================================================
volatile int32_t  current_pos    = 0;       // current position (steps)
volatile int32_t  target_pos     = 0;       // target position (steps)
volatile uint32_t current_period = 0;       // current timer period (ticks)
volatile uint8_t  moving         = 0;       // 1 = in motion
volatile float    current_vel    = 0.0f;    // current velocity (steps/s)

// Internal planner state
static int32_t  move_dir        = 1;        // +1 or -1
static int32_t  total_steps     = 0;        // total steps in this move
static int32_t  steps_done      = 0;        // steps completed
static int32_t  accel_steps     = 0;        // steps in accel phase
static int32_t  decel_start     = 0;        // step at which decel begins
static float    vel             = 0.0f;     // instantaneous velocity (steps/s)
static float    accel_f         = 0.0f;     // acceleration as float

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void start_move(int32_t target);
static void stop_move(void);
static void update_period(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void start_move(int32_t target)
{
    if (target == current_pos) return;

    target_pos = target;
    int32_t delta = target - current_pos;
    move_dir = (delta > 0) ? 1 : -1;
    total_steps = abs(delta);
    steps_done = 0;

    // Set direction pin
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
                      (move_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Compute trapezoidal profile
    accel_f = (float)ACCELERATION;
    float max_vel_f = (float)MAX_VELOCITY;

    // Steps needed to accelerate to max velocity: v^2 / (2*a)
    accel_steps = (int32_t)(max_vel_f * max_vel_f / (2.0f * accel_f));

    // If we can't reach max velocity, use triangular profile
    if (accel_steps > total_steps / 2) {
        accel_steps = total_steps / 2;
    }
    decel_start = total_steps - accel_steps;

    // Start with initial low velocity (prevents division by zero)
    vel = sqrtf(2.0f * accel_f);  // velocity after 1 step of acceleration
    current_vel = vel;

    // Set initial timer period
    uint32_t period = (uint32_t)(TIM_CLOCK_FREQ / vel);
    if (period > 65535) period = 65535;
    if (period < 10) period = 10;
    current_period = period;

    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    moving = 1;
    HAL_TIM_Base_Start_IT(&htim3);
}

static void stop_move(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    moving = 0;
    vel = 0.0f;
    current_vel = 0.0f;
}

static void update_period(void)
{
    float max_vel_f = (float)MAX_VELOCITY;

    if (steps_done < accel_steps) {
        // Accelerating
        vel = sqrtf(2.0f * accel_f * (float)(steps_done + 1));
        if (vel > max_vel_f) vel = max_vel_f;
    } else if (steps_done >= decel_start) {
        // Decelerating
        int32_t steps_left = total_steps - steps_done;
        vel = sqrtf(2.0f * accel_f * (float)steps_left);
        if (vel < sqrtf(2.0f * accel_f)) vel = sqrtf(2.0f * accel_f);
    } else {
        // Cruising
        vel = max_vel_f;
    }

    current_vel = vel;
    uint32_t period = (uint32_t)(TIM_CLOCK_FREQ / vel);
    if (period > 65535) period = 65535;
    if (period < 10) period = 10;
    current_period = period;

    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1);
}

// TIM3 update ISR — one step per interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;
    if (!moving) return;

    // Pulse STEP pin (rising edge triggers step on TMC2209)
    // TMC2209 needs minimum ~100ns pulse, give it ~2us to be safe
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < 40; d++) {}
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);

    current_pos += move_dir;
    steps_done++;

    if (steps_done >= total_steps) {
        stop_move();
        return;
    }

    update_period();
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    // Turn on LED immediately to confirm we got past GPIO init
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

    MX_USART2_UART_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */

    // Enable TIM3 interrupt
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    // Enable the driver (active low)
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    // Start move to target
    start_move(TARGET_STEPS);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    uint32_t last_print = 0;
    while (1)
    {
        uint32_t now = HAL_GetTick();
        if (now - last_print >= 50)  // 20 Hz feedback
        {
            last_print = now;
            char buf[80];
            int len = snprintf(buf, sizeof(buf),
                "pos=%ld  vel=%.1f  target=%ld  %s\r\n",
                (long)current_pos, (double)current_vel,
                (long)target_pos, moving ? "MOVING" : "IDLE");
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 50);
        }

        HAL_Delay(1);
    }
    /* USER CODE END WHILE */
}

/* -------------------------------------------------------------------------- */
/*  Peripheral init                                                           */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

// TIM3: 1 MHz tick (84 MHz APB1 timer clock / 84), variable ARR
static void MX_TIM3_Init(void)
{
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 83;           // 84 MHz / 84 = 1 MHz
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;         // will be set by start_move()
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // LD2 (PA5)
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    // STEP (PA6)
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = STEP_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(STEP_GPIO_Port, &GPIO_InitStruct);

    // DIR (PA7)
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = DIR_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

    // EN (PB6) — active low, start disabled (high)
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_GPIO_Port, &GPIO_InitStruct);


    // B1 user button (PC13)
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
