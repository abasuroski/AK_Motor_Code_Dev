/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AK60-6 MIT mode — direct parameter control
  *
  * MIT mode torque equation:
  *   τ = Kp*(p_des - p) + Kd*(v_des - v) + t_ff
  *
  * Set the five parameters below to get any control mode:
  *
  *   Pure position hold:   Kp>0, Kd>0,  v_des=0, t_ff=0
  *   Pure velocity:        Kp=0, Kd>0,  v_des=target, t_ff=0
  *   Pure torque:          Kp=0, Kd=0,  v_des=0, t_ff=target
  *   Impedance + gravity:  Kp>0, Kd>0,  v_des=0, t_ff=gravity_comp
  *
  * Position units: radians  (MIT range: -12.56 to +12.56 rad = ±4π)
  * Velocity units: rad/s    (MIT range: -60 to +60 rad/s)
  * Torque units:   N·m      (MIT range: -12 to +12 N·m)
  *
  * Feedback over UART2 at 115200 baud (PA2/PA3).
  * Hardware: STM32F446RE Nucleo + Waveshare CAN shield, motor CAN ID = 1
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ============================================================
//  CONTROL PARAMETERS — edit these
// ============================================================
#define TARGET_POS_RAD    0.0f     // desired position  (rad)
#define TARGET_VEL_RAD    0.0f     // desired velocity  (rad/s)
#define CTRL_KP           8.0f     // position gain     (0 – 500)
#define CTRL_KD           0.4f     // velocity gain     (0 – 5)
#define CTRL_T_FF         0.0f     // feedforward torque (N·m)

// ============================================================
//  AK60-6 MIT PARAMETER LIMITS — do not exceed
// ============================================================
#define P_MIN    -12.56f
#define P_MAX     12.56f
#define V_MIN    -60.0f
#define V_MAX     60.0f
#define T_MIN    -12.0f
#define T_MAX     12.0f
#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

// ============================================================
//  MOTOR CONFIG
// ============================================================
#define MOTOR_CAN_ID      1
#define CAN_PACKET_MIT    8

// ============================================================
//  MOTOR STATE (written by RX ISR)
// ============================================================
volatile float   motor_pos   = 0.0f;   // degrees (servo feedback format)
volatile float   motor_spd   = 0.0f;   // eRPM
volatile float   motor_cur   = 0.0f;   // amps
volatile int8_t  motor_temp  = 0;
volatile int8_t  motor_error = 0;
volatile uint8_t fb_ready    = 0;

// ============================================================
//  CAN HANDLES
// ============================================================
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t  txData[8];
uint8_t  rxData[8];
uint32_t txMailbox;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
static float        uint_to_float(uint16_t x_int, float x_min, float x_max, int bits);
static void         mit_send(float p_des, float v_des, float kp, float kd, float t_ff);
static void         motor_enable(void);
static void         motor_disable(void);
static void         unpack_reply(void);
static uint8_t      wait_for_feedback(uint32_t timeout_ms);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// ----------------------------------------------------------------
// MIT enable / disable (magic byte sequences)
// ----------------------------------------------------------------
static void motor_enable(void)
{
    uint8_t buf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)MOTOR_CAN_ID | ((uint32_t)CAN_PACKET_MIT << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, buf, &txMailbox);
}

static void motor_disable(void)
{
    uint8_t buf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)MOTOR_CAN_ID | ((uint32_t)CAN_PACKET_MIT << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, buf, &txMailbox);
}

// ----------------------------------------------------------------
// Fixed-point encoding helpers
// ----------------------------------------------------------------
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) / (x_max - x_min) * (float)((1u << bits) - 1));
}

static float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    return (float)x_int / (float)((1 << bits) - 1) * (x_max - x_min) + x_min;
}

// ----------------------------------------------------------------
// MIT CAN frame — 8 bytes, extended ID
//
//  Byte  Bits    Field
//  [0]   11:4    KP  high 8
//  [1]    3:0    KP  low  4  |  KD high 4
//  [2]    7:0    KD  low  8
//  [3]   15:8    Position high 8  (16-bit)
//  [4]    7:0    Position low  8
//  [5]   11:4    Velocity high 8  (12-bit)
//  [6]    3:0    Velocity low  4  |  Torque high 4  (12-bit)
//  [7]    7:0    Torque low 8
// ----------------------------------------------------------------
static void mit_send(float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
    uint16_t p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16);
    uint16_t v_int  = float_to_uint(v_des, V_MIN,  V_MAX,  12);
    uint16_t t_int  = float_to_uint(t_ff,  T_MIN,  T_MAX,  12);

    txData[0] = (kp_int >> 4) & 0xFF;
    txData[1] = ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF);
    txData[2] = kd_int & 0xFF;
    txData[3] = (p_int >> 8) & 0xFF;
    txData[4] = p_int & 0xFF;
    txData[5] = (v_int >> 4) & 0xFF;           // correct: 12-bit velocity high 8
    txData[6] = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)MOTOR_CAN_ID | ((uint32_t)CAN_PACKET_MIT << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// ----------------------------------------------------------------
// Decode servo-mode feedback frame
//   pos  : int16 * 0.1  = degrees
//   spd  : int16 * 10   = eRPM
//   cur  : int16 * 0.01 = amps
// ----------------------------------------------------------------
static void unpack_reply(void)
{
    int16_t pos_int = (int16_t)((rxData[0] << 8) | rxData[1]);
    int16_t spd_int = (int16_t)((rxData[2] << 8) | rxData[3]);
    int16_t cur_int = (int16_t)((rxData[4] << 8) | rxData[5]);

    motor_pos   = (float)pos_int *  0.1f;
    motor_spd   = (float)spd_int * 10.0f;
    motor_cur   = (float)cur_int *  0.01f;
    motor_temp  = (int8_t)rxData[6];
    motor_error = (int8_t)rxData[7];

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "RX | pos=%.1f deg  spd=%.0f eRPM  I=%.2f A  T=%d  err=%d\r\n",
        (double)motor_pos, (double)motor_spd,
        (double)motor_cur, motor_temp, motor_error);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

// ----------------------------------------------------------------
// CAN RX ISR
// ----------------------------------------------------------------
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;
    unpack_reply();
    fb_ready = 1;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

static uint8_t wait_for_feedback(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (!fb_ready && (HAL_GetTick() - t0) < timeout_ms) {}
    return fb_ready;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_Delay(100);

    // Enable motor into MIT mode
    motor_enable();
    HAL_Delay(100);

    // Wait for first feedback to confirm motor is alive
    wait_for_feedback(500);

    // Soft-start: ramp gains from 0 to full over 1 s to avoid torque spike
    for (int i = 1; i <= 50; i++)
    {
        float kp_r = CTRL_KP * (float)i / 50.0f;
        float kd_r = CTRL_KD * (float)i / 50.0f;
        mit_send(TARGET_POS_RAD, TARGET_VEL_RAD, kp_r, kd_r, CTRL_T_FF);
        HAL_Delay(20);
    }

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        mit_send(TARGET_POS_RAD, TARGET_VEL_RAD, CTRL_KP, CTRL_KD, CTRL_T_FF);

        if (fb_ready)
            fb_ready = 0;

        HAL_Delay(10);  // 100 Hz
    }
    /* USER CODE END WHILE */
}

/* -------------------------------------------------------------------------- */
/*  Peripheral init — identical to spring-back project (same hardware)        */
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

static void MX_CAN1_Init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation     = ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;

    hcan1.Instance              = CAN1;
    hcan1.Init.Prescaler        = 3;
    hcan1.Init.Mode             = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth    = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1         = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2         = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();

    HAL_CAN_ConfigFilter(&hcan1, &filter);
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

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
