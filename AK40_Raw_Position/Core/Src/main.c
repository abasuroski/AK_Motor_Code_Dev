/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : AK40-10 MIT mode — raw position control
  *
  * MIT mode torque equation:
  *   τ = Kp*(p_des - p) + Kd*(v_des - v) + t_ff
  *
  * AK40-10 Parameter Ranges:
  *   Position: -12.5 to +12.5 rad
  *   Velocity: -45.5 to +45.5 rad/s
  *   Torque:   -5.0 to +5.0 N·m
  *   Kp:       0 to 500
  *   Kd:       0 to 5
  *
  * CAN: Standard frame, ID = 3, 1 Mbit/s
  * Feedback over UART2 at 115200 baud (PA2/PA3).
  * Hardware: STM32F446RE Nucleo + Waveshare CAN shield
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
#define TARGET_POS_RAD    6.0f     // desired position  (rad)
#define TARGET_VEL_RAD    0.5f     // desired velocity  (rad/s)
#define CTRL_KP           6.0f     // position gain     (0 – 500)
#define CTRL_KD           0.2f     // velocity gain     (0 – 5)
#define CTRL_T_FF         0.0f     // feedforward torque (N·m)

// ============================================================
//  AK40-10 MIT PARAMETER LIMITS
// ============================================================
#define P_MIN    -12.5f
#define P_MAX     12.5f
#define V_MIN    -45.5f
#define V_MAX     45.5f
#define T_MIN    -5.0f
#define T_MAX     5.0f
#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

// ============================================================
//  MOTOR CONFIG
// ============================================================
#define MOTOR_CAN_ID      3

// ============================================================
//  MOTOR STATE (written by RX ISR)
// ============================================================
volatile float   motor_pos   = 6.0f;   // rad
volatile float   motor_vel   = 0.1f;   // rad/s
volatile float   motor_torque = 0.0f;  // N·m
volatile int8_t  motor_temp  = 0;      // deg C (raw - 40)
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
static float        uint_to_float(int x_int, float x_min, float x_max, int bits);
static void         pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff);
static void         send_cmd(void);
static void         motor_enable(void);
static void         unpack_reply(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// ----------------------------------------------------------------
// Fixed-point encoding helpers (from manual)
// ----------------------------------------------------------------
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) * ((float)(1 << bits) / span));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

// ----------------------------------------------------------------
// Send raw CAN frame (standard ID)
// ----------------------------------------------------------------
static void send_cmd(void)
{
    txHeader.StdId              = MOTOR_CAN_ID;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// ----------------------------------------------------------------
// Enter MIT control mode (required before sending commands)
// ----------------------------------------------------------------
static void motor_enable(void)
{
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFC;
    send_cmd();
}

// ----------------------------------------------------------------
// Pack MIT command frame
//   [0-1]: position 16-bit
//   [2]:   velocity high 8
//   [3]:   velocity low 4 | Kp high 4
//   [4]:   Kp low 8
//   [5]:   Kd high 8
//   [6]:   Kd low 4 | torque high 4
//   [7]:   torque low 8
// ----------------------------------------------------------------
static void pack_cmd(float p_des, float v_des, float kp, float kd, float t_ff)
{
    unsigned int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
    unsigned int t_int  = float_to_uint(t_ff,  T_MIN, T_MAX, 12);

    txData[0] = p_int >> 8;
    txData[1] = p_int & 0xFF;
    txData[2] = v_int >> 4;
    txData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    txData[4] = kp_int & 0xFF;
    txData[5] = kd_int >> 4;
    txData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    txData[7] = t_int & 0xFF;
}

// ----------------------------------------------------------------
// Unpack motor feedback
//   [0]: driver ID
//   [1-2]: position 16-bit
//   [3] high 8, [4] high 4: velocity 12-bit
//   [4] low 4, [5]: torque 12-bit
//   [6]: temperature (raw - 40 = deg C)
//   [7]: error code
// ----------------------------------------------------------------
static void unpack_reply(void)
{
    int p_int = (rxData[1] << 8) | rxData[2];
    int v_int = (rxData[3] << 4) | (rxData[4] >> 4);
    int i_int = ((rxData[4] & 0xF) << 8) | rxData[5];

    motor_pos    = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_vel    = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_torque = uint_to_float(i_int, -T_MAX, T_MAX, 12);
    motor_temp   = (int8_t)(rxData[6] - 40);
    motor_error  = (int8_t)rxData[7];

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "pos=%.3f rad  vel=%.2f  tau=%.2f  T=%d  err=%d\r\n",
        (double)motor_pos, (double)motor_vel,
        (double)motor_torque, motor_temp, motor_error);
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

    // Enter MIT control mode
    for (int i = 0; i < 10; i++)
    {
        motor_enable();
        HAL_Delay(10);
    }
    HAL_Delay(100);

    // Soft-start: ramp gains from 0 to full over 1 s
    for (int i = 1; i <= 50; i++)
    {
        float kp_r = CTRL_KP * (float)i / 50.0f;
        float kd_r = CTRL_KD * (float)i / 50.0f;
        pack_cmd(TARGET_POS_RAD, TARGET_VEL_RAD, kp_r, kd_r, CTRL_T_FF);
        send_cmd();
        HAL_Delay(20);
    }

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        pack_cmd(TARGET_POS_RAD, TARGET_VEL_RAD, CTRL_KP, CTRL_KD, CTRL_T_FF);
        send_cmd();

        if (fb_ready)
            fb_ready = 0;

        HAL_Delay(10);  // 100 Hz
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
    HAL_CAN_ConfigFilter(&hcan1, &filter);

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
