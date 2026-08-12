/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Position Control - AK60-6, MIT Mode, return-to-home
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

// ===== AK60-6 MIT PARAMETER RANGES =====
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

// ===== MOTOR CONFIG =====
#define MOTOR_CAN_ID      1
#define CAN_PACKET_MIT    8

// Base gains - motor should yield to hand at KP=2, return smoothly
#define CTRL_KP           8.0f
#define CTRL_KD           0.4f

// Velocity low-pass: 0=max smooth, 1=no filter
#define VEL_ALPHA         0.15f

// ===== MOTOR STATE (written by RX ISR) =====
volatile float    motor_pos     = 0.0f;
volatile float    motor_vel     = 0.0f;
volatile uint8_t  fb_ready      = 0;

float vel_filt = 0.0f;
float home_pos = 0.0f;   // captured after first valid feedback

// ===== MOTOR STATE =====
float pos = 0.0f; // Current Position
float vel = 0.0f; // Current Velocity
// ===== CAN HANDLES =====
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
static uint8_t wait_for_feedback(uint32_t timeout_ms);
/* USER CODE END PFP */


/* USER CODE BEGIN 0 */

// ----------------------------------------------------------------
// Encoding helpers
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
// MIT frame - correct byte layout per CubeMars datasheet:
//
//   Byte  Bits    Field
//   [0]   15:8    Position high
//   [1]    7:0    Position low
//   [2]   11:4    Velocity high
//   [3]    3:0    Velocity low | KP[11:8]
//   [4]    7:0    KP low
//   [5]   11:4    KD high
//   [6]    3:0    KD low | Torque[11:8]
//   [7]    7:0    Torque low
// ----------------------------------------------------------------
static void mit_send(float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 16);
    uint16_t t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);

    txData[0] = (kp_int >> 4) & 0xFF;
    txData[1] = ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF);
    txData[2] = kd_int & 0xFF;
    txData[3] = (p_int >> 8) & 0xFF;
    txData[4] = p_int & 0xFF;
    txData[5] = (v_int >> 8) & 0xFF;
    txData[6] = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    txData[7] = t_int & 0xFF;

    // --- DEBUG TX ---
    char buf[128];
    int len;

    len = snprintf(buf, sizeof(buf),
        "TX CMD  | p_des=%.3f v_des=%.3f kp=%.3f kd=%.3f t_ff=%.3f\r\n",
        (double)p_des, (double)v_des, (double)kp, (double)kd, (double)t_ff);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

    len = snprintf(buf, sizeof(buf),
        "TX BITS | %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        txData[0], txData[1], txData[2], txData[3],
        txData[4], txData[5], txData[6], txData[7]);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);

    txHeader.ExtId = ((uint32_t)CAN_PACKET_MIT << 8) | MOTOR_CAN_ID;
    txHeader.IDE   = CAN_ID_EXT;
    txHeader.RTR   = CAN_RTR_DATA;
    txHeader.DLC   = 8;

    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// ----------------------------------------------------------------
// Motor enable / disable (MIT magic bytes)
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
void unpack_reply()
{
    // ---------------- RAW 16-bit decoding (FIXED ALIGNMENT) ----------------
    int16_t p_raw = (int16_t)((rxData[0] << 8) | rxData[1]);
    int16_t v_raw = (int16_t)((rxData[2] << 8) | rxData[3]);
    int16_t i_raw = (int16_t)((rxData[4] << 8) | rxData[5]);

    int8_t temp  = (int8_t)rxData[6];
    int8_t error = (int8_t)rxData[7];

    // ---------------- SCALING (MATCHES CUBEMARS EXAMPLE) ----------------
    pos = (float)p_raw * 0.1f;   // position (deg or internal units per spec)
    vel = (float)v_raw * 10.0f;  // speed (eRPM)
    float current = (float)i_raw * 0.01f;

    // ---------------- DEBUG PRINT (UNCHANGED STYLE) ----------------
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "pos=%.2f deg vel=%.2f eRPM I=%.2f T=%d\r\n",
        (double)pos,
        (double)vel,
        (double)current,
        temp);

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
void motor_receive(float *motor_pos,
                   float *motor_spd,
                   float *motor_cur,
                   int8_t *motor_temp,
                   int8_t *motor_error,
                   CAN_RxHeaderTypeDef *rx_message,
                   uint8_t *rx_data)
{
    int16_t pos_int = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t spd_int = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t cur_int = (int16_t)((rx_data[4] << 8) | rx_data[5]);

    *motor_pos = (float)pos_int * 0.1f;
    *motor_spd = (float)spd_int * 10.0f;
    *motor_cur = (float)cur_int * 0.01f;

    *motor_temp  = (int8_t)rx_data[6];
    *motor_error = (int8_t)rx_data[7];
}

// ----------------------------------------------------------------
// Block until ISR delivers at least one feedback frame, or timeout
// ----------------------------------------------------------------
static uint8_t wait_for_feedback(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while (!fb_ready && (HAL_GetTick() - t0) < timeout_ms) {}

    return fb_ready;
}

// ----------------------------------------------------------------
// CAN RX ISR - decodes MIT feedback frame
//
//   Byte  Field
//   [0]   Motor ID (ignored)
//   [1]   Position[15:8]
//   [2]   Position[7:0]
//   [3]   Velocity[11:4]
//   [4]   Velocity[3:0] | Torque[11:8]
//   [5]   Torque[7:0]
//   [6]   Temperature
//   [7]   Error code

// ----------------------------------------------------------------
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
//        return;
//
//    // Decode position and velocity from feedback frame
//    uint16_t p_raw = ((uint16_t)rxData[1] << 8) | rxData[2];
//    uint16_t v_raw = ((uint16_t)rxData[3] << 4) | (rxData[4] >> 4);
//
//    motor_pos = uint_to_float(p_raw, P_MIN, P_MAX, 16);
//    motor_vel = uint_to_float(v_raw, V_MIN, V_MAX, 12);
//    fb_ready  = 1;
//
//    // Temporary debug
//    char dbg[48];
//    int len = snprintf(dbg, sizeof(dbg), "RX pos=%.3f vel=%.3f\r\n",
//					   (double)motor_pos, (double)motor_vel);
//    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, 100);
//}
/* USER CODE END 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    unpack_reply();

    fb_ready = 1;

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_CAN_Start(&hcan1);

    HAL_CAN_ActivateNotification(&hcan1,
                                 CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    char boot[] = "MCU STARTED\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)boot, strlen(boot), 100);

    // 1. Enable motor then block until we receive real position feedback
    HAL_Delay(100);
    //motor_enable();

    if (!wait_for_feedback(500))
    {
        char err[] = "NO FEEDBACK\r\n";
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)err,
                          strlen(err),
                          100);
    }
    else
    {
        char ok[] = "FEEDBACK RECEIVED\r\n";
        HAL_UART_Transmit(&huart2,
                          (uint8_t*)ok,
                          strlen(ok),
                          100);
    }
    HAL_Delay(50);

    // 2. Capture home - wherever the shaft is right now becomes the target
    home_pos = motor_pos;

    char hbuf[48];
    int hlen = snprintf(hbuf, sizeof(hbuf), "Home: %.4f rad\r\n", (double)home_pos);
    HAL_UART_Transmit(&huart2, (uint8_t*)hbuf, (uint16_t)hlen, 100);

    // 3. Soft-start: ramp gains from 0 to full over 1 second holding home_pos
    //    Eliminates torque spike if motor is displaced at power-on
    for (int i = 1; i <= 50; i++)
    {
        float kp_r = CTRL_KP * (float)i / 50.0f;
        float kd_r = CTRL_KD * (float)i / 50.0f;
        mit_send(home_pos, 0.0f, kp_r, kd_r, 0.0f);
        //mit_send(1.0f, 0.0f, 50.0f, 1.0f, 0.2f);
        HAL_Delay(20);
    }

    char ready[] = "Control loop active\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)ready, strlen(ready), 100);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        float pos = motor_pos;
        float vel = motor_vel;

        // Velocity low-pass filter
        vel_filt = VEL_ALPHA * vel + (1.0f - VEL_ALPHA) * vel_filt;

        float error = home_pos - pos;

        // Dynamic KP: soften in the last ~3 deg to stop buzzing near home
        float kp_eff = CTRL_KP;

        // Dynamic KD: extra damping proportional to speed
        float kd_eff = CTRL_KD + 0.05f * fabsf(vel_filt);
        if (kd_eff > KD_MAX) kd_eff = KD_MAX;

        //mit_send(1.0f, 0.0f, 50.0f, 1.0f, 0.5f);
        mit_send(home_pos, 0.0f, kp_eff, kd_eff, 0.0f);

        if (fb_ready)
        {
            fb_ready = 0;
//            char buf[72];
//            int len = snprintf(buf, sizeof(buf),
//                "pos=%.3f err=%.3f kp=%.2f kd=%.3f\r\n",
//                (double)pos, (double)error,
//                (double)kp_eff, (double)kd_eff);
//            HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)len, 50);
        }

        HAL_Delay(10);  // 100 Hz
    }
    /* USER CODE END WHILE */
}


/* -------------------------------------------------------------------------- */
/*  Peripheral init (unchanged from CubeMX)                                   */
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
    hcan1.Init.Mode = CAN_MODE_NORMAL;  // was CAN_MODE_NORMAL
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
