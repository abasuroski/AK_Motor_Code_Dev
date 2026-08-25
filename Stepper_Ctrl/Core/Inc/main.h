#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

// Nucleo-F446RE pin defines
#define B1_Pin          GPIO_PIN_13
#define B1_GPIO_Port    GPIOC
#define LD2_Pin         GPIO_PIN_5
#define LD2_GPIO_Port   GPIOA

// Stepper pins
#define STEP_Pin        GPIO_PIN_6
#define STEP_GPIO_Port  GPIOA
#define DIR_Pin         GPIO_PIN_7
#define DIR_GPIO_Port   GPIOA
#define EN_Pin          GPIO_PIN_6
#define EN_GPIO_Port    GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
