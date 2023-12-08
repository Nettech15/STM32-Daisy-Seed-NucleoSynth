#pragma once

#include "daisysp.h"

// Synth Engine
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"

void Error_Handler(void);

/* Defines ------------------------------------------------------------------*/
#define User_Led_Pin GPIO_PIN_7
#define User_Led_GPIO_Port GPIOC
#define Boot_Button_Pin GPIO_PIN_3
#define Boot_Button_GPIO_Port GPIOG
