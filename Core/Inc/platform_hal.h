#ifndef PLATFORM_HAL_H
#define PLATFORM_HAL_H

#include <stdint.h>

/*
 * 平台HAL隔离层。
 *
 * PC_SIM 模式下不依赖真实STM32 HAL、定时器、串口或中断；
 * 实机适配时取消 PC_SIM，即可继续包含 CubeMX 生成的 stm32f1xx_hal.h。
 */
#ifdef PC_SIM

typedef struct {
    uint32_t compare[4];
    uint8_t pwm_started[4];
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1 1u
#define TIM_CHANNEL_2 2u
#define TIM_CHANNEL_3 3u
#define TIM_CHANNEL_4 4u

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t channel);
void HAL_SetCompare(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t pulse);
uint32_t HAL_GetTick(void);
void HAL_SetTick(uint32_t tick_ms);
void HAL_AdvanceTick(uint32_t delta_ms);
void __disable_irq(void);
void __enable_irq(void);

#define __HAL_TIM_SET_COMPARE(htim, channel, pulse) HAL_SetCompare((htim), (channel), (pulse))

#else

#include "stm32f1xx_hal.h"

#endif

#endif
