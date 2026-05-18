#ifdef PC_SIM

#include "platform_hal.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

static uint32_t g_pc_tick_ms = 0;
static uint32_t g_irq_disable_depth = 0;

static uint32_t channel_to_index(uint32_t channel) {
    if (channel == TIM_CHANNEL_1) return 0;
    if (channel == TIM_CHANNEL_2) return 1;
    if (channel == TIM_CHANNEL_3) return 2;
    return 3;
}

void HAL_TIM_PWM_Start(TIM_HandleTypeDef* htim, uint32_t channel) {
    if (htim == 0) return;
    htim->pwm_started[channel_to_index(channel)] = 1u;
}

void HAL_SetCompare(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t pulse) {
    if (htim == 0) return;
    htim->compare[channel_to_index(channel)] = pulse;
}

uint32_t HAL_GetTick(void) {
    return g_pc_tick_ms;
}

void HAL_SetTick(uint32_t tick_ms) {
    g_pc_tick_ms = tick_ms;
}

void HAL_AdvanceTick(uint32_t delta_ms) {
    g_pc_tick_ms += delta_ms;
}

void __disable_irq(void) {
    g_irq_disable_depth++;
}

void __enable_irq(void) {
    if (g_irq_disable_depth > 0u) {
        g_irq_disable_depth--;
    }
}

#endif
