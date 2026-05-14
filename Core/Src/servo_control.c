#include "servo_control.h"
#include <string.h>

// ====== 硬件映射 ======
// TIM2 CH1-4: J1-J4  (PA0-PA3)
// TIM3 CH1-2: J5-J6  (PA6-PA7)
// 用户在 CubeMX 中应配好 TIM2/TIM3 为 50Hz (PSC=71, ARR=19999 @72MHz)
// TIM_HandleTypeDef 全局变量由 CubeMX 生成，需要 extern

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// ====== 内部状态 ======
static int g_current[6] = {90, 90, 90, 90, 0, 0};
static int g_target[6];
static int g_start[6];
static int g_total_steps = 0;
static int g_step_ms = 0;
static uint32_t g_start_tick = 0;
static int g_moving = 0;

// ====== 辅助：角度 -> CCR 值 ======
static uint32_t AngleToPulse(int angle) {
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    return SERVO_MIN_PULSE + (uint32_t)((float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180.0f);
}

// ====== 硬件写PWM ======
static void Servo_WritePulse(int id, uint32_t pulse) {
    switch (id) {
        case 0: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse); break;
        case 1: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse); break;
        case 2: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse); break;
        case 3: __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse); break;
        case 4: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); break;
        case 5: __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); break;
        default: break;
    }
}

// ====== 公共接口 ======

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    int init_angles[6] = {90, 90, 90, 90, 0, 0};
    Servo_SetAll(init_angles);
}

void Servo_SetAngle(int id, int angle) {
    if (id < 0 || id >= SERVO_COUNT) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    g_current[id] = angle;
    Servo_WritePulse(id, AngleToPulse(angle));
}

void Servo_SetAll(int angles[6]) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        Servo_SetAngle(i, angles[i]);
    }
}

void Servo_GetCurrent(int angles[6]) {
    memcpy(angles, g_current, sizeof(g_current));
}

// ====== 非阻塞S曲线插值 ======

void Servo_MoveStart(int target[6], int total_steps, int step_ms) {
    if (total_steps <= 0) total_steps = 1;
    if (step_ms <= 0) step_ms = 10;               // 默认每步10ms
    memcpy(g_start, g_current, sizeof(g_start));
    memcpy(g_target, target, sizeof(g_target));
    g_total_steps = total_steps;
    g_step_ms = step_ms;
    g_start_tick = HAL_GetTick();                  // 记录开始时间
    g_moving = 1;
}

int Servo_MoveIsDone(void) {
    return !g_moving;
}

void Servo_MoveStop(void) {
    g_moving = 0;
}

void Servo_Tick(void) {
    if (!g_moving) return;

    uint32_t elapsed = HAL_GetTick() - g_start_tick;
    uint32_t total_duration = (uint32_t)g_total_steps * (uint32_t)g_step_ms;

    float t;
    if (total_duration == 0) {
        t = 1.0f;
    } else if (elapsed >= total_duration) {
        t = 1.0f;
    } else {
        t = (float)elapsed / (float)total_duration;
    }

    // 三次多项式S曲线：f(t) = 3t² - 2t³  (起止速度=0)
    float s = 3.0f * t * t - 2.0f * t * t * t;

    for (int i = 0; i < SERVO_COUNT; i++) {
        float diff = g_target[i] - g_start[i];
        int angle = (int)(g_start[i] + diff * s + 0.5f);
        if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
        if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
        g_current[i] = angle;
        Servo_WritePulse(i, AngleToPulse(angle));
    }

    if (t >= 1.0f) {
        g_moving = 0;
    }
}
