#ifndef SIM_METRICS_H
#define SIM_METRICS_H

#include <stdint.h>

typedef struct {
    uint32_t target_frames;
    uint32_t confirmed_targets;
    uint32_t grasp_attempts;
    uint32_t grasp_success;
    uint32_t grasp_failed;
    uint32_t ik_unreachable;
    uint32_t motion_timeout;
    uint32_t target_lost;
    float last_position_error_mm;
    float max_position_error_mm;
} SimMetrics;

void Metrics_Reset(void);
void Metrics_Get(SimMetrics* out);
void Metrics_RecordTargetFrame(void);
void Metrics_RecordConfirmedTarget(void);
void Metrics_RecordGraspAttempt(void);
void Metrics_RecordGraspSuccess(void);
void Metrics_RecordGraspFailure(void);
void Metrics_RecordIkUnreachable(void);
void Metrics_RecordMotionTimeout(void);
void Metrics_RecordTargetLost(void);
void Metrics_RecordPositionError(float error_mm);
float Metrics_GetSuccessRate(void);
int Metrics_IsPositionWithinTolerance(float error_mm);
void Metrics_Print(void);

#endif
