#include "sim_metrics.h"

#include "debug_log.h"
#include "sim_config.h"

#include <string.h>

static SimMetrics g_metrics;

void Metrics_Reset(void) {
    memset(&g_metrics, 0, sizeof(g_metrics));
}

void Metrics_Get(SimMetrics* out) {
    if (out) {
        *out = g_metrics;
    }
}

void Metrics_RecordTargetFrame(void) {
    g_metrics.target_frames++;
}

void Metrics_RecordConfirmedTarget(void) {
    g_metrics.confirmed_targets++;
}

void Metrics_RecordGraspAttempt(void) {
    g_metrics.grasp_attempts++;
}

void Metrics_RecordGraspSuccess(void) {
    g_metrics.grasp_success++;
}

void Metrics_RecordGraspFailure(void) {
    g_metrics.grasp_failed++;
}

void Metrics_RecordIkUnreachable(void) {
    g_metrics.ik_unreachable++;
}

void Metrics_RecordMotionTimeout(void) {
    g_metrics.motion_timeout++;
}

void Metrics_RecordTargetLost(void) {
    g_metrics.target_lost++;
}

void Metrics_RecordPositionError(float error_mm) {
    g_metrics.last_position_error_mm = error_mm;
    if (error_mm > g_metrics.max_position_error_mm) {
        g_metrics.max_position_error_mm = error_mm;
    }
}

float Metrics_GetSuccessRate(void) {
    if (g_metrics.grasp_attempts == 0u) return 0.0f;
    return (float)g_metrics.grasp_success * 100.0f / (float)g_metrics.grasp_attempts;
}

int Metrics_IsPositionWithinTolerance(float error_mm) {
    return error_mm <= SimConfig_Get()->positioning_tolerance_mm;
}

void Metrics_Print(void) {
    DebugLog_Info(
        "METRICS",
        "frames=%lu confirmed=%lu attempts=%lu success=%lu failed=%lu success_rate=%.1f%% max_error=%.2fmm ik_unreachable=%lu motion_timeout=%lu target_lost=%lu",
        (unsigned long)g_metrics.target_frames,
        (unsigned long)g_metrics.confirmed_targets,
        (unsigned long)g_metrics.grasp_attempts,
        (unsigned long)g_metrics.grasp_success,
        (unsigned long)g_metrics.grasp_failed,
        Metrics_GetSuccessRate(),
        g_metrics.max_position_error_mm,
        (unsigned long)g_metrics.ik_unreachable,
        (unsigned long)g_metrics.motion_timeout,
        (unsigned long)g_metrics.target_lost);
}
