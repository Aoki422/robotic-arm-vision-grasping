#include "grasp_state_machine.h"
#include "kinematics.h"
#include "calibration.h"
#include "servo_control.h"
#include "uart_protocol.h"
#include "platform_hal.h"
#include "debug_log.h"
#include "sim_config.h"
#include "sim_metrics.h"

#include <math.h>

static GraspState g_state = GRASP_IDLE;
static GraspError g_last_error = GRASP_ERR_NONE;
static int g_retry = 0;
static uint32_t g_wait_until = 0;
static uint32_t g_motion_deadline = 0;
static uint8_t g_confirm_count = 0;

// 预计算的关节角度
static int g_grasp_angles[6];
static int g_approach_angles[6];
static int g_place_angles[6];
static int g_place_approach[6];
static int g_home_angles[6];
static float g_place_x = PLACE_X;
static float g_place_y = PLACE_Y;
static float g_place_z = PLACE_Z;
static uint8_t g_candidate_color = 0;
static int16_t g_candidate_px = 0;
static int16_t g_candidate_py = 0;
static uint32_t g_last_frame_tick = 0;

static const char* state_name(GraspState state) {
    switch (state) {
        case GRASP_IDLE:        return "IDLE";
        case GRASP_CONFIRM_TARGET: return "CONFIRM_TARGET";
        case GRASP_APPROACH:    return "APPROACH";
        case GRASP_DESCEND:     return "DESCEND";
        case GRASP_CLOSE:       return "CLOSE";
        case GRASP_LIFT:        return "LIFT";
        case GRASP_MOVE_PLACE:  return "MOVE_PLACE";
        case GRASP_RELEASE:     return "RELEASE";
        case GRASP_RETURN_HOME: return "RETURN_HOME";
        case GRASP_DONE:        return "DONE";
        case GRASP_FAILED:      return "FAILED";
        default:                return "UNKNOWN";
    }
}

const char* Grasp_GetStateName(void) {
    return state_name(g_state);
}

GraspState Grasp_GetState(void) {
    return g_state;
}

GraspError Grasp_GetLastError(void) {
    return g_last_error;
}

void Grasp_SetPlaceTarget(float x, float y, float z) {
    g_place_x = x;
    g_place_y = y;
    g_place_z = z;
}

static const char* error_name(GraspError error) {
    switch (error) {
        case GRASP_ERR_NONE: return "NONE";
        case GRASP_ERR_TARGET_LOST: return "TARGET_LOST";
        case GRASP_ERR_TARGET_UNREACHABLE: return "TARGET_UNREACHABLE";
        case GRASP_ERR_PLACE_UNREACHABLE: return "PLACE_UNREACHABLE";
        case GRASP_ERR_MOTION_TIMEOUT: return "MOTION_TIMEOUT";
        case GRASP_ERR_JOINT_LIMIT: return "JOINT_LIMIT";
        default: return "UNKNOWN";
    }
}

static void transition_to(GraspState next, const char* reason) {
    if (g_state != next) {
        DebugLog_Info("GRASP", "%s -> %s : %s", state_name(g_state), state_name(next), reason ? reason : "");
    }
    g_state = next;
}

static void reset_candidate(void) {
    g_confirm_count = 0;
    g_candidate_color = 0;
    g_candidate_px = 0;
    g_candidate_py = 0;
    g_last_frame_tick = 0;
}

static void begin_motion(GraspState state, const int target[6], int steps, int step_ms, const char* reason) {
    uint32_t expected_ms = (uint32_t)steps * (uint32_t)step_ms;
    Servo_MoveStart(target, steps, step_ms);
    g_motion_deadline = HAL_GetTick() + expected_ms + SimConfig_Get()->motion_timeout_ms;
    transition_to(state, reason);
}

static void begin_return_home(const char* reason) {
    begin_motion(GRASP_RETURN_HOME, g_home_angles, 30, 20, reason);
}

static void fail_with(GraspError error, const char* reason, uint8_t return_home) {
    g_last_error = error;
    DebugLog_Info("GRASP_ERR", "%s : %s", error_name(error), reason ? reason : "");
    Metrics_RecordGraspFailure();
    if (error == GRASP_ERR_TARGET_LOST) Metrics_RecordTargetLost();
    if (error == GRASP_ERR_MOTION_TIMEOUT) Metrics_RecordMotionTimeout();
    if (error == GRASP_ERR_TARGET_UNREACHABLE || error == GRASP_ERR_PLACE_UNREACHABLE) {
        Metrics_RecordIkUnreachable();
    }

    if (return_home) {
        begin_return_home("failure fallback home");
    } else {
        transition_to(GRASP_FAILED, "failure without motion");
    }
}

static uint8_t motion_timed_out(void) {
    if (!Servo_MoveIsDone() && HAL_GetTick() > g_motion_deadline) {
        Servo_MoveStop();
        fail_with(GRASP_ERR_MOTION_TIMEOUT, "motion deadline exceeded", 1u);
        return 1u;
    }
    return 0u;
}

static int is_same_candidate(uint8_t color, int16_t px, int16_t py) {
    float dx = (float)px - (float)g_candidate_px;
    float dy = (float)py - (float)g_candidate_py;
    float dist = sqrtf(dx * dx + dy * dy);
    return color == g_candidate_color && dist <= SimConfig_Get()->target_confirm_pixel_tolerance;
}

static void consume_target_frames(void) {
    uint8_t color;
    int16_t px;
    int16_t py;

    while (UART_ReadFrame(&color, &px, &py)) {
        Metrics_RecordTargetFrame();
        g_last_frame_tick = HAL_GetTick();

        if (g_confirm_count == 0u || !is_same_candidate(color, px, py)) {
            g_candidate_color = color;
            g_candidate_px = px;
            g_candidate_py = py;
            g_confirm_count = 1u;
            DebugLog_Info("VISION", "new candidate color=%u px=%d py=%d", color, px, py);
        } else if (g_confirm_count < 255u) {
            g_confirm_count++;
            g_candidate_px = px;
            g_candidate_py = py;
        }

        if (g_confirm_count >= SimConfig_Get()->target_confirm_frames) {
            transition_to(GRASP_CONFIRM_TARGET, "target stable");
            return;
        }
    }

    if (g_confirm_count > 0u &&
        (HAL_GetTick() - g_last_frame_tick) > SimConfig_Get()->target_lost_timeout_ms) {
        reset_candidate();
        fail_with(GRASP_ERR_TARGET_LOST, "target disappeared before confirmation", 0u);
    }
}

void Grasp_Init(void) {
    Grasp_Reset();
}

void Grasp_Reset(void) {
    g_state = GRASP_IDLE;
    g_last_error = GRASP_ERR_NONE;
    g_retry = 0;
    g_wait_until = 0;
    g_motion_deadline = 0;
    reset_candidate();
    Servo_MoveStop();
    IK_GetHome(g_home_angles);
    DebugLog_Info("GRASP", "state machine reset");
}

void Grasp_Tick(void) {
    float wx, wy;
    int ret;

    switch (g_state) {

    case GRASP_IDLE:
        consume_target_frames();
        break;

    case GRASP_CONFIRM_TARGET:
        Calib_PixelToWorld(g_candidate_px, g_candidate_py, &wx, &wy);
        Metrics_RecordConfirmedTarget();
        Metrics_RecordGraspAttempt();

        ret = IK_Solve(wx, wy, GRASP_Z + SAFE_Z_OFFSET, 0, g_approach_angles);
        if (ret != IK_OK) {
            fail_with(GRASP_ERR_TARGET_UNREACHABLE, IK_StatusName(ret), 0u);
            break;
        }

        ret = IK_Solve(wx, wy, GRASP_Z, 0, g_grasp_angles);
        if (ret != IK_OK) {
            fail_with(GRASP_ERR_TARGET_UNREACHABLE, IK_StatusName(ret), 0u);
            break;
        }

        ret = IK_Solve(g_place_x, g_place_y, g_place_z + SAFE_Z_OFFSET, 0, g_place_approach);
        if (ret != IK_OK) {
            fail_with(GRASP_ERR_PLACE_UNREACHABLE, IK_StatusName(ret), 0u);
            break;
        }

        ret = IK_Solve(g_place_x, g_place_y, g_place_z, 0, g_place_angles);
        if (ret != IK_OK) {
            fail_with(GRASP_ERR_PLACE_UNREACHABLE, IK_StatusName(ret), 0u);
            break;
        }

        g_last_error = GRASP_ERR_NONE;
        reset_candidate();
        begin_motion(GRASP_APPROACH, g_approach_angles, 50, 20, "approach target");
        break;

    case GRASP_APPROACH:
        if (motion_timed_out()) break;
        if (Servo_MoveIsDone()) {
            begin_motion(GRASP_DESCEND, g_grasp_angles, 20, 30, "descend to grasp");
        }
        break;

    case GRASP_DESCEND:
        if (motion_timed_out()) break;
        if (Servo_MoveIsDone()) {
            Servo_SetAngle(5, 90);
            g_wait_until = HAL_GetTick() + 300u;
            transition_to(GRASP_CLOSE, "close gripper");
        }
        break;

    case GRASP_CLOSE:
        if (HAL_GetTick() >= g_wait_until) {
            begin_motion(GRASP_LIFT, g_approach_angles, 20, 20, "lift object");
        }
        break;

    case GRASP_LIFT:
        if (motion_timed_out()) break;
        if (Servo_MoveIsDone()) {
            begin_motion(GRASP_MOVE_PLACE, g_place_approach, 50, 20, "move to place");
        }
        break;

    case GRASP_MOVE_PLACE:
        if (motion_timed_out()) break;
        if (Servo_MoveIsDone()) {
            Servo_SetAngle(5, 0);
            g_wait_until = HAL_GetTick() + 300u;
            transition_to(GRASP_RELEASE, "release gripper");
        }
        break;

    case GRASP_RELEASE:
        if (HAL_GetTick() >= g_wait_until) {
            Metrics_RecordGraspSuccess();
            Metrics_Print();
            begin_return_home("return home after success");
        }
        break;

    case GRASP_RETURN_HOME:
        if (HAL_GetTick() > g_motion_deadline) {
            Servo_MoveStop();
            transition_to(GRASP_DONE, "home motion timeout forced idle");
            break;
        }
        if (Servo_MoveIsDone()) {
            transition_to(GRASP_DONE, "home reached");
        }
        break;

    case GRASP_DONE:
        if (Servo_MoveIsDone()) {
            g_retry = 0;
            g_state = GRASP_IDLE;
        }
        break;

    case GRASP_FAILED:
        if (g_retry < MAX_RETRY) {
            g_retry++;
            transition_to(GRASP_IDLE, "retry waits for new stable target");
        } else {
            begin_return_home("retry exhausted");
        }
        break;
    }
}
