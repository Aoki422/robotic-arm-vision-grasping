#include "grasp_state_machine.h"
#include "kinematics.h"
#include "calibration.h"
#include "servo_control.h"
#include "uart_protocol.h"

static GraspState g_state = GRASP_IDLE;
static int g_retry = 0;

// 预计算的关节角度
static int g_grasp_angles[6];
static int g_approach_angles[6];
static int g_place_angles[6];
static int g_place_approach[6];
static int g_home_angles[6];

const char* Grasp_GetStateName(void) {
    switch (g_state) {
        case GRASP_IDLE:        return "IDLE";
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

void Grasp_Init(void) {
    g_state = GRASP_IDLE;
    g_retry = 0;
    Servo_MoveStop();
    IK_GetHome(g_home_angles);
}

void Grasp_Tick(void) {
    float wx, wy;
    uint8_t color;
    int16_t px, py;
    int ret;

    switch (g_state) {

    case GRASP_IDLE:
        // 使用中断安全接口读取MV4数据
        if (UART_ReadFrame(&color, &px, &py)) {
            // 坐标转换：像素 → 世界坐标
            Calib_PixelToWorld(px, py, &wx, &wy);

            // 预计算 APPROACH 位置（目标上方安全高度）
            ret = IK_Solve(wx, wy, GRASP_Z + SAFE_Z_OFFSET, 0, g_approach_angles);
            if (ret != 0) break;  // 不可达

            // 预计算 GRASP 位置（桌面高度）
            ret = IK_Solve(wx, wy, GRASP_Z, 0, g_grasp_angles);
            if (ret != 0) break;

            // 预计算放置区角度
            IK_Solve(PLACE_X, PLACE_Y, PLACE_Z + SAFE_Z_OFFSET, 0, g_place_approach);
            IK_Solve(PLACE_X, PLACE_Y, PLACE_Z, 0, g_place_angles);

            g_state = GRASP_APPROACH;
        }
        break;

    case GRASP_APPROACH:
        Servo_MoveStart(g_approach_angles, 50, 20);
        g_state = GRASP_DESCEND;
        break;

    case GRASP_DESCEND:
        if (Servo_MoveIsDone()) {
            Servo_MoveStart(g_grasp_angles, 20, 30);
            g_state = GRASP_CLOSE;
        }
        break;

    case GRASP_CLOSE:
        if (Servo_MoveIsDone()) {
            Servo_SetAngle(5, 90);      // 闭合夹爪
            HAL_Delay(300);
            g_state = GRASP_LIFT;
        }
        break;

    case GRASP_LIFT:
        Servo_MoveStart(g_approach_angles, 20, 20);
        g_state = GRASP_MOVE_PLACE;
        break;

    case GRASP_MOVE_PLACE:
        if (Servo_MoveIsDone()) {
            Servo_MoveStart(g_place_approach, 50, 20);
            g_state = GRASP_RELEASE;
        }
        break;

    case GRASP_RELEASE:
        if (Servo_MoveIsDone()) {
            Servo_SetAngle(5, 0);       // 张开夹爪
            HAL_Delay(300);
            g_state = GRASP_RETURN_HOME;
        }
        break;

    case GRASP_RETURN_HOME:
        Servo_MoveStart(g_home_angles, 30, 20);
        g_state = GRASP_DONE;
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
            g_state = GRASP_APPROACH;
        } else {
            g_state = GRASP_RETURN_HOME;
        }
        break;
    }
}
