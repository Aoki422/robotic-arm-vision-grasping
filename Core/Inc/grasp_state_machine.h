#ifndef GRASP_STATE_MACHINE_H
#define GRASP_STATE_MACHINE_H

#include <stdint.h>

typedef enum {
    GRASP_IDLE = 0,
    GRASP_CONFIRM_TARGET,
    GRASP_APPROACH,
    GRASP_DESCEND,
    GRASP_CLOSE,
    GRASP_LIFT,
    GRASP_MOVE_PLACE,
    GRASP_RELEASE,
    GRASP_RETURN_HOME,
    GRASP_DONE,
    GRASP_FAILED
} GraspState;

typedef enum {
    GRASP_ERR_NONE = 0,
    GRASP_ERR_TARGET_LOST,
    GRASP_ERR_TARGET_UNREACHABLE,
    GRASP_ERR_PLACE_UNREACHABLE,
    GRASP_ERR_MOTION_TIMEOUT,
    GRASP_ERR_JOINT_LIMIT
} GraspError;

// 放置区坐标（mm，相对机械臂基坐标系）
#define PLACE_X     150
#define PLACE_Y     0
#define PLACE_Z     20

// 安全高度偏移（mm）
#define SAFE_Z_OFFSET  50

// 最大重试次数
#define MAX_RETRY   2

// 初始化状态机
void Grasp_Init(void);

// 状态机 Tick（在主循环中高频调用）
void Grasp_Tick(void);

// 获取当前状态名称
const char* Grasp_GetStateName(void);

GraspState Grasp_GetState(void);
GraspError Grasp_GetLastError(void);
void Grasp_Reset(void);
void Grasp_SetPlaceTarget(float x, float y, float z);

#endif
