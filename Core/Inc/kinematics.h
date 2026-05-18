#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>
#include "sim_config.h"

// 机械臂连杆长度（单位：mm，根据实际尺寸修改）
#define ARM_L1  50   // 底座到肩部高度
#define ARM_L2  120  // 大臂长度
#define ARM_L3  100  // 小臂长度
#define ARM_L4  60   // 腕部到夹爪长度

// 夹爪抓取高度（桌面以上 mm）
#define GRASP_Z  20

typedef enum {
    IK_OK = 0,
    IK_ERR_ARG = -1,
    IK_ERR_WORKSPACE = -2,
    IK_ERR_UNREACHABLE = -3,
    IK_ERR_JOINT_LIMIT = -4
} IKStatus;

typedef struct {
    float x;
    float y;
    float z;
    float yaw_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_pitch_deg;
    float wrist_roll_deg;
} KinematicPose;

// 逆运动学求解
// 输入：目标点桌面坐标 (x, y, z) 单位mm，末端 roll 角度
// 输出：angles[6] 六个舵机角度
// 返回：0=成功，负数=保护拒绝
int IK_Solve(float x, float y, float z, float roll, int angles[6]);

// 正运动学：由舵机角度回算末端中心位置，用于PC单测与仿真验证
int IK_Forward(const int angles[6], KinematicPose* pose);

// 仅检查目标点是否落在仿真工作区内
int IK_ValidateTarget(float x, float y, float z);

// 检查舵机角度是否满足软限位
int IK_ValidateAngles(const int angles[6]);

// 注入仿真/实机参数；传NULL时恢复默认参数
void IK_SetConfig(const SimConfig* config);

const char* IK_StatusName(int status);

// 设置/获取 Home 位置角度
void IK_SetHome(const int home_angles[6]);
void IK_GetHome(int home_angles[6]);

#endif
