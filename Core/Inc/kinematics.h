#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

// 机械臂连杆长度（单位：mm，根据实际尺寸修改）
#define ARM_L1  50   // 底座到肩部高度
#define ARM_L2  120  // 大臂长度
#define ARM_L3  100  // 小臂长度
#define ARM_L4  60   // 腕部到夹爪长度

// 夹爪抓取高度（桌面以上 mm）
#define GRASP_Z  20

// 逆运动学求解
// 输入：目标点桌面坐标 (x, y, z) 单位mm，末端 roll 角度
// 输出：angles[6] 六个舵机角度
// 返回：0=成功, -1=目标不可达
int IK_Solve(float x, float y, float z, float roll, int angles[6]);

// 设置/获取 Home 位置角度
void IK_SetHome(const int home_angles[6]);
void IK_GetHome(int home_angles[6]);

#endif
