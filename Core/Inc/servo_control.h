#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <stdint.h>

#define SERVO_COUNT     6
#define SERVO_MIN_PULSE 500   // 0° 对应的脉宽(us)
#define SERVO_MAX_PULSE 2500  // 180° 对应的脉宽(us)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// 舵机ID定义
#define SERVO_BASE      0   // J1 底座
#define SERVO_SHOULDER  1   // J2 大臂
#define SERVO_ELBOW     2   // J3 小臂
#define SERVO_WRIST_P   3   // J4 腕部俯仰
#define SERVO_WRIST_R   4   // J5 腕部旋转
#define SERVO_GRIPPER   5   // J6 夹爪

// 夹爪角度（根据实际舵机标定后修改）
#define GRIPPER_OPEN    0
#define GRIPPER_CLOSE   90

// 初始化PWM（需在CubeMX配好TIM后调用）
void Servo_Init(void);

// 立即设置单个舵机角度（0-180）
void Servo_SetAngle(int id, int angle);

// 立即设置全部6个舵机
void Servo_SetAll(int angles[6]);

// 获取当前实际角度
void Servo_GetCurrent(int angles[6]);

// ====== 非阻塞S曲线插值 ======

// 启动一次插值移动（不会阻塞）
// target: 目标角度数组，total_steps: 总步数，step_ms: 每步间隔ms
void Servo_MoveStart(int target[6], int total_steps, int step_ms);

// 查询插值是否完成，返回1=完成，0=进行中
int Servo_MoveIsDone(void);

// 强制终止当前插值
void Servo_MoveStop(void);

// 这个函数必须在主循环中周期性调用（或从TIM中断调用）
// 每次调用推进一个插值步
void Servo_Tick(void);

#endif
