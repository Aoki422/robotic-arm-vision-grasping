#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

// 仿射变换矩阵：2×3
// [wx]   [a  b  c] [px]
// [wy] = [d  e  f] [py]
//                [1]
typedef struct {
    float a, b, c;  // wx = a*px + b*py + c
    float d, e, f;  // wy = d*px + e*py + f
} AffineMatrix;

typedef struct {
    int px;
    int py;
    float wx;
    float wy;
} CalibPoint;

#define CALIB_SIM_POINT_COUNT 9u

// 初始化标定参数
void Calib_Init(const AffineMatrix* matrix);

// 像素坐标 → 世界坐标（单位：mm）
void Calib_PixelToWorld(int px, int py, float* wx, float* wy);

// 运行时更新标定矩阵
void Calib_SetMatrix(const AffineMatrix* matrix);

// 用9点或更多点通过最小二乘求解仿射矩阵，返回0表示成功
int Calib_SolveAffine(const CalibPoint* points, uint8_t count, AffineMatrix* out_matrix);

// 计算给定矩阵在标定点上的最大重投影误差，单位mm
float Calib_ComputeMaxError(const CalibPoint* points, uint8_t count, const AffineMatrix* matrix);

// 生成无硬件依赖的虚拟9点标定数据，用于PC单测和仿真
void Calib_GenerateSimulatedPoints(const AffineMatrix* truth, CalibPoint points[CALIB_SIM_POINT_COUNT]);

#endif
