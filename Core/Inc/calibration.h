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

// 初始化标定参数
void Calib_Init(const AffineMatrix* matrix);

// 像素坐标 → 世界坐标（单位：mm）
void Calib_PixelToWorld(int px, int py, float* wx, float* wy);

// 运行时更新标定矩阵
void Calib_SetMatrix(const AffineMatrix* matrix);

#endif
