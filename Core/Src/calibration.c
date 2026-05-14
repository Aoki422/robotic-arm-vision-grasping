/*
 * ===== 坐标标定模块 =====
 *
 * 本模块实现 9 点仿射变换坐标标定，将 MV4 相机输出的像素坐标
 * 转换为世界坐标系下的毫米坐标。
 *
 * --- PC 端标定流程（Python / numpy） ---
 *
 * 1. 在视野中摆放 9 个标记点（3×3 网格），记录每个点的：
 *      像素坐标 (px, py)   —— 由 MV4 相机获取
 *      世界坐标 (wx, wy)   —— 手动测量，单位 mm
 *
 * 2. 构造线性方程组并求解仿射矩阵：
 *
 *    import numpy as np
 *
 *    # 9 个点的像素坐标 (px, py)
 *    pixel = np.array([
 *        [px0, py0],
 *        [px1, py1],
 *        ...
 *        [px8, py8],
 *    ], dtype=float)
 *
 *    # 对应的世界坐标 (wx, wy)
 *    world = np.array([
 *        [wx0, wy0],
 *        [wx1, wy1],
 *        ...
 *        [wx8, wy8],
 *    ], dtype=float)
 *
 *    # 构造设计矩阵 A = [px, py, 1]
 *    ones = np.ones((9, 1))
 *    A = np.hstack([pixel, ones])          # shape (9, 3)
 *
 *    # 最小二乘求解：A * [a, b, c]^T = wx,  A * [d, e, f]^T = wy
 *    abc, _, _, _ = np.linalg.lstsq(A, world[:, 0], rcond=None)
 *    def_, _, _, _ = np.linalg.lstsq(A, world[:, 1], rcond=None)
 *
 *    # 结果
 *    a, b, c = abc
 *    d, e, f = def_
 *
 *    # 验证：计算重投影误差
 *    reprojected = A @ np.column_stack([abc, def_])  # (9, 2)
 *    errors = np.sqrt(np.sum((reprojected - world) ** 2, axis=1))
 *    print("Max reprojection error (mm):", np.max(errors))
 *
 * 3. 将求得的 a~f 系数填入 Calib_Init() 调用中即可。
 *
 * 注意：
 *   - 如果画面存在畸变，应先对像素坐标做去畸变后再标定。
 *   - 标定区域应覆盖整个工作范围，外推精度会显著下降。
 *   - 建议重投影误差 < 0.5 mm，否则需要重新采集标定点。
 */

#include "calibration.h"

static AffineMatrix g_matrix;

void Calib_Init(const AffineMatrix* matrix) {
    if (matrix) {
        g_matrix = *matrix;
    } else {
        // 默认单位映射：1像素=1mm（占位，实际必须标定）
        g_matrix.a = 1.0f; g_matrix.b = 0.0f; g_matrix.c = 0.0f;
        g_matrix.d = 0.0f; g_matrix.e = 1.0f; g_matrix.f = 0.0f;
    }
}

void Calib_PixelToWorld(int px, int py, float* wx, float* wy) {
    *wx = g_matrix.a * px + g_matrix.b * py + g_matrix.c;
    *wy = g_matrix.d * px + g_matrix.e * py + g_matrix.f;
}

void Calib_SetMatrix(const AffineMatrix* matrix) {
    if (matrix) {
        g_matrix = *matrix;
    }
}
