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

#include <math.h>
#include <string.h>

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

static int solve_3x3(float a[3][3], float b[3], float out[3]) {
    float m[3][4];
    int row;
    int col;

    for (row = 0; row < 3; row++) {
        for (col = 0; col < 3; col++) {
            m[row][col] = a[row][col];
        }
        m[row][3] = b[row];
    }

    for (col = 0; col < 3; col++) {
        int pivot = col;
        float max_abs = fabsf(m[col][col]);
        for (row = col + 1; row < 3; row++) {
            float v = fabsf(m[row][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = row;
            }
        }

        if (max_abs < 1.0e-6f) {
            return -1;
        }

        if (pivot != col) {
            for (int k = col; k < 4; k++) {
                float tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }

        {
            float div = m[col][col];
            for (int k = col; k < 4; k++) {
                m[col][k] /= div;
            }
        }

        for (row = 0; row < 3; row++) {
            if (row == col) continue;
            {
                float factor = m[row][col];
                for (int k = col; k < 4; k++) {
                    m[row][k] -= factor * m[col][k];
                }
            }
        }
    }

    out[0] = m[0][3];
    out[1] = m[1][3];
    out[2] = m[2][3];
    return 0;
}

int Calib_SolveAffine(const CalibPoint* points, uint8_t count, AffineMatrix* out_matrix) {
    float ata[3][3];
    float atx[3];
    float aty[3];
    float sol_x[3];
    float sol_y[3];

    if (points == 0 || out_matrix == 0 || count < 3u) {
        return -1;
    }

    memset(ata, 0, sizeof(ata));
    memset(atx, 0, sizeof(atx));
    memset(aty, 0, sizeof(aty));

    for (uint8_t i = 0; i < count; i++) {
        float row[3];
        row[0] = (float)points[i].px;
        row[1] = (float)points[i].py;
        row[2] = 1.0f;

        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ata[r][c] += row[r] * row[c];
            }
            atx[r] += row[r] * points[i].wx;
            aty[r] += row[r] * points[i].wy;
        }
    }

    if (solve_3x3(ata, atx, sol_x) != 0) {
        return -1;
    }

    /*
     * solve_3x3 会原地消元，所以 Y 方向需要重新构造一次 ATA。
     */
    memset(ata, 0, sizeof(ata));
    for (uint8_t i = 0; i < count; i++) {
        float row[3];
        row[0] = (float)points[i].px;
        row[1] = (float)points[i].py;
        row[2] = 1.0f;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ata[r][c] += row[r] * row[c];
            }
        }
    }

    if (solve_3x3(ata, aty, sol_y) != 0) {
        return -1;
    }

    out_matrix->a = sol_x[0];
    out_matrix->b = sol_x[1];
    out_matrix->c = sol_x[2];
    out_matrix->d = sol_y[0];
    out_matrix->e = sol_y[1];
    out_matrix->f = sol_y[2];
    return 0;
}

float Calib_ComputeMaxError(const CalibPoint* points, uint8_t count, const AffineMatrix* matrix) {
    float max_error = 0.0f;

    if (points == 0 || matrix == 0) {
        return -1.0f;
    }

    for (uint8_t i = 0; i < count; i++) {
        float wx = matrix->a * (float)points[i].px + matrix->b * (float)points[i].py + matrix->c;
        float wy = matrix->d * (float)points[i].px + matrix->e * (float)points[i].py + matrix->f;
        float dx = wx - points[i].wx;
        float dy = wy - points[i].wy;
        float err = sqrtf(dx * dx + dy * dy);
        if (err > max_error) {
            max_error = err;
        }
    }

    return max_error;
}

void Calib_GenerateSimulatedPoints(const AffineMatrix* truth, CalibPoint points[CALIB_SIM_POINT_COUNT]) {
    static const int pixels[CALIB_SIM_POINT_COUNT][2] = {
        {160, 140}, {320, 140}, {480, 140},
        {160, 260}, {320, 260}, {480, 260},
        {160, 380}, {320, 380}, {480, 380}
    };
    AffineMatrix default_truth;
    const AffineMatrix* m = truth;

    if (points == 0) return;

    if (m == 0) {
        default_truth.a = 0.50f;
        default_truth.b = 0.00f;
        default_truth.c = -80.0f;
        default_truth.d = 0.00f;
        default_truth.e = 0.50f;
        default_truth.f = -130.0f;
        m = &default_truth;
    }

    for (uint8_t i = 0; i < CALIB_SIM_POINT_COUNT; i++) {
        points[i].px = pixels[i][0];
        points[i].py = pixels[i][1];
        points[i].wx = m->a * (float)points[i].px + m->b * (float)points[i].py + m->c;
        points[i].wy = m->d * (float)points[i].px + m->e * (float)points[i].py + m->f;
    }
}
