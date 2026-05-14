#include "kinematics.h"
#include <math.h>

static int g_home[6] = {90, 90, 90, 90, 0, 0};

void IK_SetHome(const int home_angles[6]) {
    for (int i = 0; i < 6; i++) g_home[i] = home_angles[i];
}

void IK_GetHome(int home_angles[6]) {
    for (int i = 0; i < 6; i++) home_angles[i] = g_home[i];
}

int IK_Solve(float x, float y, float z, float roll, int angles[6]) {
    // J1：底座旋转（方向角）
    float j1_rad = atan2f(y, x);
    float j1_deg = j1_rad * 180.0f / 3.14159265f;
    if (j1_deg < 0) j1_deg += 180.0f;
    angles[0] = (int)(j1_deg + 0.5f);

    float R = sqrtf(x * x + y * y);
    float Z = z - ARM_L1;
    float d = sqrtf(R * R + Z * Z);

    // J3：小臂（肘部）用余弦定理
    float cos_j3 = (d * d - ARM_L2 * ARM_L2 - ARM_L3 * ARM_L3)
                 / (2.0f * ARM_L2 * ARM_L3);

    if (cos_j3 < -1.0f || cos_j3 > 1.0f) {
        return -1;  // 不可达
    }

    float j3_rad = acosf(cos_j3);
    float j3_deg = j3_rad * 180.0f / 3.14159265f;
    angles[2] = (int)(j3_deg + 0.5f);

    // J2：大臂（肩部）
    float alpha = atan2f(Z, R);
    float beta = acosf((ARM_L2 * ARM_L2 + d * d - ARM_L3 * ARM_L3)
                      / (2.0f * ARM_L2 * d));
    float j2_rad = alpha + beta;
    float j2_deg = j2_rad * 180.0f / 3.14159265f;
    angles[1] = (int)(j2_deg + 0.5f);

    // J4：腕部俯仰（保持夹爪水平向下）
    float j4_deg = 90.0f - (j2_deg + j3_deg);
    if (j4_deg < 0)   j4_deg = 0;
    if (j4_deg > 180) j4_deg = 180;
    angles[3] = (int)(j4_deg + 0.5f);

    angles[4] = 0;                    // J5 固定
    angles[5] = 90;                   // J6 夹爪默认打开

    // 角度限位
    for (int i = 0; i < 6; i++) {
        if (angles[i] < 0)   angles[i] = 0;
        if (angles[i] > 180) angles[i] = 180;
    }

    return 0;
}
