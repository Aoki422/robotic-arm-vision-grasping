#include "kinematics.h"
#include <math.h>

static int g_home[6] = {90, 90, 90, 90, 0, 0};
static SimConfig g_kin_config;
static uint8_t g_kin_config_ready = 0;

static const float PI_F = 3.14159265358979323846f;

static void ensure_config(void) {
    if (!g_kin_config_ready) {
        g_kin_config = *SimConfig_Get();
        g_kin_config_ready = 1u;
    }
}

static float rad_to_deg(float rad) {
    return rad * 180.0f / PI_F;
}

static float deg_to_rad(float deg) {
    return deg * PI_F / 180.0f;
}

static int map_raw_to_servo(int joint, float raw_deg, int* servo_angle) {
    const JointSimConfig* cfg;
    float mapped;

    ensure_config();
    if (joint < 0 || joint >= SIM_JOINT_COUNT || servo_angle == 0) {
        return IK_ERR_ARG;
    }

    cfg = &g_kin_config.joints[joint];
    mapped = cfg->zero_deg + cfg->direction * raw_deg;

    if (mapped < cfg->soft_min_deg || mapped > cfg->soft_max_deg) {
        return IK_ERR_JOINT_LIMIT;
    }

    if (mapped < 0.0f || mapped > 180.0f) {
        return IK_ERR_JOINT_LIMIT;
    }

    *servo_angle = (int)(mapped + 0.5f);
    return IK_OK;
}

static float map_servo_to_raw(int joint, int servo_angle) {
    const JointSimConfig* cfg;
    ensure_config();
    cfg = &g_kin_config.joints[joint];
    if (fabsf(cfg->direction) < 1.0e-6f) {
        return 0.0f;
    }
    return ((float)servo_angle - cfg->zero_deg) / cfg->direction;
}

void IK_SetHome(const int home_angles[6]) {
    for (int i = 0; i < 6; i++) g_home[i] = home_angles[i];
}

void IK_GetHome(int home_angles[6]) {
    for (int i = 0; i < 6; i++) home_angles[i] = g_home[i];
}

void IK_SetConfig(const SimConfig* config) {
    if (config) {
        g_kin_config = *config;
    } else {
        SimConfig_SetDefaults(&g_kin_config);
    }
    g_kin_config_ready = 1u;
}

int IK_ValidateTarget(float x, float y, float z) {
    ensure_config();
    if (x < g_kin_config.workspace_x_min_mm || x > g_kin_config.workspace_x_max_mm) {
        return IK_ERR_WORKSPACE;
    }
    if (y < g_kin_config.workspace_y_min_mm || y > g_kin_config.workspace_y_max_mm) {
        return IK_ERR_WORKSPACE;
    }
    if (z < g_kin_config.workspace_z_min_mm || z > g_kin_config.workspace_z_max_mm) {
        return IK_ERR_WORKSPACE;
    }
    return IK_OK;
}

int IK_ValidateAngles(const int angles[6]) {
    ensure_config();
    if (angles == 0) {
        return IK_ERR_ARG;
    }

    for (int i = 0; i < SIM_JOINT_COUNT; i++) {
        if ((float)angles[i] < g_kin_config.joints[i].soft_min_deg ||
            (float)angles[i] > g_kin_config.joints[i].soft_max_deg) {
            return IK_ERR_JOINT_LIMIT;
        }
    }
    return IK_OK;
}

int IK_Solve(float x, float y, float z, float roll, int angles[6]) {
    float target_r;
    float wrist_r;
    float wrist_z;
    float plane_z;
    float d;
    float cos_elbow;
    float elbow_rad;
    float shoulder_rad;
    float yaw_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_pitch_deg;
    int status;

    ensure_config();
    if (angles == 0) {
        return IK_ERR_ARG;
    }

    status = IK_ValidateTarget(x, y, z);
    if (status != IK_OK) {
        return status;
    }

    /*
     * ARM_L4 表示腕部到夹爪中心的径向偏移。
     * IK先求腕部中心，再由腕部补偿保持夹爪姿态。
     */
    target_r = sqrtf(x * x + y * y);
    wrist_r = target_r - g_kin_config.arm_l4_mm;
    wrist_z = z + g_kin_config.gripper_center_z_offset_mm;
    plane_z = wrist_z - g_kin_config.arm_l1_mm;

    if (wrist_r <= 1.0f) {
        return IK_ERR_UNREACHABLE;
    }

    d = sqrtf(wrist_r * wrist_r + plane_z * plane_z);
    if (d > (g_kin_config.arm_l2_mm + g_kin_config.arm_l3_mm) ||
        d < fabsf(g_kin_config.arm_l2_mm - g_kin_config.arm_l3_mm)) {
        return IK_ERR_UNREACHABLE;
    }

    cos_elbow = (d * d - g_kin_config.arm_l2_mm * g_kin_config.arm_l2_mm -
                 g_kin_config.arm_l3_mm * g_kin_config.arm_l3_mm) /
                (2.0f * g_kin_config.arm_l2_mm * g_kin_config.arm_l3_mm);

    if (cos_elbow < -1.0f) cos_elbow = -1.0f;
    if (cos_elbow > 1.0f) cos_elbow = 1.0f;

    elbow_rad = acosf(cos_elbow);
    shoulder_rad = atan2f(plane_z, wrist_r) -
                   atan2f(g_kin_config.arm_l3_mm * sinf(elbow_rad),
                          g_kin_config.arm_l2_mm + g_kin_config.arm_l3_mm * cosf(elbow_rad));

    yaw_deg = rad_to_deg(atan2f(y, x));
    shoulder_deg = rad_to_deg(shoulder_rad);
    elbow_deg = rad_to_deg(elbow_rad);

    /*
     * 姿态补偿：末端希望近似保持水平/朝下。
     * roll 参数映射到J5，J4抵消肩肘累计俯仰。
     */
    wrist_pitch_deg = -(shoulder_deg + elbow_deg);

    status = map_raw_to_servo(0, yaw_deg, &angles[0]);
    if (status != IK_OK) return status;
    status = map_raw_to_servo(1, shoulder_deg, &angles[1]);
    if (status != IK_OK) return status;
    status = map_raw_to_servo(2, elbow_deg, &angles[2]);
    if (status != IK_OK) return status;
    status = map_raw_to_servo(3, wrist_pitch_deg, &angles[3]);
    if (status != IK_OK) return status;
    status = map_raw_to_servo(4, roll, &angles[4]);
    if (status != IK_OK) return status;
    status = map_raw_to_servo(5, 0.0f, &angles[5]);
    if (status != IK_OK) return status;

    return IK_ValidateAngles(angles);
}

int IK_Forward(const int angles[6], KinematicPose* pose) {
    float yaw;
    float shoulder;
    float elbow;
    float wrist;
    float roll;
    float radial;
    float z;

    ensure_config();
    if (angles == 0 || pose == 0) {
        return IK_ERR_ARG;
    }

    if (IK_ValidateAngles(angles) != IK_OK) {
        return IK_ERR_JOINT_LIMIT;
    }

    yaw = map_servo_to_raw(0, angles[0]);
    shoulder = map_servo_to_raw(1, angles[1]);
    elbow = map_servo_to_raw(2, angles[2]);
    wrist = map_servo_to_raw(3, angles[3]);
    roll = map_servo_to_raw(4, angles[4]);

    radial = g_kin_config.arm_l2_mm * cosf(deg_to_rad(shoulder)) +
             g_kin_config.arm_l3_mm * cosf(deg_to_rad(shoulder + elbow)) +
             g_kin_config.arm_l4_mm;
    z = g_kin_config.arm_l1_mm +
        g_kin_config.arm_l2_mm * sinf(deg_to_rad(shoulder)) +
        g_kin_config.arm_l3_mm * sinf(deg_to_rad(shoulder + elbow)) -
        g_kin_config.gripper_center_z_offset_mm;

    pose->x = radial * cosf(deg_to_rad(yaw));
    pose->y = radial * sinf(deg_to_rad(yaw));
    pose->z = z;
    pose->yaw_deg = yaw;
    pose->shoulder_deg = shoulder;
    pose->elbow_deg = elbow;
    pose->wrist_pitch_deg = wrist;
    pose->wrist_roll_deg = roll;
    return IK_OK;
}

const char* IK_StatusName(int status) {
    switch (status) {
        case IK_OK: return "OK";
        case IK_ERR_ARG: return "ARG";
        case IK_ERR_WORKSPACE: return "WORKSPACE";
        case IK_ERR_UNREACHABLE: return "UNREACHABLE";
        case IK_ERR_JOINT_LIMIT: return "JOINT_LIMIT";
        default: return "UNKNOWN";
    }
}
