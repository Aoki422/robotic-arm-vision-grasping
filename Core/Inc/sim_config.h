#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <stdint.h>

#define SIM_JOINT_COUNT 6

typedef struct {
    float zero_deg;
    float direction;
    float soft_min_deg;
    float soft_max_deg;
} JointSimConfig;

typedef struct {
    float arm_l1_mm;
    float arm_l2_mm;
    float arm_l3_mm;
    float arm_l4_mm;
    float gripper_center_z_offset_mm;

    float workspace_x_min_mm;
    float workspace_x_max_mm;
    float workspace_y_min_mm;
    float workspace_y_max_mm;
    float workspace_z_min_mm;
    float workspace_z_max_mm;

    float positioning_tolerance_mm;
    float target_confirm_pixel_tolerance;
    uint8_t target_confirm_frames;
    uint32_t target_lost_timeout_ms;
    uint32_t motion_timeout_ms;

    int servo_min_pulse_us;
    int servo_max_pulse_us;
    JointSimConfig joints[SIM_JOINT_COUNT];
} SimConfig;

void SimConfig_SetDefaults(SimConfig* config);
const SimConfig* SimConfig_Get(void);
void SimConfig_Apply(const SimConfig* config);
int SimConfig_LoadIni(const char* path, SimConfig* config);
int SimConfig_LoadDefaultIni(const char* path);

#endif
