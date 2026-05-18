#include "sim_config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SimConfig g_config;
static uint8_t g_config_ready = 0;

static void ensure_defaults(void) {
    if (!g_config_ready) {
        SimConfig_SetDefaults(&g_config);
        g_config_ready = 1u;
    }
}

void SimConfig_SetDefaults(SimConfig* config) {
    int i;
    if (config == 0) return;

    config->arm_l1_mm = 50.0f;
    config->arm_l2_mm = 120.0f;
    config->arm_l3_mm = 100.0f;
    config->arm_l4_mm = 60.0f;
    config->gripper_center_z_offset_mm = 0.0f;

    /* 默认工作区为 200mm x 150mm，位于机械臂前方桌面区域。 */
    config->workspace_x_min_mm = 40.0f;
    config->workspace_x_max_mm = 240.0f;
    config->workspace_y_min_mm = -75.0f;
    config->workspace_y_max_mm = 75.0f;
    config->workspace_z_min_mm = 0.0f;
    config->workspace_z_max_mm = 160.0f;

    config->positioning_tolerance_mm = 5.0f;
    config->target_confirm_pixel_tolerance = 6.0f;
    config->target_confirm_frames = 3u;
    config->target_lost_timeout_ms = 500u;
    config->motion_timeout_ms = 1200u;

    config->servo_min_pulse_us = 500;
    config->servo_max_pulse_us = 2500;

    for (i = 0; i < SIM_JOINT_COUNT; i++) {
        config->joints[i].zero_deg = 0.0f;
        config->joints[i].direction = 1.0f;
        config->joints[i].soft_min_deg = 0.0f;
        config->joints[i].soft_max_deg = 180.0f;
    }

    config->joints[0].zero_deg = 90.0f;  /* yaw: 90deg 对应正前方 */
    config->joints[1].zero_deg = 90.0f;  /* shoulder: 支持负仰角 */
    config->joints[2].zero_deg = 0.0f;
    config->joints[3].zero_deg = 135.0f; /* wrist pitch compensation, tuned for PC workspace coverage */
    config->joints[4].zero_deg = 90.0f;  /* wrist roll */
    config->joints[5].zero_deg = 0.0f;   /* gripper */
}

const SimConfig* SimConfig_Get(void) {
    ensure_defaults();
    return &g_config;
}

void SimConfig_Apply(const SimConfig* config) {
    if (config == 0) return;
    g_config = *config;
    g_config_ready = 1u;
}

static char* trim(char* text) {
    char* end;
    while (*text && isspace((unsigned char)*text)) text++;
    end = text + strlen(text);
    while (end > text && isspace((unsigned char)end[-1])) {
        end--;
        *end = '\0';
    }
    return text;
}

static int parse_joint_array(const char* value, SimConfig* config, int field) {
    char buf[192];
    char* token;
    int idx = 0;
    if (value == 0 || config == 0) return -1;

    (void)snprintf(buf, sizeof(buf), "%s", value);
    token = strtok(buf, ",");
    while (token != 0 && idx < SIM_JOINT_COUNT) {
        float v = (float)atof(trim(token));
        if (field == 0) config->joints[idx].zero_deg = v;
        if (field == 1) config->joints[idx].direction = v;
        if (field == 2) config->joints[idx].soft_min_deg = v;
        if (field == 3) config->joints[idx].soft_max_deg = v;
        idx++;
        token = strtok(0, ",");
    }

    return idx == SIM_JOINT_COUNT ? 0 : -1;
}

static void apply_key_value(SimConfig* config, const char* key, const char* value) {
    if (strcmp(key, "arm_l1_mm") == 0) config->arm_l1_mm = (float)atof(value);
    else if (strcmp(key, "arm_l2_mm") == 0) config->arm_l2_mm = (float)atof(value);
    else if (strcmp(key, "arm_l3_mm") == 0) config->arm_l3_mm = (float)atof(value);
    else if (strcmp(key, "arm_l4_mm") == 0) config->arm_l4_mm = (float)atof(value);
    else if (strcmp(key, "gripper_center_z_offset_mm") == 0) config->gripper_center_z_offset_mm = (float)atof(value);
    else if (strcmp(key, "workspace_x_min_mm") == 0) config->workspace_x_min_mm = (float)atof(value);
    else if (strcmp(key, "workspace_x_max_mm") == 0) config->workspace_x_max_mm = (float)atof(value);
    else if (strcmp(key, "workspace_y_min_mm") == 0) config->workspace_y_min_mm = (float)atof(value);
    else if (strcmp(key, "workspace_y_max_mm") == 0) config->workspace_y_max_mm = (float)atof(value);
    else if (strcmp(key, "workspace_z_min_mm") == 0) config->workspace_z_min_mm = (float)atof(value);
    else if (strcmp(key, "workspace_z_max_mm") == 0) config->workspace_z_max_mm = (float)atof(value);
    else if (strcmp(key, "positioning_tolerance_mm") == 0) config->positioning_tolerance_mm = (float)atof(value);
    else if (strcmp(key, "target_confirm_pixel_tolerance") == 0) config->target_confirm_pixel_tolerance = (float)atof(value);
    else if (strcmp(key, "target_confirm_frames") == 0) config->target_confirm_frames = (uint8_t)atoi(value);
    else if (strcmp(key, "target_lost_timeout_ms") == 0) config->target_lost_timeout_ms = (uint32_t)strtoul(value, 0, 10);
    else if (strcmp(key, "motion_timeout_ms") == 0) config->motion_timeout_ms = (uint32_t)strtoul(value, 0, 10);
    else if (strcmp(key, "servo_min_pulse_us") == 0) config->servo_min_pulse_us = atoi(value);
    else if (strcmp(key, "servo_max_pulse_us") == 0) config->servo_max_pulse_us = atoi(value);
    else if (strcmp(key, "joint_zero_deg") == 0) (void)parse_joint_array(value, config, 0);
    else if (strcmp(key, "joint_direction") == 0) (void)parse_joint_array(value, config, 1);
    else if (strcmp(key, "joint_soft_min_deg") == 0) (void)parse_joint_array(value, config, 2);
    else if (strcmp(key, "joint_soft_max_deg") == 0) (void)parse_joint_array(value, config, 3);
}

int SimConfig_LoadIni(const char* path, SimConfig* config) {
    FILE* fp;
    char line[256];

    if (path == 0 || config == 0) return -1;
    SimConfig_SetDefaults(config);

    fp = fopen(path, "r");
    if (fp == 0) {
        return -1;
    }

    while (fgets(line, sizeof(line), fp) != 0) {
        char* key;
        char* value;
        char* eq;
        key = trim(line);
        if (*key == '\0' || *key == '#' || *key == ';') continue;
        eq = strchr(key, '=');
        if (eq == 0) continue;
        *eq = '\0';
        value = trim(eq + 1);
        key = trim(key);
        apply_key_value(config, key, value);
    }

    (void)fclose(fp);
    return 0;
}

int SimConfig_LoadDefaultIni(const char* path) {
    SimConfig loaded;
    if (SimConfig_LoadIni(path, &loaded) != 0) {
        return -1;
    }
    SimConfig_Apply(&loaded);
    return 0;
}
