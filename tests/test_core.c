#include "calibration.h"
#include "grasp_state_machine.h"
#include "kinematics.h"
#include "platform_hal.h"
#include "servo_control.h"
#include "sim_config.h"
#include "sim_metrics.h"
#include "uart_protocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_true(int condition, const char* message) {
    if (!condition) {
        fprintf(stderr, "[FAIL] %s\n", message);
        exit(1);
    }
}

static void require_near(float actual, float expected, float tolerance, const char* message) {
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "[FAIL] %s actual=%.4f expected=%.4f tolerance=%.4f\n",
                message, actual, expected, tolerance);
        exit(1);
    }
}

static void test_uart_crc_and_frame_parser(void) {
    uint8_t frame[FRAME_LENGTH] = {FRAME_HEAD, COLOR_RED, 0x40, 0x01, 0x80, 0x02, 0x00, FRAME_TAIL};
    uint8_t color = 0;
    int16_t px = 0;
    int16_t py = 0;

    UARTProtocol_Init();
    frame[FRAME_IDX_CRC] = CRC8_Calc(frame, FRAME_DATA_LEN);

    for (int i = 0; i < FRAME_LENGTH; i++) {
        (void)UART_FeedByte(frame[i]);
    }

    require_true(UART_ReadFrame(&color, &px, &py) == 1u, "valid UART frame should be readable");
    require_true(color == COLOR_RED, "UART color parsed");
    require_true(px == 0x0140, "UART x parsed");
    require_true(py == 0x0280, "UART y parsed");

    UARTProtocol_Init();
    frame[FRAME_IDX_CRC] ^= 0x5Au;
    for (int i = 0; i < FRAME_LENGTH; i++) {
        (void)UART_FeedByte(frame[i]);
    }
    require_true(UART_ReadFrame(&color, &px, &py) == 0u, "CRC error frame must be rejected");

    UARTProtocol_Init();
    frame[FRAME_IDX_CRC] ^= 0x5Au;
    frame[FRAME_IDX_TAIL] = 0x00u;
    for (int i = 0; i < FRAME_LENGTH; i++) {
        (void)UART_FeedByte(frame[i]);
    }
    require_true(UART_ReadFrame(&color, &px, &py) == 0u, "bad tail frame must be rejected");
}

static void test_calibration_simulated_nine_points(void) {
    AffineMatrix truth = {0.45f, 0.02f, -70.0f, -0.01f, 0.48f, -120.0f};
    AffineMatrix solved;
    CalibPoint points[CALIB_SIM_POINT_COUNT];
    float err;

    Calib_GenerateSimulatedPoints(&truth, points);
    require_true(Calib_SolveAffine(points, CALIB_SIM_POINT_COUNT, &solved) == 0, "affine solve should succeed");

    require_near(solved.a, truth.a, 0.001f, "affine a");
    require_near(solved.b, truth.b, 0.001f, "affine b");
    require_near(solved.c, truth.c, 0.010f, "affine c");
    require_near(solved.d, truth.d, 0.001f, "affine d");
    require_near(solved.e, truth.e, 0.001f, "affine e");
    require_near(solved.f, truth.f, 0.010f, "affine f");

    err = Calib_ComputeMaxError(points, CALIB_SIM_POINT_COUNT, &solved);
    require_true(Metrics_IsPositionWithinTolerance(err), "reprojection error within configured tolerance");
}

static void test_ik_boundaries_and_forward_check(void) {
    int angles[6];
    KinematicPose pose;
    int status;

    IK_SetConfig(0);
    status = IK_Solve(180.0f, 0.0f, 80.0f, 0.0f, angles);
    require_true(status == IK_OK, "reachable IK target should solve");
    require_true(IK_Forward(angles, &pose) == IK_OK, "forward kinematics should succeed");
    require_near(pose.x, 180.0f, 3.0f, "forward x close to target");
    require_near(pose.y, 0.0f, 3.0f, "forward y close to target");
    require_near(pose.z, 80.0f, 3.0f, "forward z close to target");

    status = IK_Solve(500.0f, 0.0f, 80.0f, 0.0f, angles);
    require_true(status == IK_ERR_WORKSPACE, "out-of-workspace target must be rejected");

    status = IK_Solve(45.0f, 0.0f, 5.0f, 0.0f, angles);
    require_true(status < 0, "unsafe near-base target must be rejected");
}

static void feed_frame(uint8_t color, int16_t px, int16_t py) {
    uint8_t frame[FRAME_LENGTH];
    frame[FRAME_IDX_HEAD] = FRAME_HEAD;
    frame[FRAME_IDX_COLOR] = color;
    frame[FRAME_IDX_XL] = (uint8_t)(px & 0xFF);
    frame[FRAME_IDX_XH] = (uint8_t)((px >> 8) & 0xFF);
    frame[FRAME_IDX_YL] = (uint8_t)(py & 0xFF);
    frame[FRAME_IDX_YH] = (uint8_t)((py >> 8) & 0xFF);
    frame[FRAME_IDX_CRC] = CRC8_Calc(frame, FRAME_DATA_LEN);
    frame[FRAME_IDX_TAIL] = FRAME_TAIL;

    for (int i = 0; i < FRAME_LENGTH; i++) {
        (void)UART_FeedByte(frame[i]);
    }
}

static void run_ticks(uint32_t total_ms, uint32_t step_ms) {
    uint32_t elapsed = 0;
    while (elapsed <= total_ms) {
        Servo_Tick();
        Grasp_Tick();
        HAL_AdvanceTick(step_ms);
        elapsed += step_ms;
    }
}

static void test_grasp_state_machine_pc_smoke(void) {
    AffineMatrix mat = {0.50f, 0.00f, -80.0f, 0.00f, 0.50f, -130.0f};
    SimMetrics metrics;

    HAL_SetTick(0);
    UARTProtocol_Init();
    Metrics_Reset();
    Calib_Init(&mat);
    Servo_Init();
    Grasp_Init();

    feed_frame(COLOR_RED, 520, 260); /* world=(180,0), reachable */
    Grasp_Tick();
    feed_frame(COLOR_RED, 522, 259);
    Grasp_Tick();
    feed_frame(COLOR_RED, 521, 261);
    Grasp_Tick();

    run_ticks(6000, 20);

    Metrics_Get(&metrics);
    require_true(metrics.confirmed_targets == 1u, "state machine should confirm stable target");
    require_true(metrics.grasp_attempts == 1u, "state machine should record one attempt");
    require_true(metrics.grasp_success == 1u, "state machine should complete one logic grasp");
    require_true(Grasp_GetState() == GRASP_IDLE, "state machine should return to idle");
}

int main(void) {
    SimConfig config;
    SimConfig_SetDefaults(&config);
    SimConfig_Apply(&config);

    test_uart_crc_and_frame_parser();
    test_calibration_simulated_nine_points();
    test_ik_boundaries_and_forward_check();
    test_grasp_state_machine_pc_smoke();

    puts("[PASS] all PC logic tests passed");
    return 0;
}
