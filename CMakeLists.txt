cmake_minimum_required(VERSION 3.16)
project(robotic_arm_vision_grasping_pc_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(PC_SIM "Build with PC simulation HAL stubs instead of STM32 HAL" ON)

add_library(grasp_core
    Core/Src/platform_hal_pc.c
    Core/Src/debug_log.c
    Core/Src/sim_config.c
    Core/Src/sim_metrics.c
    Core/Src/servo_control.c
    Core/Src/uart_protocol.c
    Core/Src/calibration.c
    Core/Src/kinematics.c
    Core/Src/grasp_state_machine.c
)

target_include_directories(grasp_core PUBLIC Core/Inc)

if(PC_SIM)
    target_compile_definitions(grasp_core PUBLIC PC_SIM)
endif()

if(MSVC)
    target_compile_options(grasp_core PRIVATE /W4 /WX)
else()
    target_compile_options(grasp_core PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(grasp_core PUBLIC m)
endif()

add_executable(test_core tests/test_core.c)
target_link_libraries(test_core PRIVATE grasp_core)

enable_testing()
add_test(NAME core_logic_tests COMMAND test_core)
