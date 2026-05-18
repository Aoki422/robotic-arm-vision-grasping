CC ?= gcc
BUILD_DIR := build
CFLAGS := -std=c11 -Wall -Wextra -Werror -DPC_SIM -ICore/Inc
LDFLAGS := -lm

CORE_SRCS := \
	Core/Src/platform_hal_pc.c \
	Core/Src/debug_log.c \
	Core/Src/sim_config.c \
	Core/Src/sim_metrics.c \
	Core/Src/servo_control.c \
	Core/Src/uart_protocol.c \
	Core/Src/calibration.c \
	Core/Src/kinematics.c \
	Core/Src/grasp_state_machine.c

TEST_SRCS := tests/test_core.c
TEST_BIN := $(BUILD_DIR)/test_core

.PHONY: all test clean

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_BIN): $(CORE_SRCS) $(TEST_SRCS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CORE_SRCS) $(TEST_SRCS) -o $(TEST_BIN) $(LDFLAGS)

test: $(TEST_BIN)
	$(TEST_BIN)

clean:
	rm -rf $(BUILD_DIR)
