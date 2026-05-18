#!/usr/bin/env python3
"""一键运行PC端逻辑单元测试。

该脚本只依赖PC C编译器，不依赖真实STM32、机械臂、传感器或外部硬件链路。
优先使用CMake；如果系统没有CMake，则尝试直接调用 gcc/clang/cl。
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build"


def run(cmd: list[str]) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def has(tool: str) -> bool:
    return shutil.which(tool) is not None


def run_cmake() -> bool:
    if not has("cmake"):
        return False
    run(["cmake", "-S", ".", "-B", "build", "-DPC_SIM=ON"])
    run(["cmake", "--build", "build"])
    run(["ctest", "--test-dir", "build", "--output-on-failure"])
    return True


def run_direct_compiler() -> bool:
    sources = [
        "Core/Src/platform_hal_pc.c",
        "Core/Src/debug_log.c",
        "Core/Src/sim_config.c",
        "Core/Src/sim_metrics.c",
        "Core/Src/servo_control.c",
        "Core/Src/uart_protocol.c",
        "Core/Src/calibration.c",
        "Core/Src/kinematics.c",
        "Core/Src/grasp_state_machine.c",
        "tests/test_core.c",
    ]
    BUILD.mkdir(exist_ok=True)

    if has("gcc"):
        out = str(BUILD / "test_core.exe")
        run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DPC_SIM", "-ICore/Inc", *sources, "-o", out, "-lm"])
        run([out])
        return True

    if has("clang"):
        out = str(BUILD / "test_core.exe")
        run(["clang", "-std=c11", "-Wall", "-Wextra", "-Werror", "-DPC_SIM", "-ICore/Inc", *sources, "-o", out, "-lm"])
        run([out])
        return True

    if has("cl"):
        out = str(BUILD / "test_core.exe")
        run(["cl", "/nologo", "/W4", "/WX", "/DPC_SIM", "/ICore/Inc", *sources, f"/Fe:{out}"])
        run([out])
        return True

    return False


def main() -> int:
    try:
        if run_cmake() or run_direct_compiler():
            return 0
    except subprocess.CalledProcessError as exc:
        return exc.returncode

    print("ERROR: 未找到 cmake/gcc/clang/cl。请安装任意一种PC C编译工具链后重试。", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
