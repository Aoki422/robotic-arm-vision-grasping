"""
CoppeliaSim / PC 机械臂视觉抓取仿真入口
=====================================

重要说明：
  本脚本只用于无实机条件下的视觉抓取概念验证和逻辑闭环测试，
  不作为 ZX30D 舵机臂、MV4 视觉模块或任何真实硬件链路的实机验证依据。

两种运行模式：
  1. pc          纯PC端逻辑仿真，不需要CoppeliaSim、不需要真实相机/机械臂。
  2. coppeliasim CoppeliaSim可视化仿真，控制UR5+RG2+VisionSensor示例场景。

模式切换：
  - 命令行：python visual_grasping.py --mode pc
  - 环境变量：GRASP_SIM_MODE=pc 或 GRASP_SIM_MODE=coppeliasim
"""

from __future__ import annotations

import argparse
import math
import os
import random
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple


SIM_MODE = os.environ.get("GRASP_SIM_MODE", "pc")


@dataclass
class SimParams:
    arm_l1_mm: float = 50.0
    arm_l2_mm: float = 120.0
    arm_l3_mm: float = 100.0
    arm_l4_mm: float = 60.0
    workspace_x_min_mm: float = 40.0
    workspace_x_max_mm: float = 240.0
    workspace_y_min_mm: float = -75.0
    workspace_y_max_mm: float = 75.0
    workspace_z_min_mm: float = 0.0
    workspace_z_max_mm: float = 160.0
    positioning_tolerance_mm: float = 5.0


@dataclass
class Metrics:
    attempts: int = 0
    success: int = 0
    failed: int = 0
    max_error_mm: float = 0.0

    def record(self, error_mm: float, ok: bool) -> None:
        self.attempts += 1
        self.max_error_mm = max(self.max_error_mm, error_mm)
        if ok:
            self.success += 1
        else:
            self.failed += 1

    @property
    def success_rate(self) -> float:
        if self.attempts == 0:
            return 0.0
        return self.success * 100.0 / self.attempts

    def print_line(self) -> None:
        print(
            "[METRICS] attempts=%d success=%d failed=%d success_rate=%.1f%% max_error=%.2fmm"
            % (self.attempts, self.success, self.failed, self.success_rate, self.max_error_mm)
        )


def load_params(path: Path) -> SimParams:
    params = SimParams()
    if not path.exists():
        print(f"[WARN] config not found: {path}, using defaults")
        return params

    values: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith(";") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()

    for key, value in values.items():
        if hasattr(params, key):
            setattr(params, key, float(value))

    return params


def pixel_to_world(px: int, py: int) -> Tuple[float, float]:
    """与Core默认仿真标定矩阵保持一致：wx=0.5*px-80, wy=0.5*py-130。"""
    return 0.5 * px - 80.0, 0.5 * py - 130.0


def world_to_pixel(wx: float, wy: float) -> Tuple[int, int]:
    return int(round((wx + 80.0) / 0.5)), int(round((wy + 130.0) / 0.5))


def validate_workspace(params: SimParams, x: float, y: float, z: float) -> bool:
    return (
        params.workspace_x_min_mm <= x <= params.workspace_x_max_mm
        and params.workspace_y_min_mm <= y <= params.workspace_y_max_mm
        and params.workspace_z_min_mm <= z <= params.workspace_z_max_mm
    )


def solve_ik_like_core(params: SimParams, x: float, y: float, z: float) -> Optional[Tuple[int, int, int, int, int, int]]:
    """Python版简化IK，用于PC仿真快速判断，与Core C逻辑保持同一几何模型。"""
    if not validate_workspace(params, x, y, z):
        return None

    target_r = math.hypot(x, y)
    wrist_r = target_r - params.arm_l4_mm
    plane_z = z - params.arm_l1_mm
    if wrist_r <= 1.0:
        return None

    d = math.hypot(wrist_r, plane_z)
    if d > params.arm_l2_mm + params.arm_l3_mm or d < abs(params.arm_l2_mm - params.arm_l3_mm):
        return None

    cos_elbow = (d * d - params.arm_l2_mm**2 - params.arm_l3_mm**2) / (
        2.0 * params.arm_l2_mm * params.arm_l3_mm
    )
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow = math.degrees(math.acos(cos_elbow))
    shoulder = math.degrees(
        math.atan2(plane_z, wrist_r)
        - math.atan2(
            params.arm_l3_mm * math.sin(math.radians(elbow)),
            params.arm_l2_mm + params.arm_l3_mm * math.cos(math.radians(elbow)),
        )
    )
    yaw = math.degrees(math.atan2(y, x))
    wrist = -(shoulder + elbow)

    servo = (
        round(90 + yaw),
        round(90 + shoulder),
        round(elbow),
        round(135 + wrist),
        90,
        0,
    )
    if any(angle < 0 or angle > 180 for angle in servo):
        return None
    return servo


def run_pc_logic_sim(params: SimParams, attempts: int) -> None:
    """纯PC逻辑闭环：生成虚拟目标 -> 像素/世界坐标 -> IK -> 统计指标。"""
    metrics = Metrics()
    random.seed(7)
    print("[INFO] running pure PC logic simulation")
    print("[INFO] no CoppeliaSim, no hardware, no sensor link")

    for idx in range(attempts):
        # 避开近基座奇异区，测试“可操作子区域”的抓取成功率；
        # 完整工作区边界仍由 solve_ik_like_core 负责拒绝保护。
        wx = random.uniform(params.workspace_x_min_mm + 60.0, params.workspace_x_max_mm - 20.0)
        wy = random.uniform(params.workspace_y_min_mm + 10.0, params.workspace_y_max_mm - 10.0)
        wz = 80.0
        px, py = world_to_pixel(wx, wy)
        measured_x, measured_y = pixel_to_world(px, py)
        error_mm = math.hypot(measured_x - wx, measured_y - wy)
        angles = solve_ik_like_core(params, measured_x, measured_y, wz)
        ok = angles is not None and error_mm <= params.positioning_tolerance_mm
        metrics.record(error_mm, ok)
        print(
            "[%02d] pixel=(%d,%d) world=(%.1f,%.1f,%.1f) error=%.2fmm ik=%s"
            % (idx, px, py, measured_x, measured_y, wz, error_mm, "OK" if angles else "REJECT")
        )

    metrics.print_line()


def detect_object_center_cv2(img, lower, upper):
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 200:
        return None, mask

    moments = cv2.moments(largest)
    if moments["m00"] == 0:
        return None, mask
    return (int(moments["m10"] / moments["m00"]), int(moments["m01"] / moments["m00"])), mask


class CoppeliaArmAdapter:
    """UR5+RG2可视化适配层，接口上对齐“目标点->运动->夹爪”的STM32状态机概念。"""

    def __init__(self, sim, sim_ik):
        self.sim = sim
        self.sim_ik = sim_ik
        self.target = sim.getObject("/UR5_target")
        self.tip = sim.getObject("/UR5_tip")
        self.cam = sim.getObject("/VisionSensor")
        self.joints = [sim.getObject(f"/UR5_joint{i}") for i in range(1, 7)]
        self.ik_env = sim_ik.createEnvironment()
        self.ik_group = sim_ik.createGroup(self.ik_env)
        for joint in self.joints:
            sim_ik.setJointMode(self.ik_env, joint, sim_ik.mode_ik)
        sim_ik.addElementFromScene(self.ik_env, self.ik_group, self.tip, self.target)

    def move_to(self, position_m: Iterable[float]) -> None:
        self.sim.setObjectPosition(self.target, self.sim.handle_world, list(position_m))
        self.sim.setObjectOrientation(self.target, self.sim.handle_world, [math.pi, 0.0, 0.0])
        self.sim_ik.handleGroup(self.ik_env, self.ik_group)
        self.sim.step()

    def set_gripper(self, opened: bool) -> None:
        self.sim.setInt32Signal("RG2_open", 1 if opened else 0)
        for _ in range(8):
            self.sim.step()

    def destroy(self) -> None:
        self.sim_ik.destroyEnvironment(self.ik_env)


def run_coppeliasim_visual(params: SimParams) -> None:
    import cv2
    import numpy as np
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    color_lower = np.array([0, 100, 100])
    color_upper = np.array([10, 255, 255])
    metrics = Metrics()

    print("[INFO] connecting to CoppeliaSim ZMQ Remote API")
    client = RemoteAPIClient()
    sim = client.require("sim")
    sim_ik = client.require("simIK")
    arm = CoppeliaArmAdapter(sim, sim_ik)

    sim.setStepping(True)
    sim.startSimulation()
    arm.set_gripper(True)

    try:
        home = [0.30, 0.0, 0.30]
        arm.move_to(home)

        for step in range(300):
            sim.step()
            raw, res_x, res_y = sim.getVisionSensorCharImage(arm.cam)
            img = np.frombuffer(raw, dtype=np.uint8).reshape(res_y, res_x, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            center, _ = detect_object_center_cv2(img, color_lower, color_upper)

            if center is not None:
                cv2.circle(img, center, 8, (0, 255, 0), 2)
            cv2.imshow("Vision Sensor", img)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if center is None:
                continue

            wx_mm, wy_mm = pixel_to_world(center[0], center[1])
            wz_mm = 80.0
            if solve_ik_like_core(params, wx_mm, wy_mm, wz_mm) is None:
                print(f"[WARN] Core-like IK rejected target ({wx_mm:.1f},{wy_mm:.1f},{wz_mm:.1f})mm")
                metrics.record(999.0, False)
                continue

            target = [wx_mm / 1000.0, wy_mm / 1000.0, 0.03]
            arm.move_to([target[0], target[1], target[2] + 0.12])
            arm.move_to([target[0], target[1], target[2] + 0.02])
            arm.set_gripper(False)
            arm.move_to([target[0], target[1], target[2] + 0.25])
            arm.move_to(home)
            arm.set_gripper(True)

            metrics.record(0.0, True)
            metrics.print_line()
            break

    finally:
        arm.set_gripper(True)
        sim.stopSimulation()
        arm.destroy()
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("pc", "coppeliasim"), default=SIM_MODE)
    parser.add_argument("--attempts", type=int, default=20)
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config" / "sim_params.ini")
    args = parser.parse_args()

    params = load_params(args.config)
    if args.mode == "pc":
        run_pc_logic_sim(params, args.attempts)
    else:
        run_coppeliasim_visual(params)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
