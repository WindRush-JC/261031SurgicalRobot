import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from u_lapar_control import ULaparRobotControl
from pathlib import Path
from datetime import datetime
import time
import math

LOG_PATH = Path(__file__).resolve().parent.parent / "data" / f"data_{datetime.now():%Y%m%d_%H%M%S}.log"

def get_tool_state(ctrl, arm_id):
    """
    获取并整理指定机械臂的工具位姿。

    返回字典：
        {
            "position": np.ndarray,      # [x, y, z]
            "rotation_matrix": np.ndarray,  # 3×3旋转矩阵
            "transform": np.ndarray,     # 4×4齐次变换矩阵
            "euler_xyz": np.ndarray      # XYZ欧拉角（单位：deg）
        }

    获取失败或数据异常时返回 None。
    """

    # 1. 获取原始12维工具位姿
    pose = ctrl.get_current_toolpose(arm_id)

    # 2. 基础有效性检查
    if pose is None:
        print(f"机械臂{arm_id}：未收到工具位姿数据")
        return None

    pose = np.asarray(pose, dtype=float)

    # 检查维度
    if pose.shape != (12,):
        print(f"机械臂{arm_id}：工具位姿数据维度异常")
        return None

    # 检查 NaN 和 Inf
    if not np.all(np.isfinite(pose)):
        print(f"机械臂{arm_id}：工具位姿数据存在非法数值")
        return None

    # 3. 提取位置
    position = pose[:3]

    # 4. 提取3×3旋转矩阵
    rotation_matrix = pose[3:].reshape(3, 3)
    # 检查旋转矩阵正交性
    if not np.allclose(
        rotation_matrix.T @ rotation_matrix,
        np.eye(3),
        atol=1e-5
    ):
        print(f"机械臂{arm_id}：旋转矩阵不满足正交性")
        return None
    # 检查行列式
    if not np.isclose(
        np.linalg.det(rotation_matrix),
        1.0,
        atol=1e-5
    ):
        print(f"机械臂{arm_id}：旋转矩阵行列式异常")
        return None

    # 5. 构建4×4齐次变换矩阵
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = position

    # 6. 旋转矩阵 → 欧拉角
    try:
        euler_xyz = Rotation.from_matrix(
            rotation_matrix
        ).as_euler("xyz", degrees=True)

    except ValueError:
        print(f"机械臂{arm_id}：无法转换为欧拉角")
        return None

    print(f"机械臂{arm_id}：工具位姿数据正常")

    return {
        "position": position,
        "rotation_matrix": rotation_matrix,
        "transform": transform,
        "euler_xyz": euler_xyz,
    }

def pack(s):
    return np.concatenate([s["position"], s["rotation_matrix"].reshape(-1)])

def _write(file, text, to_terminal):
    """每行加时间戳写文件；to_terminal=True 时同时打到终端。"""
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    for line in text.splitlines() or [""]:
        out = f"[{ts}] {line}"
        file.write(out + "\n")
        if to_terminal:
            print(out)
    file.flush()


def log_tool_state(file, tag, state, *, to_terminal=False):
    """
    把 get_tool_state 的返回值写入 file。
    默认只写文件；to_terminal=True 时同时打印到终端。
    """
    if state is None:
        _write(file, f"{tag}: 无有效工具位姿数据", to_terminal)
        return

    def fmt(arr):
        return np.array2string(np.asarray(arr), precision=6, suppress_small=True)

    _write(file, f"===== {tag} =====", to_terminal)
    _write(file, f"position: {fmt(state['position'])}", to_terminal)
    _write(file, f"rotation_matrix:\n{fmt(state['rotation_matrix'])}", to_terminal)
    _write(file, f"transform:\n{fmt(state['transform'])}", to_terminal)
    _write(file, f"euler_xyz(deg): {fmt(state['euler_xyz'])}", to_terminal)


def move_dual_to_pose(
    ctrl,
    target_transform_arm1,
    target_clip_arm1,
    target_transform_arm3,
    target_clip_arm3,
    speed=1.0,
):
    """
    控制 1 号臂和 3 号臂同步运动到目标位姿。

    参数
    ----
    ctrl :
        ULaparRobotControl 控制对象。

    target_transform_arm1 :
        1号臂目标工具位姿，12维：
        [x, y, z, r11, r12, ..., r33]

    target_clip_arm1 : float
        1号臂目标夹爪状态。

    target_transform_arm3 :
        3号臂目标工具位姿，12维：
        [x, y, z, r11, r12, ..., r33]

    target_clip_arm3 :
        3号臂目标夹爪状态。

    speed : float
        相对于基准速度的倍率。
        speed=1.0 对应 Demo 的基准速度。

    说明
    ----
    位置使用线性插值。
    姿态使用四元数 SLERP 插值。
    两条机械臂使用相同步数，因此同步完成运动。
    """

    # ==============================
    # 1. 参数检查
    # ==============================

    if speed <= 0:
        raise ValueError("speed 必须大于 0")

    # 妈的好谨慎
    target_transform_arm1 = np.asarray(
        target_transform_arm1,
        dtype=float
    )

    target_transform_arm3 = np.asarray(
        target_transform_arm3,
        dtype=float
    )

    if target_transform_arm1.shape != (12,):
        raise ValueError("1号臂目标位姿必须是12维")

    if target_transform_arm3.shape != (12,):
        raise ValueError("3号臂目标位姿必须是12维")

    if not np.all(np.isfinite(target_transform_arm1)):
        raise ValueError("1号臂目标位姿存在非法数值")

    if not np.all(np.isfinite(target_transform_arm3)):
        raise ValueError("3号臂目标位姿存在非法数值")

    # ==============================
    # 2. 获取当前工具位姿
    # ==============================

    current_transform_arm1 = ctrl.get_current_toolpose(1)
    current_transform_arm3 = ctrl.get_current_toolpose(3)

    if current_transform_arm1 is None:
        raise RuntimeError("未收到1号臂当前工具位姿")

    if current_transform_arm3 is None:
        raise RuntimeError("未收到3号臂当前工具位姿")

    current_transform_arm1 = np.asarray(
        current_transform_arm1,
        dtype=float
    )

    current_transform_arm3 = np.asarray(
        current_transform_arm3,
        dtype=float
    )

    # ==============================
    # 3. 提取位置和旋转矩阵
    # ==============================

    current_position_arm1 = current_transform_arm1[:3]
    current_position_arm3 = current_transform_arm3[:3]

    target_position_arm1 = target_transform_arm1[:3]
    target_position_arm3 = target_transform_arm3[:3]

    current_rotation_arm1 = (
        current_transform_arm1[3:].reshape(3, 3)
    )

    current_rotation_arm3 = (
        current_transform_arm3[3:].reshape(3, 3)
    )

    target_rotation_arm1 = (
        target_transform_arm1[3:].reshape(3, 3)
    )

    target_rotation_arm3 = (
        target_transform_arm3[3:].reshape(3, 3)
    )

    # ==============================
    # 4. 计算需要的同步步数
    # ==============================

    BASE_STEP_MM = 0.25 # 每步移动的基准距离，单位：毫米
    CYCLE = 0.05    # 每步发送的周期，单位：秒
    BASE_ROTATION_STEP_DEG = 1.0    # 每步旋转的基准角度，单位：度

    distance_arm1 = np.linalg.norm(
        target_position_arm1 - current_position_arm1
    )

    distance_arm3 = np.linalg.norm(
        target_position_arm3 - current_position_arm3
    )

    max_distance = max(
        distance_arm1,
        distance_arm3
    )

    step_distance = BASE_STEP_MM * speed

    position_steps = max(
        1,
        math.ceil(max_distance / step_distance)
    )   # 步数

    # 计算旋转角度
    rotation_change_arm1 = (
        Rotation.from_matrix(target_rotation_arm1)
        * Rotation.from_matrix(current_rotation_arm1).inv()
    )

    rotation_change_arm3 = (
        Rotation.from_matrix(target_rotation_arm3)
        * Rotation.from_matrix(current_rotation_arm3).inv()
    )

    rotation_angle_arm1 = np.degrees(
        rotation_change_arm1.magnitude()
    )

    rotation_angle_arm3 = np.degrees(
        rotation_change_arm3.magnitude()
    )

    max_rotation_angle = max(
        rotation_angle_arm1,
        rotation_angle_arm3
    )

    rotation_step_deg = (
        BASE_ROTATION_STEP_DEG * speed
    )

    rotation_steps = math.ceil(
        max_rotation_angle / rotation_step_deg
    )

    steps = max(
        1,
        position_steps,
        rotation_steps,
    )

    # ==============================
    # 5. 创建姿态插值器
    # ==============================

    # 旋转矩阵 → 四元数
    rotations_arm1 = Rotation.from_matrix([
        current_rotation_arm1,
        target_rotation_arm1
    ])

    rotations_arm3 = Rotation.from_matrix([
        current_rotation_arm3,
        target_rotation_arm3
    ])

    slerp_arm1 = Slerp(
        [0.0, 1.0],
        rotations_arm1
    )

    slerp_arm3 = Slerp(
        [0.0, 1.0],
        rotations_arm3
    )

    # ==============================
    # 6. 逐步生成轨迹并发送
    # ==============================

    for i in range(1, steps + 1):

        t = i / steps

        # ---- 位置线性插值 ----

        position_arm1 = (
            (1 - t) * current_position_arm1
            + t * target_position_arm1
        )

        position_arm3 = (
            (1 - t) * current_position_arm3
            + t * target_position_arm3
        )

        # ---- 姿态 SLERP ----

        rotation_arm1 = (
            slerp_arm1([t])
            .as_matrix()[0]
        )

        rotation_arm3 = (
            slerp_arm3([t])
            .as_matrix()[0]
        )

        # ---- 重新组合12维 Transform ----

        transform_arm1 = np.concatenate([
            position_arm1,
            rotation_arm1.reshape(-1)
        ])

        transform_arm3 = np.concatenate([
            position_arm3,
            rotation_arm3.reshape(-1)
        ])

        # ---- 发送跟踪目标 ----

        ctrl.move_dual_inst_arm(
            transform_arm1.tolist(),
            target_clip_arm1,
            transform_arm3.tolist(),
            target_clip_arm3,
        )

        time.sleep(CYCLE)

    print(
        f"双臂运动完成："
        f"共 {steps} 步，"
        f"速度倍率 {speed}"
    )