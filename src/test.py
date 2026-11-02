#!/usr/bin/env python3

import sys
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from myrobot import (
    LOG_PATH,
    get_tool_state,
    log_tool_state,
    move_dual_to_pose,
    pack
)

# sys.path.insert(0, r"/home/user/project/lerobot_robot_lapar")   # 接口包路径
from u_lapar_control import ULaparRobotControl, WORKMODE_AUTO_SURGICAL

LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
log_file = open(LOG_PATH, "w", 
encoding="utf-8")
timeout = 5.0
start_time = time.time()


try:
    # 创建控制对象
    ctrl = ULaparRobotControl()

    # 建立通信
    ctrl.connect()

    # 确定能收到东西
    while ctrl.get_current_toolpose(1) is None:
        if time.time() - start_time > timeout:
            raise TimeoutError("等待机器人实时状态超时")
        time.sleep(0.01)
    print("已建立通信，并收到实时状态")


    # 检查收到的信息
    # 获取当前工具位姿
    tool_state1 = get_tool_state(ctrl, 1)
    tool_state2 = get_tool_state(ctrl, 2)
    tool_state3 = get_tool_state(ctrl, 3)
    tool_state4 = get_tool_state(ctrl, 4)

    log_tool_state(log_file, "tool_state1", tool_state1, to_terminal=True)  

    # 获取当前关节角度
    joint1 = ctrl.get_current_joint(1)
    joint2 = ctrl.get_current_joint(2)
    joint3 = ctrl.get_current_joint(3)
    joint4 = ctrl.get_current_joint(4)

    clip1 = joint1[-1]
    clip2 = joint2[-1]
    clip3 = joint3[-1]
    clip4 = joint4[-1]

    # 打印当前工具位姿和关节角度
    print("夹爪角度：", clip1)
    print("夹爪角度：", clip2)
    print("夹爪角度：", clip3)
    print("夹爪角度：", clip4)
    print("关节角度：", joint1)

    # 柔性段位姿
    flex_state1 = ctrl.get_current_flexpose(1)
    print("柔性段位姿：", flex_state1)

    # 力信息



    # 机器人状态
    ctrl.lockAll()
    time.sleep(1.0)
    ctrl.unlockAll()
    time.sleep(1.0)
    ctrl.set_workmode(WORKMODE_AUTO_SURGICAL)


    # 运动控制

    # 当前位姿（12维）
    p1 = pack(tool_state1)
    p3 = pack(tool_state3)

    # 1. 1号臂 XYZ 各 +1mm，3号臂不动
    t1 = p1.copy(); t1[:3] += 1.0
    move_dual_to_pose(ctrl, t1, clip1, p3, clip3)
    print("1号臂移动到", t1[:3], "3号臂保持不动", p3[:3])
    time.sleep(0.5)

    # 2. 3号臂 XYZ 各 +1mm，1号臂不动
    t3 = p3.copy(); t3[:3] += 1.0
    move_dual_to_pose(ctrl, t1, clip1, t3, clip3)
    print("3号臂移动到", t3[:3], "1号臂保持不动", t1[:3])
    time.sleep(0.5)

    # 3. 两臂绕自身 z 轴转 10°
    dR = Rotation.from_euler("z", 10, degrees=True).as_matrix()
    t1 = pack(tool_state1); t1[3:] = (tool_state1["rotation_matrix"] @ dR).reshape(-1)
    t3 = pack(tool_state3); t3[3:] = (tool_state3["rotation_matrix"] @ dR).reshape(-1)
    move_dual_to_pose(ctrl, t1, clip1, t3, clip3)
    print("两臂绕自身 z 轴转 10°")
    time.sleep(0.5)


    # 夹爪控制


# 关闭连接

finally:
    print("正在执行安全清理")

    try:
        ctrl.clear_tracking()
    except Exception:
        pass

    try:
        ctrl.lockAll()
    except Exception:
        pass

    try:
        ctrl.disconnect()
    except Exception:
        pass

    log_file.close()