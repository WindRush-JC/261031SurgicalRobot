# 项目临时笔记

> 用于暂存尚未归入正式设计、待办或阶段记录的技术复盘、验证过程和中间结论。已经整理到其他正式文档的内容不在此重复保留。



## demo流程

找到u_lapar_control模块，导入ULaparRobotControl机器人控制主类、WORKMODE_AUTO_SURGICAL工作模式常量（自动模式？）

创建控制对象，连接机器人，无限循环直到能返回实时状态

获得1、3机械臂的末端工具位姿
获得1、3机械臂的关节状态（夹爪状态）
为了获得末端关节坐标和夹爪状态

“先唤醒刹车，再切到双器械模式。”不懂

把pose1拉平
1机械臂，X方向，往复走
每步 10/40=0.25 毫米，0.05s

关闭连接



## 底层通信与控制接口
u_lapar_control.py

工作模式：
WORKMODE_AUTO_SURGICAL = 0
WORKMODE_AUTO_ENDO = 1

机械臂：
SlaveArm1 = 1
SlaveArm2 = 2
SlaveArm3 = 3
SlaveArm4 = 4

运动模式(与 EnumMotionMode 对齐)
MOTION_MOVEJ_BLOCK = 0
MOTION_MOVEJ_UNBLOCK = 1

set_workmode()  # 工作模式

class MSGHEAD:
    """报文头结构，打包/解包与采集程序保持一致。"""

class MasterSlaveMappingInfoReport:
    """从实时上报中解析 MasterSlaveMappingInfoReport。"""

class ReportInfo:
    """实时上报的整体信息(QTool + TransTool + TransFlex + 两个 MasterSlaveMappingInfo)。"""
    QTool1，QTool2……：11维
    TransTool1……：12维
    TransFlex1……：12维

class MoveDoneInfo:
    """MoveJ 完成结果，对应 MoveDoneInfo 结构。"""


### class ULaparRobotControl:
    """
    机器人控制主类。

    /** 控制类
     * @param ip 机器人控制服务器 IP，默认 172.31.0.151
     * @param port 机器人控制服务器端口，默认 16010
     */
    """

is_connected(self) -> bool:连接状态

#### 高层接口封装：
def set_workmode(self, mode: int) -> None:工作模式

def move_arm2_endo(self, transform: List[double], clip: double) -> None:自动持械: 2 号臂持镜

控制1、3号臂的工具位姿
def move_dual_inst_arm(
        self,
        transform_arm1: List[double],
        clip_arm1: double,
        transform_arm3: List[double],
        clip_arm3: double,
    ) -> None:自动持械: 1 号臂 + 3 号臂


def move_corobot_arm(self, transform_arm4: List[double], clip_arm4: double) -> None:持械臂(4号)运动：仅发送 SET_TRACKING_POS，不切换工作模式

def move_corobot_double_arm(
        self,
        transform_arm3: List[double],
        clip_arm3: double,
        transform_arm4: List[double],
        clip_arm4: double,
    ) -> None:持械臂(3号+4号)运动：仅发送 SET_TRACKING_POS，不切换工作模式

def clear_tracking(self) -> None:

控制某个机械臂的关节角度
def movej_arm(
        self,
        arm_id: int,
        joints: List[double],
        cycle: double = 0.01,
        motion_mode: int = MOTION_MOVEJ_BLOCK,
        timeout: float = 5.0,
    ) -> Optional[MoveDoneInfo]:关节空间 MoveJ 运动

控制某个机械臂的工具位姿
def move_tool(
        self,
        transforms: List[Optional[List[double]]],
        clips: List[Optional[double]],
        mapping_index: List[int],
    ) -> None:工具空间运动(全体 + 映射)

def lock(self, arm_id: int) -> None:
def unlock(self, arm_id: int) -> None:
def lockAll(self) -> None:
        self._send_simple_arm_cmd(self.CMD_LOCK_ALL, None)
def unlockAll(self) -> None:
        self._send_simple_arm_cmd(self.CMD_UNLOCK_ALL, None)


#### 数据获取接口：
def get_current_joint(self, arm_id: int) -> Optional[List[double]]:获取当前关节角
def get_current_toolpose(self, arm_id: int) -> Optional[List[double]]:获取当前工具位姿
def get_current_flexpose(self, arm_id: int) -> Optional[List[double]]:获取当前柔性段位姿
def get_current_master(self) -> Optional[tuple[MasterSlaveMappingInfoReport, MasterSlaveMappingInfoReport]]:


### 数据说明
关节角数组：11维，最后一维为夹爪

TransTool工具位姿：12维，3个位置 + 9个旋转矩阵元素
[
    x, y, z,
    r11, r12, r13,
    r21, r22, r23,
    r31, r32, r33
]

TransFlex柔性段位姿？

### 工作模式
WORKMODE_AUTO_SURGICAL = 0，双器械，用这个就好
WORKMODE_AUTO_ENDO = 1


## 机器人控制流程

连接

获取位姿，关节角度，力

设置工作状态，设置锁

运动控制

夹取控制

关闭连接


## 问题
4只臂有什么区别?

工具位姿TransTool：
顺序是这样吗？
[
    x, y, z,
    r11, r12, r13,
    r21, r22, r23,
    r31, r32, r33
]
坐标系映射关系是什么，坐标系是怎么定义的（原点、朝向）？
get_current_toolpose 拿到的位姿矩阵是什么坐标系下的？
数据的范围单位是什么？
数据采集率是多少？

关节角数组：
11维分别是什么？单位、范围是什么？

柔性段位姿是什么？

存在力反馈吗？

锁状态是什么

## 261103现场测试：
确认 Python 程序能够可靠读取机器人状态，并按照预期控制机器人完成基础运动。


##        