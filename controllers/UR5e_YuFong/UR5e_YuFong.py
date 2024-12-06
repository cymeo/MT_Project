from controller import Robot, Supervisor
import math

# 初始化 Webots 機器人/ Initialising WeBots
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# UR5e 關節名稱 / joint names
JOINT_NAMES = [
    "shoulder_pan_joint",  # Joint 1
    "shoulder_lift_joint",  # Joint 2
    "elbow_joint",  # Joint 3
    "wrist_1_joint",  # Joint 4
    "wrist_2_joint",  # Joint 5
    "wrist_3_joint"  # Joint 6
]

# 初始化關節設備 initialise joints
joints = [robot.getDevice(name) for name in JOINT_NAMES]
for joint in joints:
    joint.setVelocity(1.0)  # 設置關節的最大速度（控制運動速度）

# UR5e 的 DH 參數（單位：米和弧度） Robot Parameter in meters and rad 
L1 = 0.1625  # 基座到肩關節的高度 Base to shoulder length 
L2 = 0.425   # 肩到肘的水平距離 shoulder to elbow length 
L3 = 0.3922  # 肘到腕的水平距離 elbow to wrist length

# 機械手臂的全局偏移（基座位置）robot base position, global koordinates 
arm_global_base = [0, 0, 0.8]  # 全局坐標中的基座位置

# 機械手臂基座的初始旋轉角度（圍繞 Z 軸 -45 度） initial base rotation , 45 ° around Z_achsis 
base_rotation = -0.785  # 弧度/ Rad 

# 設定全局目標位置（測試目標點） test target point (global koordinate ) 
global_target_position = [0.3, 0.2, 0.5] 








# 將全局目標點轉換為相對於基座的目標位置 convert global target to base frame 
def transform_to_base_frame(target, base, rotation):
    """
    將全局目標位置轉換為相對於機械手臂基座的局部位置。
    :param target: 全局目標點 [x, y, z]
    :param base: 基座的全局位置 [x, y, z]
    :param rotation: 基座的旋轉角度（弧度）
    :return: 局部目標位置 [x', y', z']
    """
    # 計算相對位置 calculate relative positions 
    dx = target[0] - base[0]
    dy = target[1] - base[1]
    dz = target[2] - base[2]

    # 應用旋轉矩陣（繞 Z 軸旋轉） rotation matrix around Z achsis 
    local_x = math.cos(-rotation) * dx + math.sin(-rotation) * dy
    local_y = -math.sin(-rotation) * dx + math.cos(-rotation) * dy
    local_z = dz  # Z 軸不受旋轉影響

    return [local_x, local_y, local_z]

# 計算相對於基座的目標位置 # Calculate the target position relative to the base
relative_target_position = transform_to_base_frame(global_target_position, arm_global_base, base_rotation)

# 反向運動學計算 Inverse kinematics calculations 
def calculate_inverse_kinematics(target):
    """
    使用 Denavit-Hartenberg (DH) 模型計算 UR5e 的反向運動學，並確保末端執行器朝向桌面下方
    :param target: 相對於手臂基座的目標末端位置 [x, y, z]
    :return: 計算出的關節角度列表 [theta1, theta2, theta3, theta4, theta5, theta6]
    Calculate the inverse kinematics of the UR5e using the Denavit-Hartenberg (DH)model 
    and ensure that the end-effector is facing downwards towards the tabletop
    """
    x, y, z = target

    # ---- Joint 1: Shoulder pan ----
    #Based on the target position, calculate the direction of shoulder rotation
    #so that the arm is pointing in the direction of the horizontal projection of the target

    theta1 = math.atan2(y, x)  # 根據目標位置，計算肩部旋轉方向，使手臂指向目標的水平投影方向 
    # ---- Joint 2 & 3: Shoulder lift and elbow ----
    r = math.sqrt(x**2 + y**2)  # 計算水平距離（r）
    z_offset = z - L1  # 計算目標點相對基座的高度偏移量
    D = (r**2 + z_offset**2 - L2**2 - L3**2) / (2 * L2 * L3)  # 餘弦定律計算肘部夾角參數
    if abs(D) > 1.0:
        raise ValueError("目標位置超出可達範圍！")  # 如果目標位置不可達，拋出錯誤
    theta3 = math.acos(D)  # 計算肘關節角度
    theta2 = math.atan2(z_offset, r) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))  # 計算肩升角

    # ---- Joint 4: Wrist 1 ----
    theta4 = - (theta2 + theta3)  # 手腕 1 關節補償肩和肘的旋轉角度

    # ---- Joint 5: Wrist 2 ----
    theta5 = math.pi / 2  # 設置手腕 2 垂直朝下的固定姿態（沿負 Z 軸）

    # ---- Joint 6: Wrist 3 ----
    theta6 = 0.0  # 保持 Wrist 3 不旋轉，末端執行器方向不變

    return [theta1, theta2, theta3, theta4, theta5, theta6]

# 計算目標關節角度
try:
    joint_angles = calculate_inverse_kinematics(relative_target_position)  # 透過反向運動學計算目標關節角度
except ValueError as e:
    print(f"錯誤: {e}")
    joint_angles = [0.0] * 6  # 如果目標不可達，設置所有關節角度為初始位置

# 主循環
while robot.step(TIME_STEP) != -1:  # Webots 的控制迴圈
    for i, joint in enumerate(joints):  # 遍歷所有關節
        joint.setPosition(joint_angles[i])  # 設置每個關節的目標角度（一次性到位運動）
    break  # 僅執行一次移動
