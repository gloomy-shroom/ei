import pybullet as p
import pybullet_data
import time
import numpy as np

# 初始化
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 加载环境
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
cube_id = p.loadURDF("cube.urdf", [0.6, 0, 0.02], globalScaling=0.1)

# 参数
ee_index = 11  # panda_hand
finger_links = [9, 10]  # 左右夹爪链接索引
gripper_open = 0.04
gripper_close = 0.0
target_orn = p.getQuaternionFromEuler([np.pi, 0, 0])

# 夹爪控制函数
def control_gripper(opening, force=500):
    p.setJointMotorControl2(robot_id, 9, p.POSITION_CONTROL, targetPosition=opening, force=force)
    p.setJointMotorControl2(robot_id, 10, p.POSITION_CONTROL, targetPosition=opening, force=force)

# 移动末端到指定位置
def move_to(position, orn, steps=100):
    joint_poses = p.calculateInverseKinematics(robot_id, ee_index, position, orn)
    for _ in range(steps):
        for j in range(7):  # Panda 机械臂的7个关节
            p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=joint_poses[j], force=1000)
        p.stepSimulation()
        time.sleep(1/240)

# 抓取过程
control_gripper(gripper_open)                           # 打开夹爪
move_to([0.6, 0, 0.25], target_orn)                     # 移动到物体上方
move_to([0.6, 0, 0.035], target_orn)                    # 慢慢下降（物体高度大约 0.04）
control_gripper(gripper_close)                          # 闭合夹爪

# 等待夹紧
for _ in range(100):
    p.stepSimulation()
    time.sleep(1/240)

# 检查两个夹爪是否接触物体
has_contact = False
for link in finger_links:
    if p.getContactPoints(robot_id, cube_id, linkIndexA=link):
        has_contact = True
        break

# 如果成功接触，创建夹持约束
if has_contact:
    print("✅ 抓取成功！")
    p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_index,
        childBodyUniqueId=cube_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
else:
    print("❌ 抓取失败")

# 抬起物体
move_to([0.6, 0, 0.25], target_orn)

# 观察一段时间
for _ in range(240):
    p.stepSimulation()
    time.sleep(1/240)


input("Press Enter to exit...")

p.disconnect()
