import pybullet as p
import numpy as np
import time
import pybullet_data


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 加载环境和机器人
planeId = p.loadURDF("plane.urdf")
pandaId = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
tableId = p.loadURDF("table/table.urdf", [0.5, 0, -0.65])
objectId = p.loadURDF("cube.urdf", [0.6, 0, 0.02], globalScaling=0.05)

# 改进物体属性以提高抓取成功率
p.changeDynamics(objectId, -1, mass=0.1)
p.changeDynamics(objectId, -1, lateralFriction=1.0)
p.changeDynamics(objectId, -1, spinningFriction=0.1)

# 设置关节初始位置 - 使用更合理的初始姿态
p.resetJointState(pandaId, 0, -0.785)  # 基关节旋转
p.resetJointState(pandaId, 1, -0.785)  # 肩部关节
p.resetJointState(pandaId, 2, 0.785)   # 肘部关节
p.resetJointState(pandaId, 3, -1.57)   # 腕部关节
p.resetJointState(pandaId, 4, -0.785)  # 腕部关节2
p.resetJointState(pandaId, 5, 1.57)    # 腕部关节3
p.resetJointState(pandaId, 6, 0.785)   # 末端执行器旋转

# 获取末端执行器ID
end_effector_id = 11

# 修正目标位置 - 考虑末端执行器偏移
tool_offset = 0.0  # 末端执行器从腕部关节的偏移距离
target_position = [0.6 - tool_offset, 0, 0.1]  # 补偿工具偏移
target_orientation = p.getQuaternionFromEuler([np.pi, 0, 0])

# 生成轨迹点（删除可视化调用）
num_points = 20
waypoints = []
start_position = p.getLinkState(pandaId, end_effector_id)[0]

for i in range(num_points + 1):
    alpha = i / num_points
    position = [
        start_position[0] + alpha * (target_position[0] - start_position[0]),
        start_position[1] + alpha * (target_position[1] - start_position[1]),
        start_position[2] + alpha * (target_position[2] - start_position[2])
    ]
    waypoints.append((position, target_orientation))

# 执行轨迹（删除调试线条）
for position, orientation in waypoints:
    joint_angles = p.calculateInverseKinematics(
        pandaId, end_effector_id, position, orientation,
        lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
        upperLimits=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
        jointRanges=[5.7946, 3.5256, 5.7946, 3.002, 5.7946, 3.77, 5.7946],
        restPoses=[0, -0.785, 0.785, -1.57, -0.785, 1.57, 0.785],
        jointDamping=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
    )
    
    for i in range(7):
        p.setJointMotorControl2(
            pandaId, i, p.POSITION_CONTROL, 
            targetPosition=joint_angles[i], 
            force=500
        )
    
    p.stepSimulation()
    time.sleep(0.01)

# 改进的抓取流程（删除抓取位置可视化）
grab_position = [0.6 - tool_offset, 0, 0.02]  # 补偿工具偏移
grab_orientation = target_orientation

for i in range(num_points + 1):
    alpha = i / num_points
    position = [
        target_position[0] + alpha * (grab_position[0] - target_position[0]),
        target_position[1] + alpha * (grab_position[1] - target_position[1]),
        target_position[2] + alpha * (grab_position[2] - target_position[2])
    ]
    
    joint_angles = p.calculateInverseKinematics(
        pandaId, end_effector_id, position, grab_orientation,
        lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
        upperLimits=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
        jointRanges=[5.7946, 3.5256, 5.7946, 3.002, 5.7946, 3.77, 5.7946],
        restPoses=[0, -0.785, 0.785, -1.57, -0.785, 1.57, 0.785],
        jointDamping=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
    )
    
    for i in range(7):
        p.setJointMotorControl2(
            pandaId, i, p.POSITION_CONTROL, 
            targetPosition=joint_angles[i], 
            force=500
        )
    
    p.stepSimulation()
    time.sleep(0.01)

# 关闭夹爪
for i in range(2):
    p.setJointMotorControl2(
        pandaId, 9 + i, p.POSITION_CONTROL, 
        targetPosition=0.003, 
        force=100
    )

for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)

# 检查是否成功抓取
def check_grasp_success():
    contact_points = p.getContactPoints(pandaId, objectId)
    return len(contact_points) > 0

is_grasp_successful = check_grasp_success()
print(f"抓取状态: {'成功' if is_grasp_successful else '失败'}")

# 提升物体（删除调试文字）
lift_position = [0.6 - tool_offset, 0, 0.3]
lift_orientation = target_orientation

slow_num_points = 30
for i in range(slow_num_points + 1):
    alpha = i / slow_num_points
    position = [
        grab_position[0] + alpha * (lift_position[0] - grab_position[0]),
        grab_position[1] + alpha * (lift_position[1] - grab_position[1]),
        grab_position[2] + alpha * (lift_position[2] - grab_position[2])
    ]
    
    joint_angles = p.calculateInverseKinematics(
        pandaId, end_effector_id, position, lift_orientation,
        lowerLimits=[-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
        upperLimits=[2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
        jointRanges=[5.7946, 3.5256, 5.7946, 3.002, 5.7946, 3.77, 5.7946],
        restPoses=[0, -0.785, 0.785, -1.57, -0.785, 1.57, 0.785],
        jointDamping=[0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
    )
    
    for i in range(7):
        p.setJointMotorControl2(
            pandaId, i, p.POSITION_CONTROL, 
            targetPosition=joint_angles[i], 
            maxVelocity=0.1,
            force=500
        )
    
    p.stepSimulation()
    time.sleep(0.02)

is_grasp_successful_after_lift = check_grasp_success()
print(f"提升后抓取状态: {'成功' if is_grasp_successful_after_lift else '失败'}")

# 保持仿真运行
while True:
    p.stepSimulation()
    time.sleep(0.01)
