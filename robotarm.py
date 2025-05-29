import pybullet as p
import numpy as np
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF("plane.urdf")
pandaId = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
tableId = p.loadURDF("table/table.urdf", [0.5, 0, -0.65])
objectId = p.loadURDF("cube.urdf", [0.6, 0, 0.02], globalScaling=0.05)

# 改进物体属性提高抓取成功率
p.changeDynamics(objectId, -1, mass=0.1)
p.changeDynamics(objectId, -1, lateralFriction=1.0)
p.changeDynamics(objectId, -1, spinningFriction=0.1)
# 设置更合理的关节初始姿态
p.resetJointState(pandaId, 0, -0.785)  
p.resetJointState(pandaId, 1, -0.785)  
p.resetJointState(pandaId, 2, 0.785)   
p.resetJointState(pandaId, 3, -1.57)   
p.resetJointState(pandaId, 4, -0.785)  
p.resetJointState(pandaId, 5, 1.57)    
p.resetJointState(pandaId, 6, 0.785)   

end_effector_id = 11
target_position = [0.6 , 0, 0.1]  
target_orientation = p.getQuaternionFromEuler([np.pi, 0, 0]) #四元数表示

# 生成轨迹点
num_points = 20
waypoints = []
start_position = p.getLinkState(pandaId, end_effector_id)[0]

for i in range(num_points + 1):
    alpha = i / num_points
    position = [ start_position[0] + alpha * (target_position[0] - start_position[0]),
                 start_position[1] + alpha * (target_position[1] - start_position[1]),
                 start_position[2] + alpha * (target_position[2] - start_position[2])]
    waypoints.append((position, target_orientation))  #插值   位置+朝向

# 执行轨迹
for position, orientation in waypoints:
    joint_angles = p.calculateInverseKinematics(pandaId, end_effector_id, position, orientation)
    
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=500)
    
    p.stepSimulation()
    time.sleep(0.01)


# 抓取流程
grab_position = [0.6 , 0, 0.02] 
grab_orientation = target_orientation

for i in range(num_points + 1):
    alpha = i / num_points
    position = [ target_position[0] + alpha * (grab_position[0] - target_position[0]),
                 target_position[1] + alpha * (grab_position[1] - target_position[1]),
                 target_position[2] + alpha * (grab_position[2] - target_position[2])]
    
    joint_angles = p.calculateInverseKinematics(pandaId, end_effector_id, position, grab_orientation)
    
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=500)
    
    p.stepSimulation()
    time.sleep(0.01)

# 关闭夹爪（关节9+关节10）
for i in range(2):
    p.setJointMotorControl2(pandaId, 9 + i, p.POSITION_CONTROL, targetPosition=0.003, force=100)

for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)

# 检查是否成功抓取
def checkgrasp():
    contact_points = p.getContactPoints(pandaId, objectId)
    return len(contact_points) > 0

is_grasp_successful = checkgrasp()
if is_grasp_successful:
    print("抓取成功")
else:
    print("抓取失败")


# 提升物体
lift_position = [0.6 , 0, 0.3]
lift_orientation = target_orientation

num_points = 20
for i in range(num_points + 1):
    alpha = i / num_points
    position = [ grab_position[0] + alpha * (lift_position[0] - grab_position[0]),
                 grab_position[1] + alpha * (lift_position[1] - grab_position[1]),
                 grab_position[2] + alpha * (lift_position[2] - grab_position[2])]
    
    joint_angles = p.calculateInverseKinematics( pandaId, end_effector_id, position, lift_orientation)
    
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], maxVelocity=0.5,force=500)
    
    p.stepSimulation()
    time.sleep(0.01)


while True:
    p.stepSimulation()
    time.sleep(0.01)
