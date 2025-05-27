import pybullet as p
import time
import pybullet_data
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadSDF("/home/xzs/PyBullet_Practice/bullet3-master/data/kuka_iiwa/kuka_with_gripper.sdf")

robotStartPos = [0,0,0]
cylinderStartPos = [1,0,0.3]
boxStartPos = [1,0,0.6 + 0.05 + 0.01]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
cylinderStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxStartOrientation = p.getQuaternionFromEuler([0,0,0])

p.resetBasePositionAndOrientation(robotId[0],robotStartPos,robotStartOrientation)
cylinderId = p.loadURDF("/home/xzs/PyBullet_Practice/bullet3-master/data/cylinder1.urdf",cylinderStartPos,cylinderStartOrientation)
boxId = p.loadURDF("/home/xzs/PyBullet_Practice/bullet3-master/data/cube1.urdf",boxStartPos,boxStartOrientation)

p.getNumJoints(robotId[0])#得到机器人的节点总数
p.getJointInfo(robotId[0],7)#得到机器人结点的信息
robot7StartPos = [0,0,1.2]
robotEndPos = [0.75,0,0.625]
robotEndOrientation = p.getQuaternionFromEuler([1.57,0,1.57])
startPos_array = np.array(robot7StartPos)
endPos_array = np.array(robotEndPos)
stepNum = 5
step_array = (endPos_array - startPos_array)/stepNum
for j in range(stepNum):
    print(j,"step")
    robotStepPos = list(step_array + startPos_array)
    targetPositionsJoints = p.calculateInverseKinematics(robotId[0],7,robotStepPos,targetOrientation = robotEndOrientation)
    p.setJointMotorControlArray(robotId[0],range(11),p.POSITION_CONTROL,targetPositions = targetPositionsJoints)
    for i in range (100):
        p.stepSimulation()
        time.sleep(1./10.)
        print("i:",i)
    print("------------------------------------------------------------------------------")
    startPos_array = np.array(robotStepPos)
p.disconnect()

