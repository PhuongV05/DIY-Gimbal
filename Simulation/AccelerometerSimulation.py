import numpy
import pybullet as p
import pybullet_data
import time
from math import pi, atan2, sqrt

#set environment
p.connect(p.GUI) # gives visualisation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)


targid = p.loadURDF("quadruped/block.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = False)
roll = p.addUserDebugParameter("roll", -pi/2, pi/2, 0)
pitch = p.addUserDebugParameter("pitch", -pi/2, pi/2, 0)
yaw = p.addUserDebugParameter("yaw", -pi/2, pi/2, 0)
link1_s = p.addUserDebugParameter("link1", 0, pi, 0)
link3_s = p.addUserDebugParameter("link3", 0, pi, pi/2)
link6_s = p.addUserDebugParameter("link6", 0, pi, pi/2)


while True:
    
    x = p.readUserDebugParameter(roll)
    y = p.readUserDebugParameter(pitch)
    z = p.readUserDebugParameter(yaw)
    link1_r = p.readUserDebugParameter(link1_s)
    #link3_r = p.readUserDebugParameter(link3_s)
    #link6_r = p.readUserDebugParameter(link6_s)
    link3_r = pi/2 - x
    link6_r = pi/2 + y

    p.setJointMotorControl2(
        bodyIndex = targid,
        jointIndex = 0,
        controlMode = p.POSITION_CONTROL,
        targetPosition = link1_r,
        force = 100
    )

    p.setJointMotorControl2(
        bodyIndex = targid,
        jointIndex = 2,
        controlMode = p.POSITION_CONTROL,
        targetPosition = link3_r,
        force = 100
    )

    p.setJointMotorControl2(
        bodyIndex = targid,
        jointIndex = 5,
        controlMode = p.POSITION_CONTROL,
        targetPosition = link6_r,
        force = 100
    )

    focus_position, _ = p.getBasePositionAndOrientation(targid)

    orn = p.getQuaternionFromEuler([x, y, z])
    print(orn)
    p.resetBasePositionAndOrientation(targid, [0, 0, 1], orn)
    
    p.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 0, cameraPitch = -20, cameraTargetPosition = focus_position)

    p.stepSimulation()
