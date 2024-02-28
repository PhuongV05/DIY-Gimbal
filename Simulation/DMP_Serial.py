# Read data from Serial

import serial.tools.list_ports
import pybullet as p
import pybullet_data
import math

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portsList = []

#set environment
p.connect(p.GUI) # gives visualisation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)

targid = p.loadURDF("quadruped/block.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = False)

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

val = input("Select Port: COM")

for x in range(0,len(portsList)):
    if portsList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print(portVar)

serialInst.baudrate = 115200
serialInst.port = portVar
serialInst.open()

while True:
    if serialInst.in_waiting:
        packet = serialInst.readline()
        decoded = packet.decode('utf').rstrip('\n')
        raw_orn = decoded.split()
        orn = []
        for q in raw_orn:
            new_q = float(q)
            orn.append(new_q)

        euler = p.getEulerFromQuaternion(orn)
        print(euler)

        x = euler[0]
        y = euler[1]

        link3_r = math.pi/2 - x
        link6_r = math.pi/2 + y

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
        p.resetBasePositionAndOrientation(targid, [0,0,1], orn)
        p.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 0, cameraPitch = -10, cameraTargetPosition = focus_position)
                    
        p.stepSimulation()