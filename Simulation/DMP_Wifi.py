# Read data from Wifi

import pybullet as p
import pybullet_data
import math

import socket
import sys
from struct import unpack

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
host, port = '0.0.0.0', 65000
server_address = (host, port)

print(f'Starting UDP server on {host} port {port}')
sock.bind(server_address)


#set environment
p.connect(p.GUI) # gives visualisation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)

targid = p.loadURDF("quadruped/block.urdf", [0, 0, 1], [0, 0, 0, 1], useFixedBase = False)

while True:

    message, address = sock.recvfrom(4096)

    print(f'Received {len(message)} bytes:')
    i, j, k, l = unpack('4f', message)
    orn = [i,j,k,l]

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