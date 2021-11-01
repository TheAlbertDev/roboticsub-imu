"""
********************************************************************************
RoboticsUB-TCP_Endowrist_from_IMU.py - Control based on a IMU.
Copyright (C) 2021  Albert Álvarez-Carulla
Repository: https://github.com/TheAlbertDev/roboticsub-imu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
********************************************************************************
"""

import serial
import time
import math
from enum import Enum

# RoboDK API: import the robolink library (bridge with RoboDK)
from robolink import *
# Robot toolbox: import the robodk library (robotics toolbox)
from robodk import *

# ------------------------------------------------------------------------------
# Connection
# ------------------------------------------------------------------------------

# Establish the connection on a specific port
arduino = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)

# Lets bring some time to the system to stablish the connetction
time.sleep(2)

# Establish a link with the simulator
RDK = Robolink()

# ------------------------------------------------------------------------------
# Simulator setup
# ------------------------------------------------------------------------------

# Retrieve all items (object in the robodk tree)
# Define the "robot" variable with our robot (UR5e)
robot = RDK.Item('UR5e')

# Define the "tcp" variable with the TCP of Endowrist needle
tcp_tool = RDK.Item('TCP_Endowrist')

# Performs a quick check to validate items defined
if robot.Valid():
    print('Robot selected: ' + robot.Name())
if tcp_tool.Valid():
    print('Tool selected: ' + tcp_tool.Name())

# Robot Flange with respect to UR5e base Frame
print('Robot POSE is: ' + repr(robot.Pose()))
# Tool frame with respect to Robot Flange
print('Robot POSE is: ' + repr(robot.PoseTool()))
# Tool frame with respect to Tool frame
print('TCP pose is: ' + repr(tcp_tool.Pose()))

robot.setSpeed(10)

# ------------------------------------------------------------------------------
# Reference frame is fixed to TCP
#
# Data comunication
# ------------------------------------------------------------------------------
#


class Command(Enum):
    GET_RPW = b'\x01'


try:

    # Discard initial ESP32 message
    arduino.reset_input_buffer()

    while True:

        # Requesting data to Ardino
        arduino.write(Command.GET_RPW.value)

        # Storing received data
        roll_str = arduino.readline().strip()
        pitch_str = arduino.readline().strip()
        yaw_str = arduino.readline().strip()
        s1_str = arduino.readline().strip()
        s2_str = arduino.readline().strip()

        # Convert variable values from string to float
        roll = float(roll_str)
        pitch = float(pitch_str)
        yaw = float(yaw_str)
        s1 = bool(int(s1_str))
        s2 = bool(int(s2_str))

        print(roll, pitch, yaw, s1, s2)

        # Convert from degrees to radians R,P,Y angles
        R = math.radians(roll)
        P = math.radians(pitch)
        W = math.radians(yaw)
        X = 0
        Y = -60
        Z = 320

        # Calculate the POSE matrix (UR)
        # TODO: Complete the POSE matrix
        # You have to translate the tool to the (X,Y,Z) position and apply the
        # R,P,W rotations to the tool
        # The tool and IMU axis must be consistent
        pose_matrix = <- TODO  # It will throw error if you don't implement it

        tcp_tool_pose = tcp_tool.setPoseTool(pose_matrix)

        # TODO: Move linealy the robot with the S1 and S2 buttons through the Y axis

except KeyboardInterrupt:
    print("Communication stopped.")
    pass

# ------------------------------------------------------------------------------
# Disconnect Arduino
# ------------------------------------------------------------------------------
print("Disconnecting Arduino...")
arduino.close()
