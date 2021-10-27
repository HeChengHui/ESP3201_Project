"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from controller import Camera, Motor, Keyboard, InertialUnit
from controller import Supervisor
import sys

class move():
    
    def __init__(self, key, robot):

        #initialise the wheels' rotational motors
        MotorFrontLeftW = robot.getDevice('base_to_flw')

        MotorBackLeftW = robot.getDevice('base_to_rlw')

        MotorFrontRightW = robot.getDevice('base_to_frw')

        MotorBackRightW = robot.getDevice('base_to_rrw')
        # bascially our action table.
        # each action will have the wheels move at different speed
        motor_cmd = {
            "0": (4, 4, 4, 4), # forward
            "1": (-4, -4, 4, 4),  # turn left
            "2": (4, 4, -4, -4),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
        # function to map keyboard input to motor speed
        def motorCommand(key):
            MotorFrontLeftW.setVelocity(key[0])
            MotorBackLeftW.setVelocity(key[1])
            MotorFrontRightW.setVelocity(key[2])
            MotorBackRightW.setVelocity(key[3])
            
        # flag = False
        count = 0
        while robot.step(40) != -1:
            if count == 5:
                motorCommand(motor_cmd[str(69)])
                MotorFrontLeftW.setPosition(float('inf'))
                MotorBackLeftW.setPosition(float('inf'))
                MotorFrontRightW.setPosition(float('inf'))
                MotorBackRightW.setPosition(float('inf'))
                # print(f"time end: {robot.getTime()}")
                return
                
            motorCommand(motor_cmd[str(key)])
            count += 1


class wait():
    def __init__(self, robot):
        count = 0
        while robot.step(40) != -1:
            count += 1
            if count == 15:
                return
            

