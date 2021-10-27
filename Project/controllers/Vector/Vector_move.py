"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from controller import Camera, Motor, Keyboard, InertialUnit
from controller import Supervisor
import sys

class move():
    
    def __init__(self, key, robot, timestep):

        #initialise the wheels' rotational motors
        # MotorFrontLeftW = robot.getDevice('base_to_flw')
        MotorFrontLeftW = robot.getDevice('front_left_wheel_joint')
        # MotorFrontLeftW.setPosition(float('inf'))
        # MotorFrontLeftW.setVelocity(0.0)

        # MotorBackLeftW = robot.getDevice('base_to_rlw')
        MotorBackLeftW = robot.getDevice('rear_left_wheel_joint')
        # MotorBackLeftW.setPosition(float('inf'))
        # MotorBackLeftW.setVelocity(0.0)

        # MotorFrontRightW = robot.getDevice('base_to_frw')
        MotorFrontRightW = robot.getDevice('front_right_wheel_joint')
        # MotorFrontRightW.setPosition(float('inf'))
        # MotorFrontRightW.setVelocity(0.0)

        # MotorBackRightW = robot.getDevice('base_to_rrw')
        MotorBackRightW = robot.getDevice('rear_right_wheel_joint')
        # MotorBackRightW.setPosition(float('inf'))
        # MotorBackRightW.setVelocity(0.0)

        # bascially our action table.
        # each action will have the wheels move at different speed
        motor_cmd = {
            "87": (4, 4, 4, 4), # forward
            "83": (-4, -4, -4, -4),  # backwards
            "65": (-4, -4, 4, 4),  # turn left
            "68": (4, 4, -4, -4),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
        # function to map keyboard input to motor speed
        def motorCommand(key):
            MotorFrontLeftW.setVelocity(key[0])
            MotorBackLeftW.setVelocity(key[1])
            MotorFrontRightW.setVelocity(key[2])
            MotorBackRightW.setVelocity(key[3])
            
        flag = False
        while robot.step(timestep) != -1:
            # Read the sensors:
            # Enter here functions to read sensor data, like:
            #  val = ds.getValue()
            # Enter here functions to send actuator commands, like:
            #  motor.setPosition(10.0)
            
            # Process sensor data here.
            # img = camera.getImage()
            
            # print position (aka translation field)
            # values = trans_field.getSFVec3f()
            # print("MY_ROBOT is at position: %g %g %g" % (values[0], values[1], values[2]))

            # print(IMU.getRollPitchYaw())

            if flag == False:
                count = 0
                motorCommand(motor_cmd[str(key)])
                flag = True
                print(f"time start: {robot.getTime()}")
                continue
            elif flag == True:
                count += 1
                if count == 5:
                    flag = False
                    motorCommand(motor_cmd[str(69)])
                    print(f"time end: {robot.getTime()}")
                    return


    

# Enter here exit cleanup code.
