"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from controller import Camera, Motor, Keyboard, InertialUnit
from controller import Supervisor
import sys
from Vector_move import move

# create the Robot instance.
# in this case i am using the supervisor to track pos
robot = Supervisor()
robot_node = robot.getFromDef("MY_ROBOT")
# get the time step of the current world.
# can be changed in Worldinfo, and is in ms
# so every passing timestep is 32ms in the simulation
timestep = int(robot.getBasicTimeStep())
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
robot_translateion_field = robot_node.getField("translation")
robot_rotation_field = robot_node.getField("rotation")
robot_pos = robot_translateion_field.getSFVec3f()

# enable camera 
camera = Camera('cozmo_camera')
# camera = Camera('vector_camera')
# camera.enable(timestep)
# initialise lift and head for camera view
# lift up
MotorLift = robot.getDevice('base_to_lift')
MotorLift.setPosition(-0.8)
# head down
MotorHead = robot.getDevice('base_to_head')
MotorHead.setPosition(0.42)

#initialise the wheels' rotational motors
MotorFrontLeftW = robot.getDevice('base_to_flw')
MotorFrontLeftW.setPosition(float('inf'))
MotorFrontLeftW.setVelocity(0.0)

# MotorFrontLeftW = robot.getDevice('front_left_wheel_joint')
# MotorFrontLeftW.setPosition(float('inf'))
# MotorFrontLeftW.setVelocity(0.0)

# MotorBackLeftW = robot.getDevice('rear_left_wheel_joint')
# MotorBackLeftW.setPosition(float('inf'))
# MotorBackLeftW.setVelocity(0.0)

MotorBackLeftW = robot.getDevice('base_to_rlw')
MotorBackLeftW.setPosition(float('inf'))
MotorBackLeftW.setVelocity(0.0)

# MotorFrontRightW = robot.getDevice('front_right_wheel_joint')
# MotorFrontRightW.setPosition(float('inf'))
# MotorFrontRightW.setVelocity(0.0)

MotorFrontRightW = robot.getDevice('base_to_frw')
MotorFrontRightW.setPosition(float('inf'))
MotorFrontRightW.setVelocity(0.0)

# MotorBackRightW = robot.getDevice('rear_right_wheel_joint')
# MotorBackRightW.setPosition(float('inf'))
# MotorBackRightW.setVelocity(0.0)

MotorBackRightW = robot.getDevice('base_to_rrw')
MotorBackRightW.setPosition(float('inf'))
MotorBackRightW.setVelocity(0.0)

# set their max speed
MaxWheelVelocity = 10


# keyboard control
keyboard = Keyboard()
keyboard.enable(timestep)
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
    
# supervisor to get position
# supervisor = Supervisor()
if robot_node is None:
    sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
    sys.exit(1)
trans_field = robot_node.getField("translation")

def reset():
        # reset both position and roation of robot
        robot_node.resetPhysics()
        MotorFrontLeftW.setVelocity(0)
        MotorBackLeftW.setVelocity(0)
        MotorFrontRightW.setVelocity(0)
        MotorBackRightW.setVelocity(0)
        MotorFrontLeftW.setPosition(float('inf'))
        MotorBackLeftW.setPosition(float('inf'))
        MotorFrontRightW.setPosition(float('inf'))
        MotorBackRightW.setPosition(float('inf'))
        robot_translateion_field.setSFVec3f([-0.21, 0.02, 0.21])
        robot_rotation_field.setSFRotation([1, 0, 0, -1.5708])

# IMU
# IMU = InertialUnit("imu inertial")
# IMU.enable(timestep)
# import time

flag = False
count = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
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
    
    # if flag == False:
    #     # keyboard control
    #     key = keyboard.getKey()
    #     if str(key) in motor_cmd:
    #         count = 0
    #         motorCommand(motor_cmd[str(key)])
    #         flag = True
    #         print(f"time start: {robot.getTime()}")
    #         continue
    # elif flag == True:
    #     count += 1
    #     if count == 5:
    #         flag = False
    #         motorCommand(motor_cmd[str(69)])
    #         print(f"time end: {robot.getTime()}")
            
    key = keyboard.getKey()
    if str(key) in motor_cmd:
        print(str(key))
        # move(str(key), robot, timestep)
        # key = None
        motorCommand(motor_cmd[str(key)])
        
    elif str(key) == "81":
        reset()
        
    # pauses the simulation
    # print(robot.simulationSetMode(0))

        
    
    

    
        
        

    

# Enter here exit cleanup code.
# # Process sensor data here.
        # # img = camera.getImage()
        
        # # save the image then convert into PIL
        # camera.saveImage('image.jpg', 20)
        # # img would be in RGB
        # img = Image.open("image.jpg")
        # gray_image = ImageOps.grayscale(img)
        # resize = T.Compose([
        #     T.Resize((36,64))  # height, width
        #     ,T.ToTensor()
        # ])
        # img_Tensor = resize(img).unsqueeze(0).to(device)  # into (Batch, channel, H, W)
                
        # # if the robot blast off can use this to reset
        # robot_pos = robot_translateion_field.getSFVec3f()
        # if robot_pos[1] > 0.2:
        #     ENV.reset()
        
        # # keyboard control
        # key = keyboard.getKey()
        # if str(key) in motor_cmd:
        #     motorCommand(motor_cmd[str(key)])
        # elif str(key) == "69":
        #     ENV.reset()