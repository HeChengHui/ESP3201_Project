"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera, Motor, Keyboard

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)


# enable camera 
camera = Camera('cozmo_camera')
camera.enable(timestep)
# initialise lift and head for camera view
# lift up
MotorLift = robot.getDevice('base_to_lift')
#MotorLift.setPosition(-0.8)
# head down
MotorHead = robot.getDevice('base_to_head')
#MotorHead.setPosition(0.17)

#initialise the wheels' rotational motors
MotorFrontLeftW = robot.getDevice('base_to_flw')
MotorFrontLeftW.setPosition(float('inf'))
MotorFrontLeftW.setVelocity(0.0)

MotorBackLeftW = robot.getDevice('base_to_rlw')
MotorBackLeftW.setPosition(float('inf'))
MotorBackLeftW.setVelocity(0.0)

MotorFrontRightW = robot.getDevice('base_to_frw')
MotorFrontRightW.setPosition(float('inf'))
MotorFrontRightW.setVelocity(0.0)

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
    "87": (5, 5, 5, 5), # forward
    "83": (-5, -5, -5, -5),  # backwards
    "65": (-5, -5, 5, 5),  # turn left
    "68": (5, 5, -5, -5),  # turn right
    "69": (0, 0, 0, 0)  # break
}
# function to map keyboard input to motor speed
def motorCommand(key):
    MotorFrontLeftW.setVelocity(key[0])
    MotorBackLeftW.setVelocity(key[1])
    MotorFrontRightW.setVelocity(key[2])
    MotorBackRightW.setVelocity(key[3])


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    # Process sensor data here.
    img = camera.getImage()
    
    # keyboard control
    key = keyboard.getKey()
    if str(key) in motor_cmd:
        motorCommand(motor_cmd[str(key)])
    

# Enter here exit cleanup code.
