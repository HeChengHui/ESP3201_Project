"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
from controller import Camera, Motor, Keyboard

# create the Robot instance.
robot = Supervisor()
robot_node = robot.getFromDef("MY_ROBOT")
robot_translateion_field = robot_node.getField("translation")
robot_rotation_field = robot_node.getField("rotation")
# get the time step of the current world.
# can be changed in Worldinfo, and is in ms
# so every passing timestep is 32ms in the simulation
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
MotorLift.setPosition(-0.8)
# head down
MotorHead = robot.getDevice('base_to_head')
MotorHead.setPosition(0.42)

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

def reset():
    # reset both position and roation of robot
        
    MotorFrontLeftW.setVelocity(0)
    MotorBackLeftW.setVelocity(0)
    MotorFrontRightW.setVelocity(0)
    MotorBackRightW.setVelocity(0)
    MotorFrontLeftW.setPosition(float('inf'))
    MotorBackLeftW.setPosition(float('inf'))
    MotorFrontRightW.setPosition(float('inf'))
    MotorBackRightW.setPosition(float('inf'))
    robot_translateion_field.setSFVec3f([0.000892718, 0.035, 0.00131016])
    robot_rotation_field.setSFRotation([-0.999895, -0.0102645, 0.010269, 1.57088])
    robot_node.resetPhysics()

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
    if str(key) == "81":
        reset()
        

    

# Enter here exit cleanup code.
