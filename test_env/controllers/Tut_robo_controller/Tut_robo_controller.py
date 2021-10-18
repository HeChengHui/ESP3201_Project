"""Tut_robo_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

# create the Robot instance.
robot = Robot()
# initialize motors
wheels = []
wheelsNames = ['Wheel1', 'Wheel2', 'Wheel3', 'Wheel4']
for name in wheelsNames:
    wheels.append(robot.getDevice(name))
    
# A Motor can be actuated by setting its position, its velocity, 
# its acceleration or its force. 
# Here we are interested in setting its velocity. 
# This can be achieved by setting its position to infinity, 
# and by bounding its velocity:
speed = -1.5  # [rad/s]
wheels[0].setPosition(float('inf'))
wheels[0].setVelocity(speed)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
