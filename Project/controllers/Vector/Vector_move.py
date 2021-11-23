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
        MotorFrontLeftW = robot.getDevice('base_to_flw')
        # MotorFrontLeftW = robot.getDevice('front_left_wheel_joint')
        # MotorFrontLeftW.setPosition(float('inf'))
        # MotorFrontLeftW.setVelocity(0.0)

        MotorBackLeftW = robot.getDevice('base_to_rlw')
        # MotorBackLeftW = robot.getDevice('rear_left_wheel_joint')
        # MotorBackLeftW.setPosition(float('inf'))
        # MotorBackLeftW.setVelocity(0.0)

        MotorFrontRightW = robot.getDevice('base_to_frw')
        # MotorFrontRightW = robot.getDevice('front_right_wheel_joint')
        # MotorFrontRightW.setPosition(float('inf'))
        # MotorFrontRightW.setVelocity(0.0)

        MotorBackRightW = robot.getDevice('base_to_rrw')
        # MotorBackRightW = robot.getDevice('rear_right_wheel_joint')
        # MotorBackRightW.setPosition(float('inf'))
        # MotorBackRightW.setVelocity(0.0)

        # bascially our action table.
        # each action will have the wheels move at different speed
        motor_cmd = {
            "87": (4, 4, 4, 4), # forward
            "83": (-4, -4, -4, -4),  # backwards
            "65": (-10, -10, 10, 10),  # turn left
            "68": (10, 10, -10, -10),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
        # function to map keyboard input to motor speed
        def motorCommand(key):
            MotorFrontLeftW.setVelocity(key[0])
            MotorBackLeftW.setVelocity(key[1])
            MotorFrontRightW.setVelocity(key[2])
            MotorBackRightW.setVelocity(key[3])
            
        ROT_DICT = {
            0:[-0.999893, -0.0103545, 0.0103549, 1.57088],
            1:[-0.982939, 0.119467, 0.13985, 1.58525],
            2:[-0.934915, 0.241038, 0.26045, 1.63249],
            3:[-0.862545, 0.348703, 0.366636, 1.70964],
            4:[-0.773432, 0.440096, 0.456201, 1.81335],
            5:[-0.674781, 0.514772, 0.528848, 1.9396],
            6:[-0.573401, 0.57329, 0.585278, 2.08235],
            7:[-0.471376, 0.618659, 0.628542, 2.23957],
            8:[-0.372247, 0.65236, 0.660196, 2.4051],
            9:[-0.274433, 0.677045, 0.682859, 2.57981],
            10:[-0.179279, 0.693724, 0.697571, 2.75895],
            11:[-0.0855076, 0.703563, 0.70547, 2.94206],
            12:[0.00732312, 0.707096, 0.70708, 3.12695],
            13:[0.0999666, 0.704532, 0.702596, -2.97121],
            14:[0.192944, 0.695749, 0.691886, -2.78816],
            15:[0.287322, 0.680195, 0.674374, -2.60802],
            16:[0.48192, 0.624489, 0.614627, -2.26435],
            17:[0.471974, 0.628203, 0.618548, -2.28085],
            18:[-0.581356, -0.581271, -0.569341, 2.10628],
            19:[-0.680598, -0.525018, -0.511021, 1.96114],
            20:[-0.776305, -0.453655, -0.437662, 1.83289],
            21:[-0.864069, -0.364741, -0.346914, 1.72483],
            22:[-0.935379, -0.259542, -0.240217, 1.64306],
            23:[-0.983036, -0.13946, -0.119125, 1.59061] 
        }    
        
        robot_node = robot.getFromDef("MY_ROBOT")
        robot_rotation_field = robot_node.getField("rotation")
            
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
                if count == 9:
                    flag = False
                    motorCommand(motor_cmd[str(69)])
                    print(f"time end: {robot.getTime()}")
                    return

    def rotation_move(key, robot, timestep, rotation):
        
        #initialise the wheels' rotational motors
        MotorFrontLeftW = robot.getDevice('base_to_flw')
        # MotorFrontLeftW = robot.getDevice('front_left_wheel_joint')
        # MotorFrontLeftW.setPosition(float('inf'))
        # MotorFrontLeftW.setVelocity(0.0)

        MotorBackLeftW = robot.getDevice('base_to_rlw')
        # MotorBackLeftW = robot.getDevice('rear_left_wheel_joint')
        # MotorBackLeftW.setPosition(float('inf'))
        # MotorBackLeftW.setVelocity(0.0)

        MotorFrontRightW = robot.getDevice('base_to_frw')
        # MotorFrontRightW = robot.getDevice('front_right_wheel_joint')
        # MotorFrontRightW.setPosition(float('inf'))
        # MotorFrontRightW.setVelocity(0.0)

        MotorBackRightW = robot.getDevice('base_to_rrw')
        # MotorBackRightW = robot.getDevice('rear_right_wheel_joint')
        # MotorBackRightW.setPosition(float('inf'))
        # MotorBackRightW.setVelocity(0.0)

        # bascially our action table.
        # each action will have the wheels move at different speed
        motor_cmd = {
            "87": (4, 4, 4, 4), # forward
            "83": (-4, -4, -4, -4),  # backwards
            "65": (-10, -10, 10, 10),  # turn left
            "68": (10, 10, -10, -10),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
        # function to map keyboard input to motor speed
        def motorCommand(key):
            MotorFrontLeftW.setVelocity(key[0])
            MotorBackLeftW.setVelocity(key[1])
            MotorFrontRightW.setVelocity(key[2])
            MotorBackRightW.setVelocity(key[3])
            
        ROT_DICT = {
            0:[-0.999893, -0.0103545, 0.0103549, 1.57088],
            1:[-0.982939, 0.119467, 0.13985, 1.58525],
            2:[-0.934915, 0.241038, 0.26045, 1.63249],
            3:[-0.862545, 0.348703, 0.366636, 1.70964],
            4:[-0.773432, 0.440096, 0.456201, 1.81335],
            5:[-0.674781, 0.514772, 0.528848, 1.9396],
            6:[-0.573401, 0.57329, 0.585278, 2.08235],
            7:[-0.471376, 0.618659, 0.628542, 2.23957],
            8:[-0.372247, 0.65236, 0.660196, 2.4051],
            9:[-0.274433, 0.677045, 0.682859, 2.57981],
            10:[-0.179279, 0.693724, 0.697571, 2.75895],
            11:[-0.0855076, 0.703563, 0.70547, 2.94206],
            12:[0.00732312, 0.707096, 0.70708, 3.12695],
            13:[0.0999666, 0.704532, 0.702596, -2.97121],
            14:[0.192944, 0.695749, 0.691886, -2.78816],
            15:[0.287322, 0.680195, 0.674374, -2.60802],
            16:[0.383845, 0.656841, 0.649017, -2.43225],
            17:[0.471974, 0.628203, 0.618548, -2.28085],
            18:[-0.581356, -0.581271, -0.569341, 2.10628],
            19:[-0.680598, -0.525018, -0.511021, 1.96114],
            20:[-0.776305, -0.453655, -0.437662, 1.83289],
            21:[-0.864069, -0.364741, -0.346914, 1.72483],
            22:[-0.935379, -0.259542, -0.240217, 1.64306],
            23:[-0.983036, -0.13946, -0.119125, 1.59061] 
        }    
        
        robot_node = robot.getFromDef("MY_ROBOT")
        robot_rotation_field = robot_node.getField("rotation")
            
        flag = False
        while robot.step(timestep) != -1:
        
            if flag == False:
                    count = 0
                    if key == "87":
                        motorCommand(motor_cmd[key])
                    elif key == "68":
                        rotation+=1
                        robot_rotation_field.setSFRotation(ROT_DICT[rotation%24])
                        print(rotation)
                    else:
                        rotation-=1
                        robot_rotation_field.setSFRotation(ROT_DICT[rotation%24])
                        print(rotation)
                        
                    flag = True
                    print(f"time start: {robot.getTime()}")
                    continue
            elif flag == True:
                count += 1
                if count == 10:
                    flag = False
                    motorCommand(motor_cmd[str(69)])
                    print(f"time end: {robot.getTime()}")
                    return rotation

    

# Enter here exit cleanup code.
