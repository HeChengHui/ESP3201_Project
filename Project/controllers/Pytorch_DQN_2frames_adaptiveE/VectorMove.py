"""Vector controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Robot
from controller import Camera, Motor, Keyboard, InertialUnit
# from controller import Supervisor
import sys
from collections import deque
from PIL import Image, ImageOps
import torch
import torchvision.transforms as T 



class move():
    
    @staticmethod
    def take_action(key, robot, camera):
        
        # camera.enable(20)

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
            # "1": (-1, -1, 10, 10),  # turn left
            "2": (4, 4, -4, -4),  # turn right
            # "2": (10, 10, -1, -1),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
        # function to map keyboard input to motor speed
        def motorCommand(key):
            MotorFrontLeftW.setVelocity(key[0])
            MotorBackLeftW.setVelocity(key[1])
            MotorFrontRightW.setVelocity(key[2])
            MotorBackRightW.setVelocity(key[3])
            
        # flag = False
        stacked_frames = deque(maxlen=4)
        frame_count = 1
        count = 0
        print(f"start: {robot.getTime()}")
        while robot.step(20) != -1:
            if count == 0 or count == 4 or count == 7 or count == 9:
                img_name = "image" + str(frame_count) + ".jpg"
                frame_count += 1
                img_taken = camera.saveImage(img_name, 20)
                # if img not saved, keep trying to save image
                while img_taken != 0:
                    img_taken = camera.saveImage(img_name, 20)
                
            if count == 9:
                motorCommand(motor_cmd[str(69)])
                MotorFrontLeftW.setPosition(float('inf'))
                MotorBackLeftW.setPosition(float('inf'))
                MotorFrontRightW.setPosition(float('inf'))
                MotorBackRightW.setPosition(float('inf'))
                print(f"time end: {robot.getTime()}")
                
                for i in range(1,5):
                    img_name = "image" + str(i) + ".jpg"
                    img = Image.open(img_name)
                    gray_image = ImageOps.grayscale(img)
                    resize = T.Compose([
                            T.Resize((36,64))  # height, width
                            ,T.ToTensor()
                        ])
                    stacked_frames.append(resize(gray_image).unsqueeze(0).to('cuda'))
                
                return torch.cat(tuple(stacked_frames), dim=1)
                
            motorCommand(motor_cmd[str(key)])
            count += 1
            
    @staticmethod
    def rotation_move(key, robot, camera, rotation):
        MotorFrontLeftW = robot.getDevice('base_to_flw')
        MotorBackLeftW = robot.getDevice('base_to_rlw')
        MotorFrontRightW = robot.getDevice('base_to_frw')
        MotorBackRightW = robot.getDevice('base_to_rrw')
        motor_cmd = {
            "0": (4, 4, 4, 4), # forward
            # "1": (-4, -4, 4, 4),  # turn left
            # "2": (4, 4, -4, -4),  # turn right
            "69": (0, 0, 0, 0)  # break
        }
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

        count = 0
        stacked_frames = deque(maxlen=2)
        while robot.step(20) != -1:
            
            if count == 0:
                img_taken = camera.saveImage('image1.jpg', 20)
                # if img not saved, keep trying to save image
                while img_taken != 0:
                    img_taken = camera.saveImage('image1.jpg', 20)
        
            if count == 9:
                motorCommand(motor_cmd[str(69)])
                MotorFrontLeftW.setPosition(float('inf'))
                MotorBackLeftW.setPosition(float('inf'))
                MotorFrontRightW.setPosition(float('inf'))
                MotorBackRightW.setPosition(float('inf'))
                # print(f"time end: {robot.getTime()}")
                
                img_taken = camera.saveImage('image2.jpg', 20)
                # if img not saved, keep trying to save image
                while img_taken != 0:
                    img_taken = camera.saveImage('image2.jpg', 20)
                    
                for i in range(1,3):
                    img_name = "image" + str(i) + ".jpg"
                    img = Image.open(img_name)
                    gray_image = ImageOps.grayscale(img)
                    resize = T.Compose([
                            T.Resize((36,64))  # height, width
                            ,T.ToTensor()
                        ])
                    stacked_frames.append(resize(gray_image).unsqueeze(0).to('cuda'))
                
                return (torch.cat(tuple(stacked_frames), dim=1), rotation)
                        
            if str(key) == "0":
                motorCommand(motor_cmd[str(key)])
            elif str(key) == "2":
                rotation+=1
                robot_rotation_field.setSFRotation(ROT_DICT[rotation%24])
                
                img_taken = camera.saveImage('image2.jpg', 20)
                # if img not saved, keep trying to save image
                while img_taken != 0:
                    img_taken = camera.saveImage('image2.jpg', 20)
                
                for i in range(1,3):
                    img_name = "image" + str(i) + ".jpg"
                    img = Image.open(img_name)
                    gray_image = ImageOps.grayscale(img)
                    resize = T.Compose([
                            T.Resize((36,64))  # height, width
                            ,T.ToTensor()
                        ])
                    stacked_frames.append(resize(gray_image).unsqueeze(0).to('cuda'))
                
                return (torch.cat(tuple(stacked_frames), dim=1), rotation)
            else:
                rotation-=1
                robot_rotation_field.setSFRotation(ROT_DICT[rotation%24])
                
                img_taken = camera.saveImage('image2.jpg', 20)
                # if img not saved, keep trying to save image
                while img_taken != 0:
                    img_taken = camera.saveImage('image2.jpg', 20)
                for i in range(1,3):
                    img_name = "image" + str(i) + ".jpg"
                    img = Image.open(img_name)
                    gray_image = ImageOps.grayscale(img)
                    resize = T.Compose([
                            T.Resize((36,64))  # height, width
                            ,T.ToTensor()
                        ])
                    stacked_frames.append(resize(gray_image).unsqueeze(0).to('cuda'))
                
                return (torch.cat(tuple(stacked_frames), dim=1), rotation)
            
            count += 1


class wait():
    def __init__(self, robot):
        count = 0
        while robot.step(20) != -1:
            count += 1
            if count == 15:
                return
            

