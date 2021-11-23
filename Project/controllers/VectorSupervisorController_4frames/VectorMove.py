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
            # "1": (-4, -4, 4, 4),  # turn left
            "1": (-1, -1, 10, 10),  # turn left
            # "2": (4, 4, -4, -4),  # turn right
            "2": (10, 10, -1, -1),  # turn right
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
                # print(f"time end: {robot.getTime()}")
                
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


class wait():
    def __init__(self, robot):
        count = 0
        while robot.step(20) != -1:
            count += 1
            if count == 15:
                return
            

