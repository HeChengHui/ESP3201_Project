"The actual file to use on the robot"
import io
import json
import math

import sys
import time
from collections import deque, namedtuple
from enum import Enum

# import anki_vector
import torchvision.transforms as T
# from anki_vector.util import degrees, distance_mm, speed_mmps
from PIL import Image, ImageOps

import ResNet
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
# from VectorMove import move

# class VectorRobotEnvManager():
#     """
#         Observations:
#             2D greyscale image
        
#         Actions:
#             type = discrete \n
#             0 - Forward \n
#             1 - Left \n
#             2 - Right \n
#             the wheels velocity is set after knowing what actions to take
            
#         Rewards:
#             +100 passing a checkpoint in proper order \n
#             -100 going backwards a checkpoint \n
#             -20 going off course like going into the white inner area or +4cm off the outlayer
#             -2_000 for any termination
#             +2_000 for completing the course
            
#         Starting pos:
#             (x, y, z) = (-0.21, 0.02, 0.21)
            
#         Episode Termination:
#             1) if >4cm from outer white layer \n
#             2) if from starting position go backwards a region
#             3) if goes into regions of the track not in the correct order
#             4) Just in case, if robot somehow flies off
#         """
        
#     def __init__(self, device):
    
#         self.device = device
#         self.STARTING_POS = [-0.21, 0.02, 0.21]
#         self.STARTING_ROT = [-0.999896, -0.0101921, 0.0101969, 1.5708]
#         self.STARTING_SCREEN = None  # starting screen should be just black 
#         self.move_flag = False
#         self.done = False  # for termination
#         self.finish = False # for finishing the course
#         self.checkpoint = 0
#         self.CHECKPOINT_DICT = {  # store in checkpoint_number: [x1, x2, z1, z2]. x = Left to right, z = up to down
#             0: [-0.25, 0.0, 0.17, 0.25],
#             1: [0.0, 0.17, 0.17, 0.25],
#             2: [0.17, 0.25, 0.17, 0.25],
#             3: [0.17, 0.25, 0.145, 0.17],
#             4: [0.17, 0.25, 0.065, 0.145],
#             5: [0.08, 0.17, 0.065, 0.145],
#             6: [0.0, 0.08, 0.065, 0.145],
#             7: [0.0, 0.08, 0.04, 0.065],
#             8: [0.0, 0.08, -0.04, 0.04],
#             9: [0.08, 0.17, -0.04, 0.04],
#             10: [0.17, 0.25, -0.04, 0.04],
#             11: [0.17, 0.25, -0.065, -0.04],
#             12: [0.17, 0.25, -0.145, -0.065],
#             13: [0.08, 0.17, -0.145, -0.065],
#             14: [0.0, 0.08, -0.145, -0.065],
#             15: [0.0, 0.08, -0.25, -0.145],
#             16: [-0.17, 0.0, -0.25, -0.17],
#             17: [-0.25, -0.17, -0.25, -0.17],
#             18: [-0.25, -0.17, -0.17, 0.21],
#             19: [-0.25, -0.17, 0.21, 0.25]
#         }
       
#     def reset(self):
#         # reset both position and roation of robot
#         # MotorFrontLeftW.setVelocity(0)
#         # MotorBackLeftW.setVelocity(0)
#         # MotorFrontRightW.setVelocity(0)
#         # MotorBackRightW.setVelocity(0)
#         # MotorFrontLeftW.setPosition(float('inf'))
#         # MotorBackLeftW.setPosition(float('inf'))
#         # MotorFrontRightW.setPosition(float('inf'))
#         # MotorBackRightW.setPosition(float('inf'))
#         # robot_translateion_field.setSFVec3f(self.STARTING_POS)
#         # robot_rotation_field.setSFRotation(self.STARTING_ROT)
#         # robot_node.resetPhysics()
    
#     def num_actions_available(self):
#         # forward, left, right
#         return 3
    
#     def get_state(self):
#         # camera.saveImage('image.jpg', 20)
#         img = self.camera.capture_single_image()
#         gray_image = ImageOps.grayscale(img)
#         resize = T.Compose([
#                 T.Resize((36,64))  # height, width
#                 ,T.ToTensor()
#             ])
#         if self.finish:  # final image is black screen, cause Q-value at the goal is 0
#             black_screen = torch.zeros_like(resize(gray_image).unsqueeze(0).to(device))
#             return black_screen
#         else:
#             return resize(gray_image).unsqueeze(0).to(device)  # into (Batch, channel, H, W)
    
#     def take_action(self):        
#         robot_pos = robot_translateion_field.getSFVec3f()  # follow translation field of x, y, z
#         robot_x, robot_y, robot_z = robot_pos
    
#         # check if got pass through correct checkpoint oder
#         # check current checkpoint, get vertices of before and after checkpoint regions
#         current_region = self.CHECKPOINT_DICT[self.checkpoint]  # get a list of the region vertices
#         next_region = self.CHECKPOINT_DICT[self.checkpoint+1]
#         if self.checkpoint == 0:
#             prev_region = self.CHECKPOINT_DICT[18]
#         else:
#             prev_region = self.CHECKPOINT_DICT[self.checkpoint-1]
#         # track is defined as within the black areas and not on the borders. borders are ignored so dont have <= / >=
#         # if pos in current checkpoint, reward = -1
#         if (current_region[0] < robot_x < current_region[1]) and (current_region[2] < robot_z < current_region[3]):
#             reward = -1
#         # if pos in after checkpoint, reward = 100
#         # if pos is in last region, reward = 2000, terminate
#         elif (next_region[0] < robot_x < next_region[1]) and (next_region[2] < robot_z < next_region[3]):
#             reward = 100
#             self.checkpoint = self.checkpoint+1
#             if self.checkpoint == 19:
#                 reward = 2_000
#                 self.done = True
#                 self.finish = True
#         # if pos in before checkpoint, reward = -100
#         # if the robot pass last checkpoint from checkpoint 0, -2000 and terminate
#         elif (prev_region[0] < robot_x < prev_region[1]) and (prev_region[2] < robot_z < prev_region[3]):
#             reward = -100
#             if self.checkpoint == 0:
#                 reward = -2_000
#                 print("went back from 0")
#                 self.done = True
#             else:
#                 self.checkpoint = self.checkpoint-1
#         # if pos in white area, reward = -20
#         elif (-0.17 <= robot_x <= 0) and (-0.17 <= robot_z <= 0.17):
#             reward = -20
#         elif (0 < robot_x < 0.08) and ((0.145 <= robot_z <= 0.17) or (-0.065 <= robot_z <= -0.04)):
#             reward = -20
#         elif (0.08 <= robot_x <= 0.17) and ((0.145 <= robot_z <= 0.17) or (0.04 <= robot_z <= 0.065) \
#             or (-0.25 < robot_z <= -0.145) or (-0.065 <= robot_z <= -0.04)):
#             reward = -20
#         elif (0.17 < robot_x < 0.25) and ((0.04 <= robot_z <= 0.065) or (-0.25 < robot_z <= -0.145)):
#             reward = -20
#         elif (-0.29 <= robot_x <= -0.25) or (0.25 <= robot_x <= 0.29) \
#             or (-0.29 <= robot_z <= -0.25) or (0.25 <= robot_z <= 0.29):  # can still go out of the 50cm track by 4cm
#             reward = -20
#         # else, terminate. Either by exiting boundary, blast off, or never follow 
#         elif (robot_z > 0.29) or (robot_z < -0.29) or (robot_x > 0.29) or (robot_x < -0.29):  # exit boundary
#             self.done = True
#             reward = -2_000
#             print("exit boundary")
#         elif (robot_y > 0.04):  # blast off
#             self.done = True
#             reward = 0  # since is like kinda due to the freaking model's fault, not gonna give it a -ve reward
#         else:  # going to other parts of the track and not following the checkpoint order
#             self.done = True
#             reward = -2_000
#             print("others terminate")
#             print(self.checkpoint)
#             print(f"x: {robot_x}, z: {robot_z}")
            
#         # reward is converted to a tensor to keep everything consistently tensor
#         return torch.tensor([reward], device=self.device)


class Agent():
    # device is for torch to use GPU or CPU
    def __init__(self, device):
        self.device = device  
        
    # policy_net is the DNN to train. target_net is to help calculate the loss.
    def select_action(self, state, policy_net):
        with torch.no_grad():  # use .no_grad() if not using the model for training. To turn off grad tracking
            # passing the tensor to either GPU
            return policy_net(state).argmax(dim=1).to(self.device) # exploit


class DQN(nn.Module):
    # DQN will receive images from the camera as input, to create a DQN object
    def __init__(self):
        super().__init__()
        self.DENSE_INPUT = 256  # found by flattening and printing out the shape beforehand

        # conv part based on https://lopespm.github.io/machine_learning/2016/10/06/deep-reinforcement-learning-racing-game.html
        self.conv_net = torch.nn.Sequential(
            torch.nn.Conv2d(1, 32, (8,8), (4,4)),  # in channel, out, kernel, stride
            torch.nn.ReLU(),
            torch.nn.Conv2d(32, 64, (4,4), (2,2)),  # in, out, kernel, stride
            torch.nn.ReLU(),
            torch.nn.Conv2d(64, 64, (3,3), (1,1)),  # in, out, kernel, stride
        )
        
        self.fc_net = torch.nn.Sequential(
            torch.nn.Linear(self.DENSE_INPUT, 64), 
            torch.nn.ReLU(),
            torch.nn.Linear(64, 32)
        )
        
        self.out = torch.nn.Linear(32, 3)
        
    # AKA forward pass.
    # all PyTorch neural networks require an implementation of forward()
    def forward(self, t):
        t = F.relu(self.conv_net(t))
        t = t.flatten(start_dim=1)
        self.DENSE_INPUT = t.shape[1]  # 256
        t = F.relu(self.fc_net(t))
        t = self.out(t)
        return t


### MAIN #########################################################################################################################################################################################################################################
if __name__ == "__main__":
    ## Robot initialisation ##############################################################################
    # create the Robot instance.
#  args = anki_vector.util.parse_command_args()
#  with anki_vector.Robot(args.serial,show_viewer=True) as robots:
    # print("Neutral Position...")
    # robots.behavior.set_head_angle(degrees(0.0))
    # time.sleep(2)
    # print("Head down...")
    # robots.behavior.set_head_angle(degrees(-20.0))
    # time.sleep(2) 
   
    # print("Raise Vector's lift...")
    # robots.behavior.set_lift_height(1.0)


    # enable camera 
    # camera = Camera('cozmo_camera')
    # camera.enable(timestep)

    # initialise lift and head for camera view
    # lift up
    # MotorLift = robot.getDevice('base_to_lift')
    # MotorLift.setPosition(-0.8)
    # head down
    # MotorHead = robot.getDevice('base_to_head')
    # MotorHead.setPosition(0.42)

    # initialise the wheels' rotational motors
    # Max Speed = 10
    # MotorFrontLeftW = robot.getDevice('base_to_flw')
    # MotorFrontLeftW.setPosition(float('inf'))
    # MotorFrontLeftW.setVelocity(0.0)

    # MotorBackLeftW = robot.getDevice('base_to_rlw')
    # MotorBackLeftW.setPosition(float('inf'))
    # MotorBackLeftW.setVelocity(0.0)

    # MotorFrontRightW = robot.getDevice('base_to_frw')
    # MotorFrontRightW.setPosition(float('inf'))
    # MotorFrontRightW.setVelocity(0.0)

    # MotorBackRightW = robot.getDevice('base_to_rrw')
    # MotorBackRightW.setPosition(float('inf'))
    # MotorBackRightW.setVelocity(0.0)

    # def motorCommand(key):
    #     MotorFrontLeftW.setVelocity(key[0])
    #     MotorBackLeftW.setVelocity(key[1])
    #     MotorFrontRightW.setVelocity(key[2])
    #     MotorBackRightW.setVelocity(key[3])
    
    ## HYPERPARAMETERS ##############################################################################

    device = torch.device("cuda")  # want to use GPU
    # ENV = VectorRobotEnvManager(device)
    # ENV.reset()
   
        # if False:  # final image is black screen, cause Q-value at the goal is 0
        #     black_screen = torch.zeros_like(resize(gray_image).unsqueeze(0).to(device))
        #     # return black_screen
        # else:
      # into (Batch, channel, H, W)
    # choose 1 and comment out the other
    # policy_net = ResNet.ResNet50(1, 3).to(device)  # ResNet
    policy_net = DQN().to(device)  # Naive DQN
    
    policy_net.load_state_dict(torch.load('8300_DQN.pth'))  # load in the weights
    policy_net.eval()
    
    actual_reward = 0
    agent = Agent(device)
    finish = 0
    while finish != -1:
        # state = ENV.get_state()
        
        img = Image.open("image.jpg") #load random image inside here
        gray_image = ImageOps.grayscale(img)
        resize = T.Compose([
                T.Resize((36,64))  # height, width
                ,T.ToTensor()
            ])
        img = resize(gray_image).unsqueeze(0).to(device)
        action = agent.select_action(img, policy_net)  # select an action base on the epsilon greedy
        # print(action)
        # move(action.item(), robots)  # execute action for 0.2s
        # reward = ENV.take_action()  # get reward for the action
        print(str(action.item()))
        # actual_reward += reward.item()  # accumulate the reward for this ep
        
        # if terminate or finish track, go to next ep
        # if ENV.done:  
        #     if ENV.finish:
        #         print("complete track!")
        #         print(f"reward: {actual_reward}")
        #     else:
        #         print("terminate")
            
        #     robot.simulationSetMode(0)
        