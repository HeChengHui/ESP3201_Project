import io
import json
import math
import sys
import time
from collections import deque, namedtuple
from enum import Enum

import anki_vector
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision.transforms as T
from anki_vector.util import degrees, distance_mm, speed_mmps
from PIL import Image, ImageOps, ImageEnhance

import ResNet

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



if __name__ == "__main__":
    args = anki_vector.util.parse_command_args()
    with anki_vector.robot.Robot(args.serial,show_viewer=True) as robots:
       
        robots.behavior.say_text("Initializing all systems in")
        time.sleep(1)
        robots.behavior.say_text("3")
        time.sleep(1)
        robots.behavior.say_text("2")
        time.sleep(1)
        robots.behavior.say_text("1")
        time.sleep(1)
        robots.behavior.say_text("Activating Head Actuator...")
        robots.behavior.set_head_angle(degrees(0.0))
        time.sleep(2)
        robots.behavior.say_text("Adjusting Head position...")
        robots.behavior.set_head_angle(degrees(-20.0))
        time.sleep(2)
        robots.behavior.say_text("Head position set...") 
        
   
        robots.behavior.say_text("Activating lift actuator...")
        robots.behavior.set_lift_height(1.0)
        time.sleep(1)
        
        robots.behavior.say_text("Checking imaging systems...")
        image_streaming_enabled = robots.camera.image_streaming_enabled()
        if image_streaming_enabled:
         robots.behavior.say_text("Video Streaming is enabled")
        else:
         robots.behavior.say_text("Video streaming is disabled")

        robots.behavior.say_text("Track Actuator Check...")
        robots.behavior.turn_in_place(degrees(45))  # turn left
        time.sleep(3)
        robots.behavior.turn_in_place(degrees(-45)) # turn right
        time.sleep(3)
        robots.behavior.say_text("Systems Check Complete...")
        device = torch.device("cuda")
        # policy_net = ResNet.ResNet50(1, 3).to(device) #RESNET
        policy_net = DQN().to(device)  # Naive DQN
        policy_net.load_state_dict(torch.load(r'C:\NUS\YEAR 4\ESP3201\Project\anki_vector_sdk_examples_0.6.0\tutorials\10300_DQN(c2).pth'))  # load in the weights
        policy_net.eval()
        agent = Agent(device)
        finish = 0

        while finish != 3000:
         robots.behavior.set_head_angle(degrees(-20.0))   
         img = robots.camera.latest_image.raw_image
         gray_image = ImageOps.grayscale(img)
         resize = T.Compose([
                T.Resize((36,64))  # height, width
                ,T.ToTensor()
            ])
         img = resize(gray_image).unsqueeze(0).to(device)
         action = agent.select_action(img, policy_net)
         print(str(action.item()))
         if action == 0:
          robots.behavior.set_head_angle(degrees(-20.0))    
          robots.behavior.drive_straight(distance_mm(25), speed_mmps(75))
          robots.behavior.set_head_angle(degrees(-20.0)) # forward
         elif action == 1:
          robots.behavior.set_head_angle(degrees(-20.0))    
          robots.behavior.turn_in_place(degrees(20))
          robots.behavior.set_head_angle(degrees(-20.0))  # turn left
         elif action == 2: 
          robots.behavior.set_head_angle(degrees(-20.0))   
          robots.behavior.turn_in_place(degrees(-20))
          robots.behavior.set_head_angle(degrees(-20.0))
         time.sleep(1)
         finish+=1
         robots.behavior.set_head_angle(degrees(-20.0))
        robots.disconnect() #disconnect from Vector
