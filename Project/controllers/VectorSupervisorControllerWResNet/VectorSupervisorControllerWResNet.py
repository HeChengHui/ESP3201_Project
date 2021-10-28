"""VectorSupervisorController controller."""
# from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from gym.core import Env
from gym.spaces import Box, Discrete
import numpy as np
import math
import random
from collections import namedtuple
from collections import deque
from PIL import Image, ImageOps
import torch
import torch.nn as nn
from torch.nn.modules.activation import ReLU
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T 

import sys
from VectorMove import move, wait
import ResNet
from controller import Supervisor
from controller import Camera, Motor, InertialUnit, Keyboard


class VectorRobotEnvManager():
    """
        Observations:
            2D greyscale image
        
        Actions:
            type = discrete \n
            0 - Forward \n
            1 - Left \n
            2 - Right \n
            the wheels velocity is set after knowing what actions to take
            
        Rewards:
            +100 passing a checkpoint \n
            -100 going backwards a checkpoint \n
            -10 going off course like going into the white colour area
            
        Starting pos:
            (x, y, z) = (-0.21, 0.02, 0.21)
            
        Episode Termination:
            1) if +4cm away from the 50cm track on 4 sides (x & z axis), and +1cm on top (y axis) \n
            2) if next state cross starting position
        """
        
    def __init__(self, device):
        # super().__init__()  # deepbots not usable  0242821
    
        self.device = device
        self.STARTING_POS = [-0.21, 0.02, 0.21]
        self.STARTING_ROT = [-0.999896, -0.0101921, 0.0101969, 1.5708]
        self.STARTING_SCREEN = None  # starting screen should be just black 
        self.move_flag = False
        self.done = False  # for termination
        self.finish = False # for finishing the course
        self.checkpoint = 0
        self.CHECKPOINT_DICT = {  # store in checkpoint_number: [x1, x2, z1, z2]. Left to right, up to down
            0: [-0.25, 0.0, 0.17, 0.25],
            1: [0.0, 0.17, 0.17, 0.25],
            2: [0.17, 0.25, 0.17, 0.25],
            3: [0.17, 0.25, 0.145, 0.17],
            4: [0.17, 0.25, 0.065, 0.145],
            5: [0.08, 0.17, 0.065, 0.145],
            6: [0.0, 0.08, 0.065, 0.145],
            7: [0.0, 0.08, 0.04, 0.065],
            8: [0.0, 0.08, -0.04, 0.04],
            9: [0.08, 0.17, -0.04, 0.04],
            10: [0.17, 0.25, -0.04, 0.04],
            11: [0.17, 0.25, -0.065, -0.04],
            12: [0.17, 0.25, -0.145, -0.065],
            13: [0.08, 0.17, -0.145, -0.065],
            14: [0.0, 0.08, -0.145, -0.065],
            15: [0.0, 0.08, -0.25, -0.145],
            16: [-0.17, 0.0, -0.25, -0.17],
            17: [-0.25, -0.17, -0.25, -0.17],
            18: [-0.25, -0.17, -0.17, 0.17],
            19: [-0.25, -0.17, 0.17, 0.25]
        }
       
    def reset(self):
        # reset both position and roation of robot
        
        MotorFrontLeftW.setVelocity(0)
        MotorBackLeftW.setVelocity(0)
        MotorFrontRightW.setVelocity(0)
        MotorBackRightW.setVelocity(0)
        MotorFrontLeftW.setPosition(float('inf'))
        MotorBackLeftW.setPosition(float('inf'))
        MotorFrontRightW.setPosition(float('inf'))
        MotorBackRightW.setPosition(float('inf'))
        robot_translateion_field.setSFVec3f(self.STARTING_POS)
        robot_rotation_field.setSFRotation(self.STARTING_ROT)
        robot_node.resetPhysics()
        # MotorFrontLeftW.setPosition(float('inf'))
        # MotorBackLeftW.setPosition(float('inf'))
        # MotorFrontRightW.setPosition(float('inf'))
        # MotorBackRightW.setPosition(float('inf'))
        
        
    
    def num_actions_available(self):
        # forward, left, right
        return 2
    
    def get_state(self):
        camera.saveImage('image.jpg', 20)
        img = Image.open("image.jpg")
        gray_image = ImageOps.grayscale(img)
        resize = T.Compose([
                T.Resize((36,64))  # height, width
                ,T.ToTensor()
            ])
        if self.finish:  # final image is black screen, cause Q-value at the end is 0
            # print(timestep_count)
            # print(f"starting: {self.starting}")
            # print(f"finish: {self.finish}")
            black_screen = torch.zeros_like(resize(gray_image).unsqueeze(0).to(device))
            return black_screen
        else:
            return resize(gray_image).unsqueeze(0).to(device)  # into (Batch, channel, H, W)
    
    def take_action(self, action):        
        robot_pos = robot_translateion_field.getSFVec3f()  # follow translation field of x, y, z
        robot_x, robot_y, robot_z = robot_pos
    
        # check if got pass trhough correct checkpoint oder
        # check current checkpoint, get vertices of before and after checkpoint regions
        current_region = self.CHECKPOINT_DICT[self.checkpoint]  # get a list of the region vertices
        next_region = self.CHECKPOINT_DICT[self.checkpoint+1]
        if self.checkpoint == 0:
            prev_region = self.CHECKPOINT_DICT[18]
        else:
            prev_region = self.CHECKPOINT_DICT[self.checkpoint-1]
        # track is defined as within the black areas and not on the borders. borders are ignored so dont have <= / >=
        # if pos in current checkpoint, reward = -1
        if (current_region[0] < robot_x < current_region[1]) and (current_region[2] < robot_z < current_region[3]):
            reward = -1
        # if pos in after checkpoint, reward = 100
        # if pos is in last region, reward = 2000, terminate
        elif (next_region[0] < robot_x < next_region[1]) and (next_region[2] < robot_z < next_region[3]):
            reward = 100
            self.checkpoint = self.checkpoint+1
            if self.checkpoint == 19:
                reward = 2_000
                self.done = True
                self.finish = True
        # if pos in before checkpoint, reward = -100
        # if the robot pass last checkpoint from checkpoint 0, -2000 and terminate
        elif (prev_region[0] < robot_x < prev_region[1]) and (prev_region[2] < robot_z < prev_region[3]):
            reward = -100
            if self.checkpoint == 0:
                reward = -2_000
                self.done = True
            else:
                self.checkpoint = self.checkpoint-1
        # if pos in white area, reward = -20
        elif (-0.17 <= robot_x <= 0) and (-0.17 <= robot_z <= 0.17):
            reward = -20
        elif (0 < robot_x < 0.08) and ((0.145 <= robot_z <= 0.17) or (-0.065 <= robot_z <= -0.04)):
            reward = -20
        elif (0.08 <= robot_x <= 0.17) and ((0.145 <= robot_z <= 0.17) or (0.04 <= robot_z <= 0.065) \
            or (-0.25 < robot_z <= -0.145)):
            reward = -20
        elif (0.17 < robot_x < 0.25) and ((0.04 <= robot_z <= 0.065) or (-0.25 < robot_z <= -0.145)):
            reward = -20
        elif (-0.29 <= robot_x <= -0.25) or (0.25 <= robot_x <= 0.29) \
            or (-0.29 <= robot_z <= -0.25) or (0.25 <= robot_z <= 0.29):
            reward = -20
        # else, terminate. Either by exiting boundary, blast off, or never follow 
        elif (robot_z > 0.29) or (robot_z < -0.29) or (robot_x > 0.29) or (robot_x < -0.29):  # exit boundary
            self.done = True
            reward = -2_000
        elif (robot_y > 0.04):  # blast off
            self.done = True
            reward = 0
        else:
            self.done = True
            reward = -2_000
            
        # reward is converted to a tensor to keep everything consistently tensor
        # sys.exit(1)
        return torch.tensor([reward], device=self.device)
        
        
class EpsilonGreedyStrategy():
    def __init__(self, start, end, decay):
        self.start = start
        self.end = end
        self.decay = decay
        
    def get_exploration_rate(self, current_step):
        return self.end + (self.start - self.end) * math.exp(-1. * current_step * self.decay)
    

class Agent():
    # device is for torch to use GPU or CPU
    def __init__(self, strategy, num_actions, device):
        self.current_step = 0
        self.strategy = strategy
        self.num_actions = num_actions  # always 2 in this case
        self.device = device  
        
    # policy_net is the DNN to learn the optimal policy
    def select_action(self, state, policy_net):
        rate = self.strategy.get_exploration_rate(self.current_step)
        self.current_step += 1

        if rate > random.random():  # if exploration rate > random number between 0-1
            action = random.randrange(self.num_actions)  
            # passing the tensor to either GPU/CPU
            return torch.tensor([action]).to(self.device) # explore      
        else:
            with torch.no_grad():  # use this if not using the model not for training. turn off grad tracking
                # passing the tensor to either GPU/CPU
                return policy_net(state).argmax(dim=1).to(self.device) # exploit


class ReplayMemory():
    # need to pass in some kind of number to initialise the size of the memory replay
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = deque(maxlen=self.capacity)  # using deque instead of just a list
    
    def push(self, experience):
        # this will push out the oldest (most left) element and add newest to the most right
        self.memory.append(experience)
        
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    # here to make sure that the if the replay memory doesnt have enough (< batch size), no sampling
    def can_provide_sample(self, batch_size):
        return len(self.memory) >= batch_size


# Within the nn package, there is a class called Module, which is the base class for all neural network modules, and so 
# our network and all of its layers will extend the nn.Module class.
class DQN(nn.Module):
    # DQN will receive screenshot-like images of the cart and pole environment as input, so to create a DQN object, 
    # we'll require the height and width of the image input that will be coming in to this model.
    def __init__(self, img_height, img_width):
        super().__init__()
        self.DENSE_INPUT = 256

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
        
        self.out = torch.nn.Linear(32, 2)
        
    # AKA forward pass.
    # all PyTorch neural networks require an implementation of forward()
    def forward(self, t):
        t = F.relu(self.conv_net(t))
        # print("pass conv net")
        t = t.flatten(start_dim=1)
        self.DENSE_INPUT = t.shape[1]  # 256
        t = F.relu(self.fc_net(t))
        # print("pass fc net")
        t = self.out(t)
        # print("pass output layer")
        return t


def extract_tensors(experiences):
    # Convert batch of Experiences to Experience of batches
    """
    lets say got 
    e1 = Experience(1,1,1,1)
    e2 = Experience(2,2,2,2)
    e3 = Experience(3,3,3,3)
    
    running the following code will give:
    Experience(state=(1, 2, 3), action=(1, 2, 3), next_state=(1, 2, 3), reward=(1, 2, 3))
    """
    batch = Experience(*zip(*experiences))

    # .cat() refers to concatenate. all of them is a tensor
    t1 = torch.cat(batch.state)
    t2 = torch.cat(batch.action)
    t3 = torch.cat(batch.reward)
    t4 = torch.cat(batch.next_state)
    return (t1,t2,t3,t4)


class QValues():
    device = torch.device("cuda")
    
    # using staticmethod so can call this 2 function without an instance of the class 
    # so can just QValues.get_current()
    # but cannot access the self.xxx stuff so need to create the device = ... line above
    @staticmethod
    def get_current(policy_net, states, actions):
        return policy_net(states).gather(dim=1, index=actions.unsqueeze(-1))  # gather is a pytorch stuff
    
    @staticmethod        
    def get_next(target_net, next_states):                
        # we dont want to pass the final state into the NN since Q-value of final/goal state is 0
        # first flatten the states, then find the states where the max value is 0 (cause black colour)
        # if found, in this final_state_location tensor, that element would be marked 'True'
        # the rest are marked 'False'
        final_state_locations = next_states.flatten(start_dim=1).max(dim=1)[0].eq(0).type(torch.bool)
        # print(f"final state locations: {final_state_locations}")
        non_final_state_locations = (final_state_locations == False)  # flipped of the final state locations
        non_final_states = next_states[non_final_state_locations]  # tensor of non final states
        batch_size = next_states.shape[0]  # size
        values = torch.zeros(batch_size).to(QValues.device)  # new tensor of zeroes with size = batch_size
        # set each non final states the max Q-value
        # final state will remain 0
        values[non_final_state_locations] = target_net(non_final_states).max(dim=1)[0].detach()  
        # print("values after passing target NN: ", values)
        return values

### MAIN #########################################################################################################################################################################################################################################
if __name__ == "__main__":
    ## Robot initialisation ##############################################################################
    # create the Robot instance.
    robot = Supervisor()
    robot_node = robot.getFromDef("MY_ROBOT")

    if robot_node is None:
        sys.stderr.write("No DEF MY_ROBOT node found in the current world file\n")
        sys.exit(1)
    robot_translateion_field = robot_node.getField("translation")
    robot_rotation_field = robot_node.getField("rotation")
    # get the time step of the current world.
    # can be changed in Worldinfo, and is in ms
    # so every passing timestep is 40ms in the simulation
    timestep = int(robot.getBasicTimeStep())

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

    # initialise the wheels' rotational motors
    # Max Speed = 10
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
    
    
    # keyboard control
    # keyboard = Keyboard()
    # keyboard.enable(timestep)
    # motor_cmd = {
    #     "87": (4, 4, 4, 4), # forward
    #     "83": (-4, -4, -4, -4),  # backwards
    #     "65": (-4, -4, 4, 4),  # turn left
    #     "68": (4, 4, -4, -4),  # turn right
    #     # "69": (0, 0, 0, 0)  # break
    # }
    def motorCommand(key):
        MotorFrontLeftW.setVelocity(key[0])
        MotorBackLeftW.setVelocity(key[1])
        MotorFrontRightW.setVelocity(key[2])
        MotorBackRightW.setVelocity(key[3])
    
    ## HYPERPARAMETERS ##############################################################################
    batch_size = 32
    gamma = 0.99
    eps_start = 1
    eps_end = 0.01
    eps_decay = 0.001
    target_update = 10
    memory_size = 100_000
    lr = 0.00025
    num_episodes = 75_000
    max_timestep = 2_250  # max run time of 1min and 30s. should finish the course ard 50s+
    
    PROCESSED_IMG_HEIGHT = 64  # same aspect ratio
    PROCESSED_IMG_WIDTH = 36

    device = torch.device("cuda")
    ENV = VectorRobotEnvManager(device)
    ENV.reset()
    strategy = EpsilonGreedyStrategy(eps_start, eps_end, eps_decay)  # choose to explore or exploit
    agent = Agent(strategy, ENV.num_actions_available(), device)
    memory = ReplayMemory(memory_size)
    # policy_net = DQN(PROCESSED_IMG_HEIGHT, PROCESSED_IMG_WIDTH).to(device)
    # target_net = DQN(PROCESSED_IMG_HEIGHT, PROCESSED_IMG_WIDTH).to(device)
    policy_net = ResNet.ResNet50(1, 2).to(device)  # in_channel = 1 cause grayscale, number of class = 2 cause 3 actions
    target_net = ResNet.ResNet50(1, 2).to(device)
    target_net.load_state_dict(policy_net.state_dict())  # set weights and bias to the same at the start for both NN
    target_net.eval()  # put the NN into eval mode, and not in training mode. Only use for inference
    optimizer = optim.Adam(params=policy_net.parameters(), lr=lr)
    
    # This class will be used to create instances of Experience objects that will get stored in and sampled from replay memory later.
    Experience = namedtuple(
        'Experience',
        ('state', 'action', 'next_state', 'reward')
    )
    # e = Experience(1,2,3,4)
    # output = Experience(state=1, action=2, next_state=3, reward=4)
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    # need 3 timestep for lift to be fully open
    reward_list = []
    loss_list = []
    init_count = 0
    while robot.step(timestep) != -1:
        # timestep = 1, cause reset once above
        # after lift fully up so wont obstruct the camera...
        if init_count > 2:
            # do main training loop.
            for episode in range(10):
                print(episode)
                # for every episode...
                # timestep = 4 here after lift goes up
                timestep_count = 0  # reset count for new ep
                if init_count > 3:  # dont want reset 2 times for the 1st ep
                    ENV.reset()
                else:
                    wait(robot)
                    init_count += 1
                wait(robot)  # wait for 15 timestep
                
                ENV.done = False
                ENV.finish = False
                ep_loss = 0
                ep_reward = 0
                
                state = ENV.get_state()
                timestep_count += 1  # when past into the while loop, 1 timestep have pass
                while timestep_count <= max_timestep:  # for each episode, as long as within max run time...
                    action = agent.select_action(state, policy_net)  # select an action base on the epsilon greedy
                    # print(f"e-greedy: {action}")
                    # print(f"time before moving: {robot.getTime()}")
                    move(action.item(), robot)  # execute action for 0.2s
                    # print(f"time after moving: {robot.getTime()}")
                    timestep_count += 6  # passing into move(1) + move for 0.2s(5) = 6 timesteps pass
                    reward = ENV.take_action(action)  # get reward for the action
                    ep_reward += reward  # accumulate the reward for this ep
                    # print(f"reward: {reward}")
                    next_state = ENV.get_state()  # get frame of next state
                    memory.push(Experience(state, action, next_state, reward))  # add to memory replay
                    state = next_state  # transition to next state
                    if memory.can_provide_sample(batch_size):  # first check if got enough replays
                        
                        experiences = memory.sample(batch_size)
                        # extract all the states, actions,... into their respective tensors
                        states, actions, rewards, next_states = extract_tensors(experiences)
                        # print(next_states.shape)
                        # states now has a size of (batch_size, channel, H, W)  
                        
                        current_q_values = QValues.get_current(policy_net, states, actions)  # get Q-value due to (states, actions)
                        # print(current_q_values.shape) # shape of [32,1]
                        next_q_values = QValues.get_next(target_net, next_states)  # get the optimal Q-value of next states
                        target_q_values = (next_q_values * gamma) + rewards  # Bellman Eqn
                        
                        # loss = F.mse_loss(current_q_values, target_q_values.unsqueeze(1))  # using MSE
                        loss = F.huber_loss(current_q_values, target_q_values.unsqueeze(1))  # using huber loss
                        ep_loss += loss  # accumulate the loss for this ep
                        optimizer.zero_grad()  # sets all weights and bias to 0. Else each backprog will accumulate
                        loss.backward()  # back prog
                        optimizer.step()  # updates the weights and bais
                        
                    if ENV.done:
                        print("terminate")
                        print(f"reward: {ep_reward} , loss: {ep_loss}")
                        reward_list.append(ep_reward)
                        loss_list.append(ep_loss)
                        break
                    
                    if episode % target_update == 0:  # check if need update target net
                        target_net.load_state_dict(policy_net.state_dict())
                    
                    if (timestep_count > max_timestep):
                        print("times up")
                        print(f"reward: {ep_reward} , loss: {ep_loss}")
                        reward_list.append(ep_reward)
                        loss_list.append(ep_loss)
                        
            print(f"end of {10} episodes")
            sys.exit(1)       
        else:
            init_count += 1
