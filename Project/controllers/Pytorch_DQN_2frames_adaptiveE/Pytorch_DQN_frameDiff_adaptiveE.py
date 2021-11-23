"""VectorSupervisorController controller."""
import math
import random
from collections import namedtuple
from collections import deque
from PIL import Image, ImageOps
import torch
# from torch._C import DoubleTensor, double
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision
import torchvision.transforms as T 

import sys
# import pfrl
import numpy as np
from VectorMove import move, wait
from controller import Supervisor
from controller import Camera, Motor, Keyboard


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
            +100 passing a checkpoint in proper order \n
            -100 going backwards a checkpoint \n
            -20 going off course like going into the white inner area or +4cm off the outlayer
            -2_000 for any termination
            +2_000 for completing the course
            
        Starting pos:
            (x, y, z) = (-0.21, 0.02, 0.21)
            
        Episode Termination:
            1) if >4cm from outer white layer \n
            2) if from starting position go backwards a region
            3) if goes into regions of the track not in the correct order
            4) Just in case, if robot somehow flies off
        """
        
    def __init__(self, device):
            
        self.device = device
        self.STARTING_POS = [-0.21, 0.02, 0.21]
        self.STARTING_ROT = [-0.999896, -0.0101921, 0.0101969, 1.5708]
        self.current_screen = None  # to store previous img when moving to next state
        self.starting = True
        self.done = False  # for termination
        self.finish = False # for finishing the course
        self.checkpoint = -1
        # self.CHECKPOINT_DICT = {  # store in checkpoint_number: [x1, x2, z1, z2]. x = Left to right, z = up to down
        #     # -1: [-0.25, -0.17, 0.17, 0.25],
        #     # 0: [-0.17, 0.0, 0.17, 0.25],
        #     0: [-0.25, 0.0, 0.17, 0.25],  #random start
        #     1: [0.0, 0.17, 0.17, 0.25],
        #     2: [0.17, 0.25, 0.17, 0.25],
        #     3: [0.17, 0.25, 0.145, 0.17],
        #     4: [0.17, 0.25, 0.065, 0.145],
        #     5: [0.08, 0.17, 0.065, 0.145],
        #     6: [0.0, 0.08, 0.065, 0.145],
        #     7: [0.0, 0.08, 0.04, 0.065],
        #     8: [0.0, 0.08, -0.04, 0.04],
        #     9: [0.08, 0.17, -0.04, 0.04],
        #     10: [0.17, 0.25, -0.04, 0.04],
        #     11: [0.17, 0.25, -0.065, -0.04],
        #     12: [0.17, 0.25, -0.145, -0.065],
        #     13: [0.08, 0.17, -0.145, -0.065],
        #     14: [0.0, 0.08, -0.145, -0.065],
        #     15: [0.0, 0.08, -0.25, -0.145],
        #     16: [-0.17, 0.0, -0.25, -0.17],
        #     17: [-0.25, -0.17, -0.25, -0.17],
        #     18: [-0.25, -0.17, -0.17, 0.21],
        #     19: [-0.25, -0.17, 0.21, 0.25]
        # }
        self.CHECKPOINT_DICT = {
            -1: [-0.25, -0.17, 0.17, 0.25],
            0: [-0.17, (-0.17/2), 0.17, 0.25],
            1: [(-0.17/2), 0.0, 0.17, 0.25],
            2: [0.0, (0.17/2), 0.17, 0.25],
            3: [(0.17/2), 0.17, 0.17, 0.25],
            4: [0.17, 0.25, 0.17, 0.25],
            5: [0.17, 0.25, 0.145, 0.17],
            6: [0.17, 0.25, 0.065, 0.145],
            7: [0.08, 0.17, 0.065, 0.145],
            8: [0.0, 0.08, 0.065, 0.145],
            9: [0.0, 0.08, 0.04, 0.065],
            10: [0.0, 0.08, -0.04, 0.04],
            11: [0.08, 0.17, -0.04, 0.04],
            12: [0.17, 0.25, -0.04, 0.04],
            13: [0.17, 0.25, -0.065, -0.04],
            14: [0.17, 0.25, -0.145, -0.065],
            15: [0.08, 0.17, -0.145, -0.065],  
            16: [0.0, 0.08, -0.145, -0.065],
            17: [0.0, 0.08, -0.17, -0.145],
            18: [0.0, 0.08, -0.25, -0.17],
            19: [-0.17, 0.0, -0.25, -0.17],  # random till here
            20: [-0.25, -0.17, -0.25, -0.17],
            21: [-0.25, -0.17, -0.17, 0.21],
            22: [-0.25, -0.17, 0.21, 0.25]
        }
        
        self.pass_count = {
            -1: 0,
            0: 0,
            1: 0,
            2: 0,
            3: 0,
            4: 0,
            5: 0,
            6: 0,
            7: 0,
            8: 0,
            9: 0,
            10: 0,
            11: 0,
            12: 0,
            13: 0,
            14: 0,
            15: 0,
            16: 0,
            17: 0,
            18: 0,
            19: 0,
            20: 0
        }
        
        self.straight={
            -1: 0,
            0: 0,
            1: 0,
            2: 0,
            3: 0,
            4: 6,
            5: 6,
            6: 12,
            7: 12,
            8: 6,
            9: 6,
            10: 0,
            11: 0,
            12: 6,
            13: 6,
            14: 12,
            15: 12,  
            16: 6,
            17: 6,
            18: 12,
            19: 12,
            20: 18,
            21: 18,
            22: 18
        }
      
    # for the start only  
    # def starting_reset(self):
    #     MotorFrontLeftW.setVelocity(0)
    #     MotorBackLeftW.setVelocity(0)
    #     MotorFrontRightW.setVelocity(0)
    #     MotorBackRightW.setVelocity(0)
    #     MotorFrontLeftW.setPosition(float('inf'))
    #     MotorBackLeftW.setPosition(float('inf'))
    #     MotorFrontRightW.setPosition(float('inf'))
    #     MotorBackRightW.setPosition(float('inf'))
    #     robot_translateion_field.setSFVec3f(self.STARTING_POS)
    #     robot_rotation_field.setSFRotation(self.STARTING_ROT)
    #     robot_node.resetPhysics()
       
    def reset(self):
        # reset both position and roation of robot
        # self.checkpoint = random.randint(-1, 18)
        # starting_x = round(random.uniform(self.CHECKPOINT_DICT[self.checkpoint][0], self.CHECKPOINT_DICT[self.checkpoint][1]), 2)
        # starting_z = round(random.uniform(self.CHECKPOINT_DICT[self.checkpoint][2], self.CHECKPOINT_DICT[self.checkpoint][3]), 2)
        # starting_pos = [starting_x, 0.02, starting_z]
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
        return
    
    def num_actions_available(self):
        # forward, left, right
        return 3
    
    def get_state(self):
        stacked_frames = deque(maxlen=2)
        
        img_taken = camera.saveImage('image1.jpg', 20)
        # if img not saved, keep trying to save image
        while img_taken != 0:
            img_taken = camera.saveImage('image1.jpg', 20)
            
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
            
            stacked_frames.append(resize(gray_image).unsqueeze(0).to(device))
        
        # torch.Size([1, 2, 36, 64]). As in the atari paper, added frames adds to the channel dimension
        return torch.cat(tuple(stacked_frames), dim=1)
        
    def euclidean_distance(next_region, robot_x, robot_z):
        x1, x2, z1, z2 = next_region
        robo = np.array((robot_x, robot_z))
        center = np.array(((x1+x2)/2 , (z1+z2)/2))  # next region's center
        # find the euclidean dist to from curr to next regions center
        return np.linalg.norm(robo - center)
    
    def new_reward_function(self, rotation, current_region, next_region, robot_x, robot_z):
        # cos(theta) part
        angle_reward = math.cos(abs(rotation - self.straight[self.checkpoint]) * math.radians(15)) 
        # print(f"angle: {abs(rotation - self.straight[self.checkpoint]) * (15)}")
        
        # distance from center paart
        p1=np.array([(current_region[0]+current_region[1])/2 ,(current_region[2]+current_region[3])/2])
        # print(p1)
        p2=np.array([(next_region[0]+next_region[1])/2 ,(next_region[2]+next_region[3])/2])
        # print(p2)
        p3=np.array([robot_x,robot_z])
        d=np.cross(p2-p1,p3-p1)/np.linalg.norm(p2-p1)
        # print(f"distance from center line: {d}")
        distance_reward = abs(d/8.0)
        
        return angle_reward-distance_reward
    
    def get_reward(self, rotation):        
        robot_pos = robot_translateion_field.getSFVec3f()  # follow translation field of x, y, z
        robot_x, robot_y, robot_z = robot_pos
    
        # check if got pass through correct checkpoint oder
        # check current checkpoint, get vertices of before and after checkpoint regions
        current_region = self.CHECKPOINT_DICT[self.checkpoint]  # get a list of the region vertices
        next_region = self.CHECKPOINT_DICT[self.checkpoint+1]
        if self.checkpoint == -1:
            prev_region = self.CHECKPOINT_DICT[21]
        else:
            prev_region = self.CHECKPOINT_DICT[self.checkpoint-1]
        # track is defined as within the black areas and not on the borders. borders are ignored so dont have <= / >=
        # if pos in current checkpoint, reward = -1
        if (current_region[0] < robot_x < current_region[1]) and (current_region[2] < robot_z < current_region[3]):
            # reward = -VectorRobotEnvManager.euclidean_distance(next_region, robot_x, robot_z)
            # reward = -1
            reward = VectorRobotEnvManager.new_reward_function(self, rotation, current_region, next_region, robot_x, robot_z)  # +ve reward
        # if pos in after checkpoint, reward = 100
        # if pos is in last region, reward = 2000, terminate
        elif (next_region[0] < robot_x < next_region[1]) and (next_region[2] < robot_z < next_region[3]):
            reward = 100
            self.pass_count[self.checkpoint] += 1
            self.checkpoint += 1
            if self.checkpoint == 22:
                reward = 2_000
                self.done = True
                self.finish = True
        # if pos in before checkpoint, reward = -100
        # if the robot pass last checkpoint from checkpoint 0, -2000 and terminate
        elif (prev_region[0] < robot_x < prev_region[1]) and (prev_region[2] < robot_z < prev_region[3]):
            reward = -100
            if self.checkpoint == -1:
                reward = -2_000
                self.done = True
            else:
                self.checkpoint = self.checkpoint-1
        # if pos in white area, reward = -20
        elif (-0.17 <= robot_x <= 0) and (-0.17 <= robot_z <= 0.17):
            # reward = -20
            reward = -2_000
            self.done = True
        elif (0 < robot_x < 0.08) and ((0.145 <= robot_z <= 0.17) or (-0.065 <= robot_z <= -0.04)):
            # reward = -20
            reward = -2_000
            self.done = True
        elif (0.08 <= robot_x <= 0.17) and ((0.145 <= robot_z <= 0.17) or (0.04 <= robot_z <= 0.065) \
            or (-0.25 < robot_z <= -0.145) or (-0.065 <= robot_z <= -0.04)):
            # reward = -20
            reward = -2_000
            self.done = True
        elif (0.17 < robot_x < 0.25) and ((0.04 <= robot_z <= 0.065) or (-0.25 < robot_z <= -0.145)):
            # reward = -20
            reward = -2_000
            self.done = True
        elif (-0.29 <= robot_x <= -0.25) or (0.25 <= robot_x <= 0.29) \
            or (-0.29 <= robot_z <= -0.25) or (0.25 <= robot_z <= 0.29):  # can still go out of the 50cm track by 4cm
            # reward = -20
            reward = -2_000
            self.done = True
        # else, terminate. Either by exiting boundary, blast off, or never follow 
        elif (robot_z > 0.29) or (robot_z < -0.29) or (robot_x > 0.29) or (robot_x < -0.29):  # exit boundary
            self.done = True
            reward = -2_000
        elif (robot_y > 0.04):  # blast off
            self.done = True
            reward = 0  # since is like kinda due to the freaking model's fault, not gonna give it a -ve reward
        else:  # going to other parts of the track and not following the checkpoint order
            self.done = True
            reward = -2_000
            
        # reward is converted to a tensor to keep everything consistently tensor
        return torch.tensor([reward], dtype=torch.float, device=self.device)
        # return torch.tensor([reward], device=self.device)
        
        
class EpsilonGreedyStrategy():
    def __init__(self, start, end, decay):
        self.start = start
        self.end = end
        self.decay = decay
        
    def get_exploration_rate(self, current_step):
        # the exploration rate is exponentially decaying to end value
        return self.end + (self.start - self.end) * math.exp(-1. * current_step * self.decay)
        # return math.exp(-1. * current_step * self.decay)  # ard 790 passes it shld be ard almost 0.
    

class Agent():
    # device is for torch to use GPU or CPU
    def __init__(self, strategy, num_actions, device):
        self.current_step = 0
        self.strategy = strategy
        self.num_actions = num_actions
        self.device = device  
        self.rate = None
        
    # policy_net is the DNN to train. target_net is to help calculate the loss.
    def select_action(self, state, policy_net, pass_count):
        self.rate = self.strategy.get_exploration_rate(pass_count)
        # self.rate = self.strategy.get_exploration_rate(self.current_step)
        # self.current_step += 1

        if self.rate > random.random():  # if exploration rate > random number between 0-1
            action = random.randrange(self.num_actions)  
            # passing the tensor to either GPU
            return torch.tensor([action]).to(self.device) # explore      
        else:
            with torch.no_grad():  # use .no_grad() if not using the model for training. To turn off grad tracking
                # passing the tensor to either GPU
                return policy_net(state).argmax(dim=1).to(self.device) # exploit
            
    def boltzmann_select_action(self, state, policy_net):
        with torch.no_grad():
            logits = policy_net(state).to(self.device)
            # print(logits)
            # print(torch.distributions.Categorical(logits=logits).probs)
            return torch.distributions.Categorical(logits=logits).sample()


class ReplayMemory():
    def __init__(self, capacity):
        self.capacity = capacity  # initialise size of replay memory
        # when full, in the new, out the old
        self.memory = deque(maxlen=self.capacity)  # using deque instead of just a list
    
    def push(self, experience):
        # If full, this will push out the oldest (most left) element and add newest to the most right
        self.memory.append(experience)
        
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    
    # here to make sure that the if the replay memory doesnt have enough (< batch size), no sampling
    def can_provide_sample(self, batch_size):
        return len(self.memory) >= batch_size


# Within the nn package, there is a class called Module, which is the base class for all neural network modules, and so 
# our network and all of its layers will extend the nn.Module class.
class DQN(nn.Module):
    # DQN will receive images from the camera as input, to create a DQN object
    def __init__(self):
        super().__init__()
        self.DENSE_INPUT = 256  # found by flattening and printing out the shape beforehand
        # self.relu = nn.PReLU()

        # conv part based on https://link.springer.com/article/10.1007/s42154-021-00151-3
        self.conv_net = torch.nn.Sequential(
            torch.nn.Conv2d(2, 32, (8,8), (4,4)),  # in channel, out, kernel, stride
            torch.nn.BatchNorm2d(32),
            torch.nn.ReLU(),
            torch.nn.Conv2d(32, 64, (4,4), (2,2)),  # in, out, kernel, stride
            torch.nn.BatchNorm2d(64),
            torch.nn.ReLU(),
            torch.nn.Conv2d(64, 64, (3,3), (1,1)),  # in, out, kernel, stride
            torch.nn.BatchNorm2d(64),
            torch.nn.ReLU()
        )
        
        self.fc_net = torch.nn.Sequential(
            torch.nn.Linear(self.DENSE_INPUT, 128), 
            torch.nn.ReLU(),
            torch.nn.Linear(128, 32),
            torch.nn.ReLU()
        )
        
        self.out = torch.nn.Linear(32, 3)
        
    # AKA forward pass.
    # all PyTorch neural networks require an implementation of forward()
    def forward(self, t):
        t = self.conv_net(t)
        t = t.flatten(start_dim=1)
        self.DENSE_INPUT = t.shape[1]  # 256
        t = self.fc_net(t)
        t = self.out(t)
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
    t1 = torch.cat(batch.state)  # 1 tensor of all the states
    t2 = torch.cat(batch.action)  # tensor([0, 1, 1], device='cuda:0')
    t3 = torch.cat(batch.reward)
    t4 = torch.cat(batch.next_state)
    return (t1,t2,t3,t4)

def extract_prio_tensors(experiences):
    state = []
    action = []
    reward = []
    next_state = []
    for i in range(3):
        state.append(experiences[i][0]['state'])
        action.append(experiences[i][0]['action'])
        reward.append(experiences[i][0]['reward'])
        next_state.append(experiences[i][0]['next_state'])
        
    t1 = torch.cat(tuple(state))
    t2 = torch.cat(tuple(action))
    t3 = torch.cat(tuple(reward))
    t4 = torch.cat(tuple(next_state))
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
    def get_next(target_net, next_states):  # basically if final state, Q-value = 0. Else, calulate it.
        # we dont want to pass the final state into the NN since Q-value of final/goal state is 0
        # first flatten the states, then find the states(images) where the max value is 0 (cause black colour)
        # if found, in this final_state_location tensor, that element would be marked 'True'
        # the rest are marked 'False'
        final_state_locations = next_states.flatten(start_dim=1).max(dim=1)[0].eq(0).type(torch.bool)
        non_final_state_locations = (final_state_locations == False)  # flipped of the final state locations
        non_final_states = next_states[non_final_state_locations]  # tensor of non final states
        batch_size = next_states.shape[0]  # size
        values = torch.zeros(batch_size).to(QValues.device)  # new tensor of zeroes with size = batch_size
        # set each non final states the max Q-value
        # final state will remain 0
        values[non_final_state_locations] = target_net(non_final_states).max(dim=1)[0].detach()  
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
    # so every passing timestep is timestep(msecond) in the simulation
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
    
    ## HYPERPARAMETERS ##############################################################################
    batch_size = 512
    gamma = 0.99
    eps_start = 1
    eps_end = 0.01
    eps_decay = 0.001
    target_update = 4
    memory_size = 100_000
    lr = 0.005
    num_episodes = 75_000
    max_timestep = 4_951  # max run time of 1min and 30s. Manual play finish the course ard 50s+.
    # everytime the robot moves, it take 0.2s (11 timesteps). so 450*11=4950. +1 at the start for 4951
    
    PROCESSED_IMG_HEIGHT = 36  # same aspect ratio
    PROCESSED_IMG_WIDTH = 64

    device = torch.device("cuda")  # want to use GPU
    ENV = VectorRobotEnvManager(device)
    ENV.reset()
    strategy = EpsilonGreedyStrategy(eps_start, eps_end, eps_decay)  # initialising e-greedy
    agent = Agent(strategy, ENV.num_actions_available(), device)
    memory = ReplayMemory(memory_size)
    # memory = pfrl.replay_buffers.PrioritizedReplayBuffer(capacity=memory_size)
    policy_net = DQN().to(device)
    # load the model
    # policy_net.load_state_dict(torch.load('18100_DQN.pth'))
    target_net = DQN().to(device)
    target_net.load_state_dict(policy_net.state_dict())  # set weights and bias to the same at the start for both NN
    target_net.eval()  # put the NN into eval mode, and not in training mode. Only use for inference
    # optimizer = optim.Adam(params=policy_net.parameters(), lr=lr)
    optimizer = optim.SGD(params=policy_net.parameters(), lr=lr, momentum=0.9)
    
    # This class will be used to create instances of Experience objects that will get stored in and sampled from replay memory later.
    # e = Experience(1,2,3,4)
    # output = Experience(state=1, action=2, next_state=3, reward=4)
    Experience = namedtuple(
        'Experience',
        ('state', 'action', 'next_state', 'reward')
    )
   
    # Main loop
    # need 3 timestep for lift to be fully open
    best_reward = 2000  # to check which ep the model reach a new best
    reward_list = []  # to append total reward at the end of each episode
    loss_list = []  # to append total loss at the end of each episode
    init_count = 0  # used for initialising head and lift
    
    # keyboard control
    keyboard = Keyboard()
    keyboard.enable(timestep)
    keys = {
        "87": (4, 4, 4, 4), # forward
        "83": (-4, -4, -4, -4),  # backwards
        "65": (-4, -4, 4, 4),  # turn left
        # "65": (0, 0, 10, 10),  # turn left
        "68": (4, 4, -4, -4),  # turn right
        # "68": (10, 10, 0, 0),  # turn right
        "69": (0, 0, 0, 0)  # break
    }
    rotation = 0
    while robot.step(timestep) != -1:

        ####################################################################
        # ONLY FOR TESTING
        # key = keyboard.getKey()
        # if str(key) in keys:
        #     if str(key) == "87":
        #         key = "0"
        #     elif str(key) == "65":
        #         key = "1"
        #     elif str(key) == "68":
        #         key = "2"
        #     next_state, rotation = move.rotation_move(key, robot, camera, rotation)
        #     key = None
        #     # motorCommand(motor_cmd[str(key)])
        # reward = ENV.get_reward(rotation)
        # print(reward)
        # RMB TO COMMENT OUT DURING TRAINING!!
        #####################################################################
        
        # timestep = 1, cause reset once above
        if init_count > 2:
            # after lift fully up so wont obstruct the camera...
            # do main training loop.
            for episode in range(num_episodes):
                print(episode)
                # for every episode...
                # timestep = 4 here after lift goes up, aka 0.08s simulation time passed
                
                # dont want reset 2 times for the 1st ep
                if init_count > 3:
                    ENV.reset()
                else:
                    wait(robot)
                    init_count += 1
                wait(robot)  # wait for 15 timestep
                
                # reset all the flags, counters
                ENV.starting = True
                ENV.done = False
                ENV.finish = False
                ENV.checkpoint = -1
                timestep_count = 0  # reset count within new ep
                ep_loss = 0
                ep_reward = 0
                rotation = 0
                
                # save camera image and then grayscale it before getting the difference
                state = ENV.get_state()
                timestep_count += 1  # 1 timestep to get state tensor
                while timestep_count <= max_timestep:  # for each episode, as long as within max run time...
                    pass_count = ENV.pass_count[ENV.checkpoint]  # a new step_count for calculating rate. Adaptive E
                    action = agent.select_action(state, policy_net, pass_count)  # select an action base on the epsilon greedy
                    # action = agent.boltzmann_select_action(state, policy_net)  # boltzmann exploration, https://youtu.be/PBWcN3M8Dbs?t=5032
                    # print(action)
                    next_state, rotation = move.rotation_move(action.item(), robot, camera, rotation)  # execute action for 0.2s
                    timestep_count += 11  # passing into move(1) + move for 0.2s(10) = 11 timesteps pass
                    reward = ENV.get_reward(rotation)  # get reward for the action
                    # print(reward)
                    ep_reward += reward.item()  # accumulate the reward for this ep
                    # next_state = ENV.get_state()  # get frame of next state
                    # print(next_state.shape)
                    memory.push(Experience(state, action, next_state, reward))  # add to replay memory
                    # memory.append(state=state, action=action, reward=reward, next_state=next_state)  # to prio replay mem
                    state = next_state  # transition to next state
                    if memory.can_provide_sample(batch_size):  # first check if got enough replays
                    # if len(memory) >= batch_size:  # for prio replay use
                        experiences = memory.sample(batch_size)
                        # extract all the states, actions,... into their respective tensors
                        states, actions, rewards, next_states = extract_tensors(experiences)
                        # states now has a size of (batch_size, channel, H, W)  
                        # print(states)
                        
                        # experiences = memory.sample(batch_size)
                        # states, actions, rewards, next_states = extract_prio_tensors(experiences)
                        
                        current_q_values = QValues.get_current(policy_net, states, actions)  # get Q-value due to (states, actions)
                        next_q_values = QValues.get_next(target_net, next_states)  # get the optimal Q-value of next states
                        target_q_values = (next_q_values * gamma) + rewards  # Bellman Eqn
                        
                        loss = F.mse_loss(current_q_values, target_q_values.unsqueeze(1))  # using MSE
                        # loss = F.huber_loss(current_q_values, target_q_values.unsqueeze(1))  # using huber loss
                        ep_loss += loss.item()  # accumulate the loss for this ep
                        optimizer.zero_grad()  # sets all weights and bias to 0. Else each backprog will accumulate
                        loss.backward()  # back prog
                        optimizer.step()  # updates the weights and bais   
                    
                    # if terminate or finish track, go to next ep
                    if ENV.done:  
                        # print("terminate")
                        # print(f"reward: {ep_reward} , loss: {ep_loss}")
                        reward_list.append(ep_reward)
                        loss_list.append(ep_loss)
                        break
                    
                    # if run out of time
                    if (timestep_count >= max_timestep):
                        # print("times up")
                        # print(f"reward: {ep_reward} , loss: {ep_loss}")
                        reward_list.append(ep_reward)
                        loss_list.append(ep_loss)
                        
                # save the best performing ep
                if ep_reward > best_reward:
                    best_reward = ep_reward
                    filename = 'Best_Ep_Reward.pth'
                    torch.save(policy_net.state_dict(), filename)
                    with open('best_ep_reward.txt', 'w') as f:
                        f.write("%s\n" % episode)
                        
                # check if need update target net
                    if episode % target_update == 0:  
                        target_net.load_state_dict(policy_net.state_dict()) 
                        
                # save the model, reward and loss after every 100 episodes
                if episode%100 == 0:
                    print(ENV.pass_count)
                    # print(f"agent steps: {agent.current_step}, rate: {agent.rate}")
                    filename = str(episode)+'_DQN.pth'
                    torch.save(policy_net.state_dict(), filename)
                        
                    with open('rewards.txt', 'w') as f:
                        for item in reward_list:
                            f.write("%s\n" % item)
                                
                    with open('loss.txt', 'w') as f:
                        for item in loss_list:
                            f.write("%s\n" % item)
                                            
            # pause when done
            robot.simulationSetMode(0)
            print(f"end of {num_episodes} episodes")
            # save the model
            torch.save(policy_net.state_dict(), 'DQN.pth')
            # write the rewards and loss to a file
            # rewrite everything inside
            with open('rewards.txt', 'w') as f:
                for item in reward_list:
                    f.write("%s\n" % item)
                    
            with open('loss.txt', 'w') as f:
                for item in loss_list:
                    f.write("%s\n" % item)
                   
        else:
            init_count += 1
        