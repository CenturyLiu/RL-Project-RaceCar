#!/usr/bin/env python


import gym
import time
import random
import time

import numpy as np

from gym import wrappers
import os
import cv2
from collections import deque

# ROS packages required
import rospy
import rospkg

# import our training environment
import rl_training_env

# import services for ROS1-ROS2 communication
from racecar_rl_srv.srv import Image2Vel, ResetAgent, StepAgentMul, StoreWeights

# import the wrapper for rl-agent
from agent_wrapper import AgentWrapper

DEBUG = True

if __name__ == '__main__':

    rospy.init_node('srt_rl_gym', anonymous=True, log_level=rospy.INFO)

    # Create the Gym environment
    print("start")
    env = gym.make('SrtTrainingEnv-v0')
    print("Gym environment done")
    rospy.loginfo ( "Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rl_srt')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo ( "Monitor Wrapper started")

    last_time_steps = np.ndarray(0)

    # Loads parameters from the ROS param server
    # Parameters are stored in a yaml file inside the config directory
    # They are loaded at runtime by the launch file
    nepisodes = 110
    nsteps = 1000

    

    start_time = time.time()
    rospack = rospkg.RosPack()
    
    num_agents = 1
    agent = AgentWrapper()

    # initialize memory for score
    scores_deque = deque(maxlen=100)
    scores = []

    solved = False

    # Starts the main training loop: the one about the episodes to do
    for i_episode in range(nepisodes):
        # Initialize the environment and get first state of the robot
        state = env.reset()
        agent.reset()
        score = np.zeros(num_agents)

        for t in range(nsteps):
            # get the action based on states
            action = agent.act(state)
            
            # interact with the environment
            next_state, reward, done, info = env.step(action)
            
            score += reward

            for ii in range(num_agents):
                agent.step(state, action, reward, next_state, done, t) # should be (states[ii]...)
                                                                        # here we only have one agent
            if DEBUG:
                print('Finish step')
                print(done)

            state = next_state
            if np.any(done):
                break

            if t >= nsteps - 1:
                solved = True

        score = score.mean()
        scores_deque.append(score)
        scores.append(score)
        if i_episode % 100 == 0:
            print('\rEpisode {}\tAverage Score: {:.2f}'.format(i_episode, np.mean(scores_deque)))
        if solved:
            print('\nEnvironment solved in {:d} episodes!\tAverage Score: {:.2f}'.format(i_episode, np.mean(scores_deque)))
            agent.store_weights()
            break
    print('exit training !')
    cv2.destroyAllWindows()

    env.close()
