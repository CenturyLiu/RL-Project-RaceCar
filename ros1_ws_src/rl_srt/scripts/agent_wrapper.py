#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import absolute_import, division, print_function
import os
import sys
import cv2
import numpy as np
#import h5py
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

# import services for ROS1-ROS2 communication
from racecar_rl_srv.srv import Image2Vel, ResetAgent, StepAgentMul, StoreWeights

 
class AgentWrapper(object):
    def __init__(self):
        self.image2vel = rospy.ServiceProxy('image2vel',Image2Vel)
        self.reset_agent = rospy.ServiceProxy('reset_agent',ResetAgent)
        self.step_agent = rospy.ServiceProxy('step_agent',StepAgentMul)
        self.store_agent_weight = rospy.ServiceProxy('store_agent_weight',StoreWeights)

    def reset(self):
        # reset the agent
        
        return self.reset_agent(Bool(data=True))

    def act(self, state):
        '''
        Get the action corresponding to the current state

        Parameters:
            state : [sensor_msgs/Image, geometry_msgs/Twist] 

        return value:
            velocity : Twist
            Velocity (i.e. action for this env) from the service    
        '''
        imgmsg = state[0]
        vel = state[1]
        response = self.image2vel(imgmsg,vel)
        return response.next_vel

    def step(self, state, action, reward, next_state, done, t):
        '''
        Store the tuple (s,a,r,s,d,t) for the rl-agent to learn
        '''
        print("stepping")
        if done:
            done_msg = Bool(data=True)
        else:
            done_msg = Bool(data=False)

        reward_msg = Float32(reward)
        timestep_msg = Float32(float(t))
        self.step_agent(state[0],state[1],action, reward_msg,next_state[0],next_state[1],done_msg,timestep_msg)
    
    def store_weights(self):
        # ask the agent to store weights
        return self.store_agent_weight(Bool(data=True))
