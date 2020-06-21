from gym import spaces
import rl_robot_env
from gym.envs.registration import register
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import math
from image_process import Cone_detect
from std_srvs.srv import Empty
import time
import cv2
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

# The path is __init__.py of openai_ros, where we import the MovingCubeOneDiskWalkEnv directly
timestep_limit_per_episode = 1000 # Can be any Value

register(
        id='SrtTrainingEnv-v0',
        entry_point='rl_training_env:SrtTrainingEnv',
        #timestep_limit=timestep_limit_per_episode,
    )

class SrtTrainingEnv(rl_robot_env.Srt_robot_env):
    def __init__(self):
        
        # Only variable needed to be set here
        number_actions = rospy.get_param('/rl_srt/n_actions')
        self.action_space = spaces.Discrete(number_actions)
        max_height = np.inf
        max_width = np.inf
        min_height = 0
        min_width = 0
        self.max_speed = rospy.get_param('/rl_srt/max_speed')
        self.min_speed = rospy.get_param('rl_srt/min_speed')
        self.max_angular_vel = rospy.get_param('/rl_srt/max_angular_vel')
        self.min_angular_vel = rospy.get_param('/rl_srt/min_angular_vel')

        high = np.array([
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            max_height,
            max_width,
            self.max_speed,
            self.max_angular_vel
            ])
        low = np.array([
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            min_height,
            min_width,
            self.min_speed,
            self.min_angular_vel
            ])
        self.observation_space = spaces.Box(low, high)
        self.initial_speed = Twist()
        self.initial_speed.linear.x = 0.0
        self.initial_speed.linear.y = 0.0
        self.initial_speed.linear.z = 0.0
        self.initial_speed.angular.x = 0.0
        self.initial_speed.angular.y = 0.0
        self.initial_speed.angular.z = 0.0

        self.current_speed = self.initial_speed

        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        
        self.cone_detector = Cone_detect()

        self.step_count = 0


        #redefine the initial state of the model here
        self.state_msg = ModelState()
        self.state_msg.model_name = 'eufs'
        self.state_msg.pose.position.x = 13.0
        self.state_msg.pose.position.y = 17.0
        self.state_msg.pose.position.z = 0.1
        self.state_msg.pose.orientation.x = 0.0
        self.state_msg.pose.orientation.y = 0.0
        self.state_msg.pose.orientation.z = 0.0
        self.state_msg.pose.orientation.w = 0.0


        # Here we will add any init functions prior to starting the MyRobotEnv
        super(SrtTrainingEnv, self).__init__(self.state_msg)


    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.move_car(self.initial_speed)#no speed initially
        self.current_speed = self.initial_speed
        self.step_count = 0
        #time.sleep(5)

    def _init_env_variables(self):
        """
        Inits variables needed to be initialised each time we reset at the start
        of an episode.
        :return:
        """
        # TODO
        #self.current_speed = self.initial_speed
        #self.reset_simulation()
        pass
    '''
    def _set_action(self, action):
        Move the robot based on the action variable given

        # We have set 9 different actions for moving the racing car:'


                             action index table
                         add_speed   hold_speed   decrease_speed
        turn_left           0            1              2
        hold_direction      3            4              5
        turn_right          6            7              8

        if action == 0:#add speed and turn left
            self.accelerate()    
            self.turn_left()
        elif action == 1:
            self.turn_left()
        elif action == 2:
            self.decelerate()
            self.turn_left()
        elif action == 3:
            self.accelerate()
        elif action == 4:
            pass
        elif action == 5:
            self.decelerate()
        elif action == 6:
            self.accelerate()
            self.turn_right()
        elif action == 7:
            self.turn_right()
        elif action == 8:
            self.decelerate()
            self.turn_right()
        self.move_car(self.current_speed)
    '''
    def _set_action(self, action):
        #basic version, keep the car under constant linear velocity, only let it change the steer angle
        if self.current_speed.linear.x == 0:
            self.current_speed.linear.x = 5.0
        if action == 0:#add speed and turn left
            self.current_speed.angular.z = -1.0
        elif action == 1:
            self.current_speed.angular.z = -0.9
        elif action == 2:
            self.current_speed.angular.z = -0.8
        elif action == 3:
            self.current_speed.angular.z = -0.7
        elif action == 4:
            self.current_speed.angular.z = -0.6
        elif action == 5:
            self.current_speed.angular.z = -0.5
        elif action == 6:
            self.current_speed.angular.z = -0.4
        elif action == 7:
            self.current_speed.angular.z = -0.3
        elif action == 8:
            self.current_speed.angular.z = -0.1
        elif action == 9:
            self.current_speed.angular.z = 0.0
        elif action == 10:
            self.current_speed.angular.z = 0.1
        elif action == 11:
            self.current_speed.angular.z = 0.2
        elif action == 12:
            self.current_speed.angular.z = 0.3
        elif action == 13:
            self.current_speed.angular.z = 0.4
        elif action == 14:
            self.current_speed.angular.z = 0.5
        elif action == 15:
            self.current_speed.angular.z = 0.6
        elif action == 16:
            self.current_speed.angular.z = 0.7
        elif action == 17:
            self.current_speed.angular.z = 0.8
        elif action == 18:
            self.current_speed.angular.z = 0.9
        elif action == 19:
            self.current_speed.angular.z = 1.0
        self.move_car(self.current_speed)
        time.sleep(0.1)

    def _get_obs(self):
        """
        Here we define what sensor data of our robots observations
        To know which Variables we have acces to, we need to read the
        MyRobotEnv API DOCS
        :return: observations
        """

        #observation structure: [(blue_pos[0],blue_pos[1],blue_pos[2],blue_pos[3]),(red_pos[0],red_pos[1],red_pos[2],red_pos[3]),odom=> (x,y),speed(Twist type),(blue_count,red_count)]
        # TODO
        image = self.get_latest_image()
        odom = self.get_latest_odom()
        speed = self.current_speed
        blue_pos, red_pos, blue_bound, red_bound, blue_count, red_count = self.cone_detector.detect(image)
        
        for ii in range(len(blue_bound)):
            cv2.rectangle(image,(blue_bound[ii][1],blue_bound[ii][0]), (blue_bound[ii][3],blue_bound[ii][2]),(255, 0, 0), 3)
        for ii in range(len(red_bound)):
            cv2.rectangle(image,(red_bound[ii][1],red_bound[ii][0]), (red_bound[ii][3],red_bound[ii][2]),(0, 0, 255), 3)
        cv2.imshow("cone",image)
        cv2.waitKey(1)
        
        print("blue count == ")
        print(blue_count)
        print("red count == ")
        print(red_count)
        return [blue_pos,red_pos,odom,speed,(blue_count,red_count)]

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        # "observations" is a pair of coordinate frame
        #observations structure: [(blue_pos[0],blue_pos[1],blue_pos[2],blue_pos[3]),(red_pos[0],red_pos[1],red_pos[2],red_pos[3]),odom=> (x,y),speed(Twist type)]
        return not self.checkInLane(observations[2][0],observations[2][1])

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        reward = 0
        self.step_count += 1
        if done:
            reward = -1000 #in total 1000 steps for each iteration
        else:
            blue_count = observations[4][0]
            red_count = observations[4][1]
            if self.inboundary(observations[2][0],observations[2][1]):
                reward -= 0.1
            else:
                reward += 0.1
            if blue_count > 0 and red_count > 0:
                if abs(blue_count - red_count) >= 2:
                    reward -= 2
                else:
                    reward += 2
            elif blue_count == 0 and red_count > 0:
                reward -= 10 - red_count
            elif red_count == 0 and blue_count > 0:
                reward -= 10 - blue_count
            else:
                reward -= 10

        return reward
        
    # Internal TaskEnv Methods
    def accelerate(self):
        if self.current_speed.linear.x == 0.0:
            self.current_speed.linear.x = 2.0#quick start the car
        else:
            self.current_speed.linear.x *= 1.1#increase 10% each time
            if self.current_speed.linear.x > 10.0:
                self.current_speed.linear.x = 10.0

    def turn_left(self):
        self.current_speed.angular.z += 0.05
        if self.current_speed.angular.z > 1.0:
            self.current_speed.angular.z = 1.0

    def decelerate(self):
        self.current_speed.linear.x *= 0.9#decrease 10% each time
        if self.current_speed.linear.x < 0.3:
            self.current_speed.linear.x = 0.0

    def turn_right(self):
        self.current_speed.angular.z -= 0.05
        if self.current_speed.angular.z < -1.0:
            self.current_speed.angular.z = -1.0
