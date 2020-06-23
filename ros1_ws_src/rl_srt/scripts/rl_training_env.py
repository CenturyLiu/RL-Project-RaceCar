from gym import spaces
import rl_robot_env
from gym.envs.registration import register
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
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
        self.number_actions = 2#rospy.get_param('/rl_srt/n_actions')

        self.max_speed = 40.0#rospy.get_param('/rl_srt/max_speed')
        self.min_speed = 20.0#rospy.get_param('rl_srt/min_speed')
        self.max_steer = 1.0#rospy.get_param('/rl_srt/max_steer')
        self.min_steer = -1.0#rospy.get_param('/rl_srt/min_steer')
        self.move_time = 0.1#rospy.get_param('/rl_srt/move_time')

        high_action = np.array([
            self.max_steer,
            self.max_speed
            ])
        low_action = np.array([
            self.min_steer,
            self.min_speed
            ])



        self.action_space = spaces.Box(low_action,high_action,dtype=np.float32)#spaces.Discrete(number_actions)

        high_obs = np.ones((720,1280,3))#size of the image
        high_obs = np.inf * high_obs

        self.observation_space = spaces.Box(-high_obs, high_obs, dtype=np.float32)

        

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

    def _set_action(self, action):
        #basic version, keep the car under constant linear velocity, only let it change the steer angle
        self.current_speed = action
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

        '''
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
        '''


        imgmsg = self.get_lastest_imgmsg()
        speed_stamp = self.get_latest_speed()
        speed = Twist()
        speed.linear.x = speed_stamp.drive.speed
        speed.angular.z = speed_stamp.drive.steering_angle
        return [imgmsg,speed]

    def _is_done(self, observations):
        """
        Decide if episode is done based on the observations
        """
        # "observations" is a pair of coordinate frame
        #observations structure: [(blue_pos[0],blue_pos[1],blue_pos[2],blue_pos[3]),(red_pos[0],red_pos[1],red_pos[2],red_pos[3]),odom=> (x,y),speed(Twist type)]
        return False

    def _compute_reward(self, observations, done):
        """
        Return the reward based on the observations given
        """
        
        linear_speed = observations[1].linear.x
        if done:
            reward = -10 - 0.1 * linear_speed
        else:
            reward = 1 + 0.02 * linear_speed
        

        return reward

