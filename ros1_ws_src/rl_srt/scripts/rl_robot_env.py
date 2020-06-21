from openai_ros import modified_robot_gazebo_env
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_state_listener import RobotStateListener
from image_listener import ImageListener
from odom_listener import OdomListener
from inLane import mypolygon
import rospkg

import rospy
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

class Srt_robot_env(modified_robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all Robot environments.
    """

    def __init__(self, robot_pose):
        """Initializes a new Robot environment.
        """
        # Variables that we give through the constructor.

        # Internal Vars
        self.controllers_list = ['left_front_shock_controller','left_steering_joint_controller', 'left_front_axle_controller', 'right_front_shock_controller', 'right_front_shock_controller', 'right_front_axle_controller','left_rear_shock_controller','left_rear_axle_controller','right_steering_joint_controller', 'right_rear_shock_controller','right_rear_axle_controller','joint_read_state_controller']
        print("start init custom robot env")
        self.robot_name_space = "eufs"

        reset_controls_bool = False
        
        # We launch the init function of the Parent Class robot_gazebo_env.RobotGazeboEnv
        
        super(Srt_robot_env, self).__init__(controllers_list = self.controllers_list,robot_name_space=self.robot_name_space,
                                                reset_controls=reset_controls_bool, robot_pose = robot_pose)#controllers_list=self.controllers_list,

    # Methods needed by the RobotGazeboEnv
    # ----------------------------
        print("Execute Methods needed by the RobotGazeboEnv")
    
        #self.gazebo.unpauseSim()
        #self.controllers_object.reset_controllers()
        
        #The new control system takes only the input image and output the commands of Twist type
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1) #velocity publisher
        print("create listeners")
        self.image_listener = ImageListener()#the listener for the image topic. 
        print("created image listeners")
        self.odom_listener = OdomListener()#the listener for the car model
        print("created odom listeners")
        #self.state_listener = RobotStateListener()#listener for the robot's speed and angle
        print("create state listeners")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('rl_srt')
        self.done_detect = mypolygon(pkg_path + "/scripts")

        #self.gazebo.pauseSim()

        print("finish init custom robot env")

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        # TODO
        return True
    
    # Methods that the TrainingEnvironment will need to define here as virtual
    # because they will be used in RobotGazeboEnv GrandParentClass and defined in the
    # TrainingEnvironment.
    # ----------------------------
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        raise NotImplementedError()
    
    
    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()
        
    # Methods that the TrainingEnvironment will need.
    # ----------------------------
    def move_car(self, speed):
        self.vel_pub.publish(speed)

    def get_latest_image(self):
        return self.image_listener.get_image()

    def get_latest_odom(self):
        return self.odom_listener.get_car_pos()

    def checkInLane(self,x,y):
        return self.done_detect.inLane(x,y)

    def inboundary(self,x,y):
        return self.done_detect.closeToBoundary(x,y)

    
