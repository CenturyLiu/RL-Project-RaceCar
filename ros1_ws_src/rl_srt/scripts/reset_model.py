import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


def main():
    rospy.init_node("reset", anonymous = True)
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
    #reset_model = 
    '''
    state_msg = ModelState()
    state_msg.model_name = 'eufs'
    state_msg.pose.position.x = 13.0
    state_msg.pose.position.y = 17.0
    state_msg.pose.position.z = 0.1
    state_msg.pose.orientation.x = 0.0
    state_msg.pose.orientation.y = 0.0
    state_msg.pose.orientation.z = 0.0
    state_msg.pose.orientation.w = 0.0
    reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    #resp = set_state( state_msg )
    reset_world()
    '''
    reset_simulation()



if __name__ == "__main__":
    main()
