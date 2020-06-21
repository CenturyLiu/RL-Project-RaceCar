from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import rospy

class OdomListener():
    def __init__(self):
        self.latest_odom = rospy.wait_for_message('/gazebo/model_states',ModelStates)
        self.odom_sub = rospy.Subscriber('/gazebo/model_states',ModelStates,self.odom_callback)
    def odom_callback(self,data):
        self.latest_odom = data
    def get_car_pos(self):
        return (self.latest_odom.pose[2].position.x,self.latest_odom.pose[2].position.y)

def main():
    rospy.init_node("odom_listener", anonymous = True)
    odom_listener = OdomListener()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print odom_listener.get_car_pos()
        rate.sleep()

if  __name__ == "__main__":
    main()
