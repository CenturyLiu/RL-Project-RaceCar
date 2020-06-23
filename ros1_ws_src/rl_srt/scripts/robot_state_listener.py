import rospy
from ackermann_msgs.msg import AckermannDriveStamped

class RobotStateListener():
    def __init__(self):
        self.latest_state = AckermannDriveStamped()#rospy.wait_for_message('/robot_control/command',AckermannDriveStamped)
        self.state_sub = rospy.Subscriber('/robot_control/command',AckermannDriveStamped,self.state_callback)

    def get_car_state(self):
        #print self.latest_state.drive.speed
        return self.latest_state

    def state_callback(self,data):
        self.latest_state = data

def main():
    rospy.init_node("robot_state_listener", anonymous = True)
    state = RobotStateListener()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state.get_car_state()
        rate.sleep()

if  __name__ == "__main__":
    main()
