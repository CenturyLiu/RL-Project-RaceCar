from racecar_rl_srv.srv import Image2Vel

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Image2Vel, 'image2vel', self.image2vel_callback)
        self.bridge_object = CvBridge()

    def image2vel_callback(self, request, response):
        img_msg = request.image
        cv2_img = self.bridge_object.imgmsg_to_cv2(img_msg,desired_encoding="bgr8")
        cv2.imwrite('/home/shijiliu/rl_ros2_ws/src/racecar_rl_agent/racecar_rl_agent/test.png',cv2_img)

        vel = Twist()
        vel.linear.x = 1.0

        response.next_vel = vel
        self.get_logger().info('Return speed\nlinear_x == %f angular_z == %f' % (vel.linear.x,vel.angular.z))
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
