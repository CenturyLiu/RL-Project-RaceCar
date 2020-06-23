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
from racecar_rl_srv.srv import Image2Vel
from geometry_msgs.msg import Twist

class ImageListener(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.latest_imgmsg = rospy.wait_for_message('/mynteye/image_raw',Image)
        self.latest_image = self.bridge_object.imgmsg_to_cv2(self.latest_imgmsg,desired_encoding="bgr8")
        self.image_sub = rospy.Subscriber('/mynteye/image_raw',Image,self.camera_callback)

    def get_image(self):
        return self.latest_image

    def get_imgmsg(self):
        return self.latest_imgmsg
    def save_img(self,name):
        cv2.imwrite(name,self.latest_image)

    def camera_callback(self,data):
        try:
            self.latest_imgmsg = data
            self.latest_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            #print("got new image")
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('image_listener', anonymous=True)
    listener = ImageListener()
    #listener.save_img('0.jpg')
    image = listener.get_imgmsg()
    curr_vel = Twist()
    rospy.wait_for_service('image2vel')
    image2vel = rospy.ServiceProxy('image2vel',Image2Vel)
    response = image2vel(image,curr_vel)
    print("return velocity: linear_x = %f, angular_z = %f" % (response.next_vel.linear.x, response.next_vel.angular.z))
    
    

if __name__ == "__main__":
    main()



