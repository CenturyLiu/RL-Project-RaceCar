#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from __future__ import absolute_import, division, print_function
import os
import sys
import cv2
import numpy as np
import h5py
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageListener(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.latest_image = self.bridge_object.imgmsg_to_cv2(rospy.wait_for_message('/mynteye/image_raw',Image),desired_encoding="bgr8")
        self.image_sub = rospy.Subscriber('/mynteye/image_raw',Image,self.camera_callback)

    def get_image(self):
        return self.latest_image

    def save_img(self,name):
        cv2.imwrite(name,self.latest_image)

    def camera_callback(self,data):
        try:
            self.latest_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            print("got new image")
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('image_listener', anonymous=True)
    listener = ImageListener()
    listener.save_img('0.jpg')

if __name__ == "__main__":
    main()



