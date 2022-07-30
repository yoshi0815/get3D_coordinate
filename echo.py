#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# echo.py: sample script to print ros message to terminal
# Author: Ravi Joshi
# Date: 2020/03/02

# import modules
import rospy
from ros_openpose.msg import Frame
import cv2
import sys
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

class depth_estimater:
    queue_size = 100
    fps = 100.
    delay = 0.5#1 / fps * 0.5

    def __init__(self):
        rospy.init_node('echo', anonymous=False)
        #rospy.init_node('depth_estimater', anonymous=True)
        self.bridge = CvBridge()
        frame_topic = rospy.get_param('~pub_topic')
        #sub_rgb = message_filters.Subscriber("/camera/color/image_raw",Image)
        sub_depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",Image)
        sub_pose = message_filters.Subscriber(frame_topic,Frame)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_depth], 100, 0.5)
        self.mf.registerCallback(self.ImageCallback)
        
    def ImageCallback(self, pose_data , depth_data):
        try:
            #color_image = self.bridge.imgmsg_to_cv2(rgb_data, 'rgb8')
            #color_image = cv2.resize(color_image, (int(color_image.shape[1]*0.25), int(color_image.shape[0]*0.25)))
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, '32FC1')#32FC1
            depth_image = cv2.resize(depth_image, (int(depth_image.shape[1]*1), int(depth_image.shape[0]*1)))
            text = [bodyPart.pixel for person in pose_data.persons for bodyPart in person.bodyParts]
            #print(depth_image.shape[0])
        except CvBridgeError as e:
            rospy.logerr(e)
        """
        color_image.flags.writeable = True
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        h, w, c = color_image.shape
        """

        textx = int(text[4].x)
        texty = int(text[4].y)
        #print(textx, texty)
        sum = 0.0
        #if depth_image.item(texty, textx) != 0: #ex.３次元座標とってみる
        if (textx != 0) and (texty != 0): 
            sum += depth_image.item(texty, textx)
        ave = sum*0.001
        x3 = textx
        y3 = texty
        point = np.array([x3,y3,ave])
        if ave==0 :
            print("ERROR: cannot get depth data ! ")
            return 

        xx=point[0]
        yy=point[1]
        zz=point[2]
        
        xx =float((xx-334.005)/618.136)*zz
        yy =float((yy-231.885)/618.370)*zz
        xx = '{:.3f}'.format(xx)
        yy = '{:.3f}'.format(yy)

        print("point:",[xx,yy,zz])

        ### ===== depth dubug =====
        """
        cv2.namedWindow("depth_image")
        cv2.imshow("depth_image", depth_image)
        cv2.waitKey(10)
        """
        ### =======================
"""
def callback(msg):
    text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    print(text[2].x)

    #rospy.loginfo('%s\n' % text)
def callbacktest(sub_pose, sub_depth):
    print("as")

def main():
    rospy.init_node('echo', anonymous=False)
    #de = depth_estimater()

    queue_size = 100
    fps = 100.
    delay = 0.5#1 / fps * 0.5

    # read the parameter from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')
    #sub_rgb = message_filters.Subscriber("/camera/color/image_raw",Image)
    sub_depth = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",Image)
    sub_pose = message_filters.Subscriber(frame_topic,Frame)
    #rospy.Subscriber(frame_topic, Frame, callback)
    mf = message_filters.ApproximateTimeSynchronizer([sub_pose, sub_depth], queue_size, delay)
    mf.registerCallback(callbacktest)
    rospy.spin()
"""

if __name__ == '__main__':
    #main()
    try:
        de = depth_estimater()
        rospy.spin()
    except rospy.ROSInterruptException: pass