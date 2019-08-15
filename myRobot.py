#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
import numpy as np

# color definition
RED   = 1
GREEN = 2
BLUE  = 3
camera_preview = True

class MyRobot():
    def __init__(self):
        # bot name
        robot_name = rospy.get_param('~robot_name')
        self.name = robot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
        # camera subscriber
        self.img = None
        self.camera_preview = camera_preview
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        # move_base
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # mode
        self.mode = 0

    def lidarCallback(self, data):
        self.scan = data

    # Respect seigosan
    def find_rect_of_target_color(self, image, color_type): # r:0, g:1, b:2
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # red detection
        if color_type == RED:
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 20) | (h > 200)) & (s > 128)] = 255

        # blue detection
        if color_type == BLUE:
            lower_blue = np.array([130, 50, 50])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # green detection
        if color_type == GREEN:
            lower_green = np.array([75, 50, 50])
            upper_green = np.array([110, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)

        # get contours
        img, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        return rects

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        frame = self.img
        # 水平がx軸、垂直がy軸 rects = [x,y,height,width]
        rects = self.find_rect_of_target_color(frame, RED)
        print(rects)
        if self.camera_preview:
            cv2.imshow("Image window", self.img)
            cv2.waitKey(1)

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.name + "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def basic_move(self):
        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415*0.25)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415*1.5)
        self.setGoal(0,0.5,3.1415)
        self.setGoal(0,0.5,3.1415*1.75)
        
        self.setGoal(0.5,0,3.1415*1.5)
        self.setGoal(0.5,0,3.1415)
        self.setGoal(0.5,0,3.1415*1.25)
        
        self.setGoal(0,-0.5,3.1415)
        self.setGoal(0,-0.5,3.1415*0.5)
        self.setGoal(0,-0.5,0)

        self.mode = 1

        # できるだけ避ける
        # self.setGoal(-0.9,-0.5,0)
        # self.setGoal(-0.9,-0.5,3.1415*0.5)

        # self.setGoal(-0.5,0,0)
        # self.setGoal(-1.2,0,0)

        # self.setGoal(-0.95,0.5,0)
        # self.setGoal(-0.95,0.5,3.1415*0.3)

        # self.setGoal(0,1.3,3.1415*0.3)
        # self.setGoal(0,1.3,3.1415*1.5)
        # self.setGoal(0,0.5,3.1415*1.5)

        # self.setGoal(0,0.5,0)

        # self.setGoal(0,0.5,3.1415)

        # self.setGoal(0,0.5,3.1415*1.5)
        # self.setGoal(0,1.3,3.1415*1.5)
        # self.setGoal(0,1.3,3.1415*1.75)

        # self.setGoal(0.9,0.5,3.1415)
        # self.setGoal(0.9,0.5,3.1415*1.5)

        # self.setGoal(0.5,0,3.1415)
        # self.setGoal(1.2,0,3.1415)

        # self.setGoal(0.9,-0.5,3.1415)

        # self.setGoal(0,-0.5,3.1415)
        # self.setGoal(0,-0.5,3.1415/2)
        # self.setGoal(0,-0.5,0)

    def one_shot(self):
        print("red or green")

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps
        while not rospy.is_shutdown():
            if self.mode == 0:
                print("test")
                self.basic_move()
            elif self.mode == 1:
                self.one_shot()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('my_robot_run')
    bot = MyRobot()
    bot.strategy()

