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

camera_preview = False

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

    def lidarCallback(self, data):
        self.scan = data
        print("scan")

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("image")
        except CvBridgeError as e:
            print(e)
        if self.camera_preview:
            cv2.imshow("Image window", self.img)
            cv2.waitKey(1)

    # def calcTwist(self):
    #     value = random.randint(1,1000)
    #     if value < 250:
    #         x = 0.2
    #         th = 0
    #     elif value < 500:
    #         x = -0.2
    #         th = 0
    #     elif value < 750:
    #         x = 0
    #         th = 1
    #     elif value < 1000:
    #         x = 0
    #         th = -1
    #     else:
    #         x = 0
    #         th = 0
    #     twist = Twist()
    #     twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
    #     twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
    #     return twist

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

    def strategy(self):
        # r = rospy.Rate(1) # change speed 1fps

        # target_speed = 0
        # target_turn = 0
        # control_speed = 0
        # control_turn = 0

        # while not rospy.is_shutdown():
        #     twist = self.calcTwist()
        #     print(twist)
        #     self.vel_pub.publish(twist)

        #     r.sleep()
        r = rospy.Rate(5) # change speed 5fps

        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415/2)
        
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415)
        
        self.setGoal(-0.5,0,-3.1415/2)
        
        self.setGoal(0,-0.5,0)
        self.setGoal(0,-0.5,3.1415)


if __name__ == '__main__':
    rospy.init_node('my_robot_run')
    bot = MyRobot()
    bot.strategy()

