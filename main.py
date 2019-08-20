#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseActionResult
#from actionlib_msgs.msg import GoalStatusArray
import subprocess
import tf
import tf2_ros
import math
import time
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class turtleSim:
    def __init__(self):
      rospy.init_node('move_turtlesim', anonymous=True)
      self.twist_pub = rospy.Publisher('Rulo/cmd_vel', Twist, queue_size=1000)
      
      self.tfBuffer = tf2_ros.Buffer()
      listener = tf2_ros.TransformListener(self.tfBuffer)

      #各データの初期化
      twist = Twist()
      twist.linear.x = 0.0
      twist.linear.y = 0.0
      twist.linear.z = 0.0
      twist.angular.x = 0.0
      twist.angular.y = 0.0
      twist.angular.z = 0.0
      self.twist_pub.publish(twist)

      self.status = 0
      self.status2 = 0
      self.flag = 1
      #self.map_flag = 0
      self.banana_flag = 0
      self.apple_flag = 0
      #self.goal_status_id = -1
      self.object_flag = -1 # 0:banana, 1:apple
      self.object_goal_flag = 0
      self.goal_reach_flag = -1

      self.usleft = 0.0
      self.usright = 0.0
      self.optleft = 0.0
      self.optright = 0.0
      self.lidarscans = [0.0] * 360
      
      #self.result = [0.0] * 5
      self.detect = [0.0]
      self.text_object = ""
      self.text_face = ""

      self.object = -1.0
      
      self.distance = 0.0
      self.distance_max = 0.0
      self.distance_goal = 1.0
      self.x = 0.0
      self.x_max = 0.0
      self.y = 0.0
      self.y_max = 0.0
      self.goal_x = 0.0
      self.goal_y = 0.0
      self.count_zero = 0
      self.map_area = 0.0
      self.map_area_threshold = 14.4
      self.area_flag = 0
      self.box_size = 4
      self.box_100_count = 0
      self.box_threshold = 10
      self.goal_z = 0.0
      self.goal_w = 0.0

      self.map_result = [0.0] * 5

      self.banana_point_x = 100.0
      self.banana_point_y = 100.0
      self.apple_point_x = 100.0
      self.apple_point_y = 100.0
      self.cable_point_x = 100.0
      self.cable_point_y = 100.0
      self.instant_goal_x = 100.0
      self.instant_goal_y = 100.0
      #self.save_banana_x = 100.0
      #self.save_banana_y = 100.0
      #self.save_apple_x = 100.0
      #self.save_apple_y = 100.0

      self.rulo_cable_distance = 1.0

      self.localizaiton_x = 0.0
      self.localization_y = 0.0

      self.voice = [0.0, 0.0]

      #対象物発見後、初期位置を目的地として発行するため
      self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1000)
      self.cable_pub = rospy.Publisher('/custom_point_cloud', PointCloud2, queue_size=1)
      #us_left, us_right:正面 opt_left, opt_right：両脇, scan:上　のセンサー
      rospy.Timer(rospy.Duration(0.1), self.timerCallback)
      rospy.Subscriber('mobile_base/event/us_left',LaserScan, self.get_us_left)
      rospy.Subscriber('mobile_base/event/us_right',LaserScan, self.get_us_right)
      rospy.Subscriber('mobile_base/event/opt_left',LaserScan, self.get_opt_left)
      rospy.Subscriber('mobile_base/event/opt_right',LaserScan, self.get_opt_right)
      rospy.Subscriber('scan', LaserScan, self.get_lidarscans)
      #ssf_result:クラス番号、枠の対角線の座標、クラスの確率
      #rospy.Subscriber('ssd_result', Float32MultiArray, self.get_result)
      rospy.Subscriber('detect', Float32MultiArray, self.get_detect)
      #rospy.Subscriber('/map', OccupancyGrid, self.mapCallback)
      rospy.Subscriber('/map_result', Float32MultiArray, self.get_map)
      #rospy.Subscriber('/mobile_base/status', GoalStatusArray, self.get_goal_status)
      rospy.Subscriber('/banana_point', Float32MultiArray, self.get_banana_point)
      rospy.Subscriber('/apple_point', Float32MultiArray, self.get_apple_point)
      rospy.Subscriber('/cable_point', Float32MultiArray, self.get_cable_point)
      rospy.Subscriber('/voice', Float32MultiArray, self.get_voice)
      rospy.Subscriber('/face', Float32MultiArray, self.get_face)
      rospy.Subscriber('move_base/result', MoveBaseActionResult, self.get_move_result)
      
    def jtalk(self, t):
      open_jtalk=['open_jtalk']
      mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
      htsvoice=['-m','/home/ubuntu/MMDAgent_Example-1.6/Voice/mei/mei_normal.htsvoice']
      speed=['-r','1.0']
      outwav=['-ow','open_jtalk.wav']
      cmd=open_jtalk+mech+htsvoice+speed+outwav
      c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
      c.stdin.write(t)
      c.stdin.close()
      c.wait()
      aplay = ['aplay','-q','open_jtalk.wav']
      wr = subprocess.Popen(aplay)

    def get_us_left(self, usleft):
      self.usleft = usleft.ranges[0]

    def get_us_right(self, usright):
      self.usright = usright.ranges[0]

    def get_opt_left(self, optleft):
      self.optleft = optleft.ranges[0]

    def get_opt_right(self, optright):
      self.optright = optright.ranges[0]
 
    def get_lidarscans(self, lidar):
      self.lidarscans = lidar.ranges

    """
    def get_result(self, result):
      if(self.map_flag == 1):
        return
      self.result = result.data
    """

    def get_banana_point(self, banana_point):
      self.banana_point_x = banana_point.data[0]
      self.banana_point_y = banana_point.data[1]
      self.banana_flag = 1

    def get_apple_point(self, apple_point):
      self.apple_point_x = apple_point.data[0]
      self.apple_point_y = apple_point.data[1]
      self.apple_flag = 1

    def get_cable_point(self, cable_point):
      self.cable_point_x = cable_point.data[0]
      self.cable_point_y = cable_point.data[1]
      #print("get_cable")

    def get_face(self, face):
      if(face.data[0] == 0):
        self.jtalk("なんだよ柏木")
      else:
        self.jtalk("なんだよ篠原")

    def get_move_result(self, move_result):
      self.goal_reach_flag = move_result.status.status
      print("self.goal_reach_flag:", self.goal_reach_flag)
      if(self.goal_reach_flag == 4):
        sa_time = 0.0
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        start = time.time()
        while(sa_time < 3.0):
          twist.angular.z = 0.3
          self.twist_pub.publish(twist)
          sa_time = time.time() - start
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.instant_goal_x
        goal.pose.position.y = self.instant_goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        print("新しいゴールをセットしました")
        self.goal_reach_flag = -1

    def get_detect(self, detect):
      if(self.flag == 1):
        return

      if((self.goal_reach_flag == 3) and (self.object_goal_flag == 0)):
	time.sleep(3)
        self.jtalk("ゴールにつきました")
        time.sleep(3)
        self.goal_reach_flag = 0
        self.detect = detect.data

      if((self.detect[0] == 101.0) or (self.detect[0] == 102.0)):
        self.flag = 1
        print("face detect")
        if(self.detect[0] == 101):
          self.object = 1112.0 #banana
          self.text_face = "しのはらさんを認識しました"
          self.object_flag = 0
          self.text_object = "バナナを発見しました"
        else:
          #self.object = 918.0 #apple
          self.object = 1112.0
          self.text_face = "かしわぎくんを認識しました"
          self.object_flag = 1
          self.text_object = "りんごを発見しました"
      else:
        return
      self.jtalk(self.text_face)
      time.sleep(3)
      sa_time = 0.0
      start = time.time()
      while(sa_time < 4.2):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.7
        self.twist_pub.publish(twist)
        sa_time = time.time() - start
        self.status = 2

    def get_map(self, map_result):
      self.map_result = map_result.data

    def get_voice(self, voice):
      self.voice = list(voice.data)
      print("self.voice:", self.voice)
      if(self.voice == [3.0, 1.0]):
        print("test")
        if((self.map_result[0] < self.map_area_threshold) and (self.status == 0)):
          self.jtalk("地図を作成中です")
        elif((self.map_result[0] >= self.map_area_threshold) and (self.status == 1) and (self.object_flag == -1)):
          self.jtalk("ゴールに向かっています")
        elif((self.status == 2) and (self.banana_flag != 1) and (self.object_flag == 0)):
          self.jtalk("バナナを探しています")
        elif((self.status == 2) and (self.apple_flag != 1) and (self.object_flag == 1)):
          self.jtalk("りんごを探しています")
        elif((self.status == 1) and (self.object_flag == 0) and (self.banana_flag == 1) and (self.object_goal_flag == 1)):
          self.jtalk("バナナに向かっています")
        elif((self.status == 1) and (self.object_flag == 1) and (self.apple_flag == 1) and (self.object_goal_flag == 1)):
          self.jtalk("りんごに向かっています")
        elif((self.object_flag == 0) and (self.banana_flag == 1) and (self.object_goal_flag == 0)):
          self.jtalk("バナナに到着しました")
        elif((self.object_flag == 1) and (self.apple_flag == 1) and (self.object_goal_flag == 0)):
          self.jtalk("りんごに到着しました")
        else:
          return
      elif(self.voice == [4.0, 1.0]):
        if(self.map_result[0] >= self.map_area_threshold):
          self.jtalk("100%です")
        else:
          self.jtalk(str(int((self.map_result[0] / self.map_area_threshold) * 100)) + "%" + "です")
      elif(self.voice == [2.0, 1.0]):
        if(self.object_flag == 1):
          self.status = 1
          self.object_flag =-1
          self.object_goal_flag = 1
          rate = rospy.Rate(10.0)
          time.sleep(8)
          goal = PoseStamped()
          goal.header.stamp = rospy.Time.now()
          goal.header.frame_id = "map"
          goal.pose.position.x = self.localization_x
          goal.pose.position.y = self.localization_y
          goal.pose.position.z = 0.0
          goal.pose.orientation.x = 0.0
          goal.pose.orientation.y = 0.0
          goal.pose.orientation.z = 0.0
          goal.pose.orientation.w = 1.0
          self.goal_pub.publish(goal)
          time.sleep(5)
          self.jtalk("対象をバナナに変更します")
          time.sleep(2)
          self.goal_reach_flag = -1
          self.status = 2
          self.object_flag = 0
        elif(self.object_flag == -1):
          if(self.status2 == 0 or self.flag == 0):
            return
          self.jtalk("バナナを探します")
          time.sleep(2)
          self.goal_reach_flag = -1
          self.object_flag = 0
          self.status = 2
          self.object_goal_flag = 1
        else:
          return
      elif(self.voice == [2.0, 0.0]):
        if(self.object_flag == 0):
          self.status = 1
          self.object_flag = -1
          self.object_goal_flag = 0
          rate = rospy.Rate(10.0)
          time.sleep(8)
          goal = PoseStamped()
          goal.header.stamp = rospy.Time.now()
          goal.header.frame_id = "map"
          goal.pose.position.x = self.localization_x
          goal.pose.position.y = self.localization_y
          goal.pose.position.z = 0.0
          goal.pose.orientation.x = 0.0
          goal.pose.orientation.y = 0.0
          goal.pose.orientation.z = 0.0
          goal.pose.orientation.w = 1.0
          self.goal_pub.publish(goal)
          time.sleep(5)
          self.jtalk("バナナの探索を中止します")
          time.sleep(2)
        else:
          return
      elif((self.voice == [1.0, 1.0])):
        if(self.object_flag == 0):
          self.status = 1
          self.object_flag =-1
          rate = rospy.Rate(10.0)
          time.sleep(8)
          goal = PoseStamped()
          goal.header.stamp = rospy.Time.now()
          goal.header.frame_id = "map"
          goal.pose.position.x = self.localization_x
          goal.pose.position.y = self.localization_y
          goal.pose.position.z = 0.0
          goal.pose.orientation.x = 0.0
          goal.pose.orientation.y = 0.0
          goal.pose.orientation.z = 0.0
          goal.pose.orientation.w = 1.0
          self.goal_pub.publish(goal)
          time.sleep(5)
          self.jtalk("対象をりんごに変更します")
          time.sleep(2)
          self.goal_reach_flag = -1
          self.object_flag = 1
          self.status = 2
          self.object_goal_flag = 1
        elif(self.object_flag == -1):
          if(self.status2 == 0 or self.flag == 0):
            return
          self.jtalk("りんごを探します")
          time.sleep(2)
          self.goal_reach_flag = -1
          self.object_flag = 1
          self.status = 2
          self.object_goal_flag = 1
        else:
          return
      elif(self.voice == [1.0, 0.0]):
        if(self.object_flag == 1):
          self.status = 1
          self.object_flag = -1
          self.object_goal_flag = 0
          rate = rospy.Rate(10.0)
          time.sleep(8)
          goal = PoseStamped()
          goal.header.stamp = rospy.Time.now()
          goal.header.frame_id = "map"
          goal.pose.position.x = self.localization_x
          goal.pose.position.y = self.localization_y
          goal.pose.position.z = 0.0
          goal.pose.orientation.x = 0.0
          goal.pose.orientation.y = 0.0
          goal.pose.orientation.z = 0.0
          goal.pose.orientation.w = 1.0
          self.goal_pub.publish(goal)
          self.goal_reach_flag = -1
          time.sleep(5)
          self.jtalk("りんごの探索を中止します")
          time.sleep(2)
        else:
          return
      else:        
        return 

    def timerCallback(self, event):
      #publish cable position
      POINTS = []
      iterations = 10
      for i in range(-iterations,iterations+1):
            for j in range(-iterations,iterations+1):
                POINTS.append([self.cable_point_x+i*0.01,self.cable_point_y+j*0.01,0.0])
      point_cloud = pc2.create_cloud_xyz32(Header(frame_id='/map'), POINTS)
      self.cable_pub.publish(point_cloud)

      twist = Twist()
      twist.linear.x = 0.0
      twist.linear.y = 0.0
      twist.linear.z = 0.0
      twist.angular.x = 0.0
      twist.angular.y = 0.0
      twist.angular.z = 0.0
      self.count = 0
      self.countleft = 0
      self.countright = 0
      rate = rospy.Rate(10.0)

      while True:
        try:
          t = self.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time())
          break
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          rate.sleep()

      self.localization_x = t.transform.translation.x
      self.localization_y = t.transform.translation.y

      if((self.goal_reach_flag == 3) and (self.object_goal_flag == 1) and (self.object_flag == 0)):
        self.jtalk("バナナに到着しました。")
        self.goal_reach_flag = 0
        self.object_goal_flag = 0
      if((self.goal_reach_flag == 3) and (self.object_goal_flag == 1) and (self.object_flag == 1)):
        self.jtalk("りんごに到着しました。")
        self.goal_reach_flag = 0
        self.object_goal_flag = 0

      """
      if(self.object_goal_flag == 1):
        if(self.object_flag == 0):
          distance_x = self.localization_x - self.save_banana_x
          distance_y = self.localization_y - self.save_banana_y
          distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
          print("distance:",distance)
          if(distance <= 0.70):
            print("goal_banana")
            time.sleep(10)
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.localization_x
            goal.pose.position.y = self.localization_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            time.sleep(5)
            self.jtalk("バナナに到着しました。")
            self.object_goal_flag = 0
        if(self.object_flag == 1):
          distance_x = self.localization_x - self.save_apple_x
          distance_y = self.localization_y - self.save_apple_y
          distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
          print("distance:",distance)
          if(distance <= 0.70):
            print("goal_apple")
            time.sleep(10)
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.localization_x
            goal.pose.position.y = self.localization_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            time.sleep(5)
            self.jtalk("りんごに到着しました")
            self.object_goal_flag = 0
      """

      if(self.status == 1):
        return
      
      for i in range(120, 240):
        if((self.lidarscans[i] > 0.0) and (self.lidarscans[i] < 0.25)):
          self.count += 1
          if(i <= 180):
            self.countright += 1
          else:
            self.countleft += 1

      rulo_cable_distance_x = self.localization_x - self.cable_point_x
      rulo_cable_distance_y = self.localization_y - self.cable_point_y
      self.rulo_cable_distance = math.sqrt(rulo_cable_distance_x ** 2 + rulo_cable_distance_y ** 2)
      #print("self.rulo_cable_distance:", self.rulo_cable_distance)

      if((self.usleft < 0.15) or (self.usright < 0.15)):
        self.status = 1
        #右が壁に近いとき
        if(self.usright <= self.usleft):
          #print("usleft")
          while((self.usright < 0.30) or (self.usleft < 0.20)):
            twist.angular.z = 0.3
            self.twist_pub.publish(twist)
          self.status = self.status2
          #print("turnleft")
        #左が壁に近いとき
        else:
          #print("usright")
          while((self.usleft < 0.30) or (self.usright < 0.20)):
            twist.angular.z = -0.3
            self.twist_pub.publish(twist)
          self.status = self.status2
          #print("turnright")
      #optが壁を検知したとき
      elif(self.optleft <= 0.15):
          #print("optleft")
          self.status = 1
          while((self.optleft < 0.20) or (self.optright < 0.15)):
            twist.angular.z = -0.3
            self.twist_pub.publish(twist)
          self.status = self.status2
          #print("turnleftleft")
      elif(self.optright <= 0.15):
          #print("optright")
          self.status = 1
          while((self.optright < 0.20) or (self.optleft < 0.15)):
            twist.angular.z = 0.3
            self.twist_pub.publish(twist)
          self.status = self.status2
          #print("turnrightright")
      #lidarscanが壁を検知したとき
      elif(self.count > 3):
        if(self.countleft >= self.countright):
          twist.angular.z = -0.3
          self.twist_pub.publish(twist)
          #print("laser")
        else:
          twist.angular.z = 0.3
          self.twist_pub.publish(twist)
          #print("laser")
      elif(self.rulo_cable_distance <= 0.50):
        self.status = 1
        self.jtalk("ケーブルを回避します")
        time.sleep(3)
        sa_time = 0.0
        start = time.time()
        while(sa_time < 7.5):
          twist.angular.z = 0.3
          self.twist_pub.publish(twist)
          sa_time = time.time() - start
        while(self.rulo_cable_distance < 0.60):
          twist.angular.z = 0.0
          twist.linear.x = 0.10
          self.twist_pub.publish(twist)
          if((self.usleft < 0.15) or (self.usright < 0.15) or (self.count > 3)):
            self.status = self.status2
            return
      #要変更
      elif((self.map_result[0] >= self.map_area_threshold) and (self.status == 0)):
        self.jtalk("地図が完成したので、ゴールに向かいます")
        print("self.goal.x:", self.map_result[1])
        print("self.goal.y:", self.map_result[2])
        print("self.goal.z:", self.map_result[3])
        print("self.goal.w:", self.map_result[4])
        self.status = 1
        self.flag = 0
        self.status2 = 2
        #self.map_flag = 1
        twist.linear.x = 0.0
        self.twist_pub.publish(twist)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.map_result[1]
        goal.pose.position.y = self.map_result[2]
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = self.map_result[3]
        goal.pose.orientation.w = self.map_result[4]
        self.goal_pub.publish(goal)
        self.instant_goal_x = self.map_result[1]
        self.instant_goal_y = self.map_result[2]
        print("publish")
      #対象を認識したとき
      elif((self.status == 2) and (self.object_flag == 0) and (self.banana_flag == 1)):
        self.status = 1
        self.jtalk("バナナを発見しました。バナナに向かいます。")
        time.sleep(5)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.banana_point_x
        goal.pose.position.y = self.banana_point_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.instant_goal_x = self.banana_point_x
        self.instant_goal_y = self.banana_point_y
        self.object_goal_flag = 1
      elif((self.status == 2) and (self.object_flag == 1) and (self.apple_flag == 1)):
        self.status = 1
        self.jtalk("りんごを発見しました。りんごに向かいます。")
        time.sleep(5)
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.apple_point_x
        goal.pose.position.y = self.apple_point_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.instant_goal_x = self.apple_point_x
        self.instant_goal_y = self.apple_point_y
        self.object_goal_falg = 1
      else:      
        #print("前進")
        twist.linear.x = 0.10
        self.twist_pub.publish(twist)

if __name__ == '__main__':
 
    try:
        ts = turtleSim()
        rospy.spin()
    except rospy.ROSInterruptException: pass
