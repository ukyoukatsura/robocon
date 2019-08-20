"""
tensorflow_in_ros_mnist.py
Copyright 2016 Shunya Seiya
This software is released under the Apache License, Version 2.0
https://opensource.org/licenses/Apache-2.0
"""
import os
import sys
sys.path.append('../')
import random
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16, Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tensorflow as tf
from nets import ssd_vgg_300, ssd_common, np_methods
from preprocessing import ssd_vgg_preprocessing
import pdb
import face_detect_eval
import darknet_test
from memory_profiler import profile
from subprocess import call, Popen, PIPE, STDOUT

import message_filters
import math
from statistics import median #pip install statistics
from geometry_msgs.msg import PointStamped
import tf as rostf
from visualization_msgs.msg import Marker, MarkerArray
import scipy.spatial as ss #pip install scipy

frame = 0
flag_find = 0

def bboxes_draw_on_img(img, classes, scores, bboxes, colors, thickness=2):

    shape = img.shape
    face_rslt = face_detection(img)
    '''
    for i in range(bboxes.shape[0]):
        bbox = bboxes[i]
        color = colors[classes[i]]
        # Draw bounding box...
        p1 = (int(bbox[0] * shape[0]), int(bbox[1] * shape[1]))
        p2 = (int(bbox[2] * shape[0]), int(bbox[3] * shape[1]))
        cv2.rectangle(img, p1[::-1], p2[::-1], color, thickness)
        # Draw t...
        s = '%s/%.3f' % (classes[i], scores[i])
        p1 = (p1[0]-5, p1[1])
        cv2.putText(img, s, p1[::-1], cv2.FONT_HERSHEY_DUPLEX, 0.4, color, 1)'''

    return(face_rslt)

def face_detection(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cascade = cv2.CascadeClassifier('/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml')
    faces = cascade.detectMultiScale(
        gray,     
        scaleFactor=1.1,
        minNeighbors=5
    )
    #print(faces)
    face_rslt = 2 #unkown
  
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]      
	face_rslt = trim(img, x, y, w, h)


    return(face_rslt)

    
def trim(img, x, y, w, h):
    label = {"0":'Kashiwagi', "1":'Shinohara', "2": 'unknown'}
    dst = img[y:y+h, x:x+w]
    new_img = np.array(dst)
    cv2.imwrite('/home/ubuntu/SSD-Tensorflow/trim/cut.jpeg',dst)
    result = face_detect_eval.evaluation(dst,'/home/ubuntu/SSD-Tensorflow/data_new/model.ckpt')
    print(result)
    cv2.putText(img, label[str(result)], (x,y), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255,0,0), 1)

    return(result)


def read_file():
    with open('/home/ubuntu/SSD-Tensorflow/result_yolo.txt', mode = 'rt') as f:
            txtdata = list(f)

    print txtdata[0].strip('\n')
    #print int(txtdata[1])
    name = txtdata[0].strip('\n')
    x = float(txtdata[1])
    y = float(txtdata[2])
    w = float(txtdata[3])
    h = float(txtdata[4])
    return name, x, y, w, h

class RosTensorFlow():
    def __init__(self, figsize=(10,10)):
        config = tf.ConfigProto(gpu_options=tf.GPUOptions(allow_growth=True))
        self._sess = tf.Session(config=config)
        self._cv_bridge = CvBridge()

        self.distance = []
        self.center = 0.0

        self.depth_horizon = 59

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.banana_data = []
        self.apple_data = []
        self.cable_data = []
        self.latest_cable_x = -100.0
        self.latest_cable_y = -100.0
        self.r = 0.3
        self.data_count = []


        self._tf_listener = rostf.TransformListener()
        self._latest_point = None

        net_shape = (300, 300)
        data_format = 'NHWC'

        self.img_input = tf.placeholder(tf.uint8, shape=(None, None, 3))
        # Evaluation pre-processing: resize to SSD net shape.
        self.image_pre, self.labels_pre, self.bboxes_pre, self.bbox_img = \
            ssd_vgg_preprocessing.preprocess_for_eval(
                self.img_input, None, None, net_shape, data_format,
                resize=ssd_vgg_preprocessing.Resize.WARP_RESIZE)
        self.image_4d = tf.expand_dims(self.image_pre, 0)

        self.ssd_net = ssd_vgg_300.SSDNet()
        with tf.contrib.slim.arg_scope(self.ssd_net.arg_scope(data_format=data_format)):
            self.predictions, self.localisations, _, _ = self.ssd_net.net(self.image_4d, is_training=False)

        # Restore SSD model.
        ckpt_filename = 'checkpoints/ssd_300_vgg.ckpt'
        # ckpt_filename = '../checkpoints/VGG_VOC0712_SSD_300x300_ft_iter_120000.ckpt'
        self._sess.run(tf.global_variables_initializer())

        saver = tf.train.Saver()
        saver.restore(self._sess, ckpt_filename)

        # SSD default anchor boxes.
        self._ssd_anchors = self.ssd_net.anchors(net_shape)

        #self._sub = rospy.Subscriber('camera/rgb/image_color/compressed', CompressedImage, self.callback, queue_size=1)
        self._pub_img = rospy.Publisher('ssd_image', Image, queue_size=1)
        self._pub_rslt = rospy.Publisher('ssd_result', Float32MultiArray, queue_size=1)
        self._pub_detect = rospy.Publisher('detect', Float32MultiArray, queue_size=1)
        self._pub_banana_point = rospy.Publisher('banana_point', Float32MultiArray, queue_size=1)
        self._pub_apple_point = rospy.Publisher('apple_point', Float32MultiArray, queue_size=1)
        self._pub_cable_point = rospy.Publisher('cable_point', Float32MultiArray, queue_size=1)
        sub_rgb = message_filters.Subscriber("camera/rgb/image_color/compressed",CompressedImage,queue_size = 1 , buff_size=2**24)
        sub_depth = message_filters.Subscriber("camera/depth/image",Image,queue_size = 1 , buff_size=2**24)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 1, 10.0) #2:queuesize 3:datanodoukinokyoyouhanni
        self.mf.registerCallback(self.callback)
        self.marker_pub = rospy.Publisher("test_text", Marker, queue_size = 10)
        self.marker_voting_cable_pub = rospy.Publisher("voting_cable_point", Marker, queue_size = 10)
        self.marker_voting_banana_pub = rospy.Publisher("voting_banana_point", Marker, queue_size = 10)
        self.marker_voting_apple_pub = rospy.Publisher("voting_apple_point", Marker, queue_size = 10)

        self._colors = []
        for i in xrange(21):
            _color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            self._colors.append(_color)


    # Main image processing routine.
    def _process_image(self, img, select_threshold=0.5, nms_threshold=.45, net_shape=(300, 300)):
        # Run SSD network.
        rpredictions, rlocalisations, rbbox_img = \
            self._sess.run(
                [self.predictions, self.localisations, self.bbox_img], feed_dict={self.img_input: img})

        # Get classes and bboxes from the net outputs.
        rclasses, rscores, rbboxes = np_methods.ssd_bboxes_select(
                rpredictions, rlocalisations, self._ssd_anchors,
                select_threshold=select_threshold, img_shape=net_shape, num_classes=21, decode=True)

        rbboxes = np_methods.bboxes_clip(rbbox_img, rbboxes)
        rclasses, rscores, rbboxes = np_methods.bboxes_sort(rclasses, rscores, rbboxes, top_k=400)
        rclasses, rscores, rbboxes = np_methods.bboxes_nms(rclasses, rscores, rbboxes, nms_threshold=nms_threshold)
        # Resize bboxes to original image shape. Note: useless for Resize.WARP!
        rbboxes = np_methods.bboxes_resize(rbbox_img, rbboxes)
        return rclasses, rscores, rbboxes

    def yolo_darknet(self,img,depth_image,imgstamp):
        cv2.imwrite('/home/ubuntu/SSD-Tensorflow/test.jpg', img)
        
        yolo_results = {}
        yolo_results[0] = "name"
        yolo_results[1] = ""     #"x"
        yolo_results[2] = ""     #"y"
        yolo_results[3] = ""     #"w"
        yolo_results[4] = ""     #"h"

        name = ""
            
        #global frame
        global flag_find
        #frame += 1
        #if frame%10 == 0:
        yolo_process = Popen(['python', 'darknet_test.py'], stdout=PIPE, stderr=STDOUT)
        results = ""
        for line in yolo_process.stdout:
            results += line.decode('utf-8', "replace")

        i = 0

        '''with open('/home/ubuntu/SSD-Tensorflow/result_yolo.txt', mode = 'rt') as f:
            txtdata = list(f)

        print txtdata[0].strip('\n')
        print int(txtdata[4])
        name = txtdata[0].strip('\n')
        x = int(txtdata[1])
        y = int(txtdata[2])
        w = int(txtdata[3])
        h = int(txtdata[4])'''
        
        name, x, y, w, h = read_file()

        if name != " ":
            flag_find = 1
        else:
            flag_find = 0

        rx = 0.0
        ry = 0.0
        rw = 0.0
        rh = 0.0


        if flag_find == 1:
            name, x, y, w, h = read_file()
            rx = (x-w/2)/320
            ry = (y-h/2)/240
            rw = (x+w/2)/320
            rh = (y+h/2)/240
            x = int(x)
            y = int(y)
            w = int(w)
            h = int(h)
            cv2.rectangle(img, (int(x-w/2),int(y-h/2)), (int(x+w/2), int(y+h/2)), (255,0,0),2)
            roi_color = img[y:y+h, x:x+w]
            cv2.putText(img, name, (int(x-w/2),int(y-h/2)), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255,0,0), 1)
            print (rx, ry, rw, rh)
            
            self.distance = []
            distance_count = []
            distance_r = 0.10
            self.center = 0.0
            x1 = x - w/2
            y1 = y - h/2
            x2 = x + w/2
            y2 = y + h/2

            if(x1 < 0):
              x1 = 0
            if(y1 < 0):
              y1 = 0
            if(x2 > 320):
              x2 = 319
            if(y2 > 240):
              y2 = 239

            img.flags.writeable = True
            for i in range(y1, y2+1):
                for j in range(x1, x2+1):
                    if depth_image.item(i,j) == depth_image.item(i,j):
                        self.distance.append((depth_image.item(i,j), 0))
                        img.itemset((i, j, 0), 0)
                        img.itemset((i, j, 1), 0)
                    
            print(len(self.distance))
            if(len(self.distance) == 0):
              return name, rx, ry, rw, rh
            if (len(self.distance) != 0):
                #self.center = median(self.distance)
              distance_count = [0] * len(self.distance)
              distance_kd_tree = ss.KDTree(self.distance, leafsize=10)
              distance_res = list(distance_kd_tree.query_pairs(distance_r))
              for j in range(len(distance_res)):
                for k in range(2):
                  distance_count[distance_res[j][k]] += 1
              max_index = distance_count.index(max(distance_count))
              self.center = self.distance[max_index][0] - 0.15
              print("%f [m]" %self.center)

            tmp = math.radians((((x2 + x1) / 2.0) - ( 320.0 / 2.0 )) * (self.depth_horizon / 320.0 ))
            self.robot_x = abs(self.center * math.cos(tmp))
            if(x2+x1 / 2.0) < 160:
                self.robot_y = abs(self.center * math.sin(tmp))
            else:
                self.robot_y = - abs(self.center * math.sin(-tmp))
            print(self.robot_x , self.robot_y)
            #TF transformation
            point_result = PointStamped()
            point_result.header.frame_id = "base_footprint"
            #point_result.header.stamp = rospy.Time(0)
            point_result.header.stamp = imgstamp
            self._latest_point = point_result
            point_result.point.x = self.robot_x
            point_result.point.y = self.robot_y
            point_result.point.z = 0.0
            print("###transformation")
            
            try:
                self._latest_point = self._tf_listener.transformPoint("/map",point_result)
                #self._latest_point = self._tf_listener.transformPoint("/map",rospy.Time.now(),imgstamp,point_result,"/map")
                #self._latest_point = self._tf_listener.transformPoint("/map",rospy.Time(0),imgstamp,point_result,"/map")
            except(rostf.LookupException, rostf.ConnectivityException, rostf.ExtrapolationException) as e:
                rospy.loginfo("Exception: {}".format(e))

            print(self._latest_point.point.x , self._latest_point.point.y)
           
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()
            marker_data.ns = "text"
            marker_data.id = 0
            marker_data.action = Marker.ADD
            marker_data.type = 9
            marker_data.text = "object"
            marker_data.pose.position.x = self._latest_point.point.x
            marker_data.pose.position.y = self._latest_point.point.y
            marker_data.pose.position.z = 0.0
            marker_data.pose.orientation.x = 0.0
            marker_data.pose.orientation.y = 0.0
            marker_data.pose.orientation.z = 0.0
            marker_data.pose.orientation.w = 0.0
            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1.0
            marker_data.scale.y = 0.1
            marker_data.scale.z = 0.1
            self.marker_pub.publish(marker_data)

            #print("#Delay:", (rospy.Time.now()-rgb_data.header.stamp).to_sec())
            if(name == "banana"):
              self.banana_data.append((self._latest_point.point.x , self._latest_point.point.y))
              self.data_count = [0] * len(self.banana_data)
              kd_tree = ss.KDTree(self.banana_data, leafsize=10)
              res = list(kd_tree.query_pairs(self.r))
              for j in range(len(res)):
                  for k in range(2):
                      self.data_count[res[j][k]] += 1
              print("------- self.data_coount -------")
              print(self.data_count)
              if(max(self.data_count) != 0):
                max_index = self.data_count.index(max(self.data_count))
                print(self.banana_data[max_index])
                print("max_data_count:", max(self.data_count))
              else:
                max_index = 0
              marker_voting = Marker()
              marker_voting.header.frame_id = "map"
              marker_voting.header.stamp = rospy.Time.now()
              marker_voting.ns = "voting_point"
              marker_voting.id = 0
              marker_voting.action = Marker.ADD
              marker_voting.type = 9
              marker_voting.text = "BANANA"
              marker_voting.pose.position.x = self.banana_data[max_index][0]
              marker_voting.pose.position.y = self.banana_data[max_index][1]
              marker_voting.pose.position.z = 0.0
              marker_voting.pose.orientation.x = 0.0
              marker_voting.pose.orientation.y = 0.0
              marker_voting.pose.orientation.z = 0.0
              marker_voting.pose.orientation.w = 0.0
              marker_voting.color.r = 0.0
              marker_voting.color.g = 0.0
              marker_voting.color.b = 1.0
              marker_voting.color.a = 1.0
              marker_voting.scale.x = 1.0
              marker_voting.scale.y = 0.1
              marker_voting.scale.z = 0.1
              self.marker_voting_banana_pub.publish(marker_voting)

              #cv2.namedWindow("color_image")
              #cv2.imshow("color_image", img)
              #cv2.waitKey(10)
              self._pub_banana_point.publish(Float32MultiArray(data=[self.banana_data[max_index][0], self.banana_data[max_index][1]]))
              #self._pub_banana_point.publish(Float32MultiArray(data=[self.banana_data[max_index][0]-0.2, self.banana_data[max_index][1]-0.2]))
            elif(name == "apple"):
              self.apple_data.append((self._latest_point.point.x , self._latest_point.point.y))
              self.data_count = [0] * len(self.apple_data)
              kd_tree = ss.KDTree(self.apple_data, leafsize=10)
              res = list(kd_tree.query_pairs(self.r))
              for j in range(len(res)):
                  for k in range(2):
                      self.data_count[res[j][k]] += 1
              print("------- self.data_coount -------")
              print(self.data_count)
              if(max(self.data_count) != 0):
                max_index = self.data_count.index(max(self.data_count))
                print(self.apple_data[max_index])
                print("max_data_count:", max(self.data_count))
              else:
                max_index = 0
              marker_voting = Marker()
              marker_voting.header.frame_id = "map"
              marker_voting.header.stamp = rospy.Time.now()
              marker_voting.ns = "voting_point"
              marker_voting.id = 0
              marker_voting.action = Marker.ADD
              marker_voting.type = 9
              marker_voting.text = "APPLE"
              marker_voting.pose.position.x = self.apple_data[max_index][0]
              marker_voting.pose.position.y = self.apple_data[max_index][1]
              marker_voting.pose.position.z = 0.0
              marker_voting.pose.orientation.x = 0.0
              marker_voting.pose.orientation.y = 0.0
              marker_voting.pose.orientation.z = 0.0
              marker_voting.pose.orientation.w = 0.0
              marker_voting.color.r = 0.0
              marker_voting.color.g = 0.0
              marker_voting.color.b = 1.0
              marker_voting.color.a = 1.0
              marker_voting.scale.x = 1.0
              marker_voting.scale.y = 0.1
              marker_voting.scale.z = 0.1
              self.marker_voting_apple_pub.publish(marker_voting)

              #cv2.namedWindow("color_image")
              #cv2.imshow("color_image", img)
              #cv2.waitKey(10)
              self._pub_apple_point.publish(Float32MultiArray(data=[self.apple_data[max_index][0], self.apple_data[max_index][1]]))
              #self._pub_apple_point.publish(Float32MultiArray(data=[self.apple_data[max_index][0]-0.2, self.apple_data[max_index][1]-0.2]))
            elif(name == "cable"):
              self.cable_data.append((self._latest_point.point.x , self._latest_point.point.y))
              self.data_count = [0] * len(self.cable_data)
              kd_tree = ss.KDTree(self.cable_data, leafsize=10)
              res = list(kd_tree.query_pairs(self.r))
              for j in range(len(res)):
                  for k in range(2):
                      self.data_count[res[j][k]] += 1
              print("------- self.data_coount -------")
              print(self.data_count)
              if(max(self.data_count) != 0):
                max_index = self.data_count.index(max(self.data_count))
                print(self.cable_data[max_index])
                print("max_data_count:", max(self.data_count))
              else:
                max_index = 0
              marker_voting = Marker()
              marker_voting.header.frame_id = "map"
              marker_voting.header.stamp = rospy.Time.now()
              marker_voting.ns = "voting_point"
              marker_voting.id = 0
              marker_voting.action = Marker.ADD
              marker_voting.type = 9
              marker_voting.text = "CABLE"
              marker_voting.pose.position.x = self.cable_data[max_index][0]
              marker_voting.pose.position.y = self.cable_data[max_index][1]
              marker_voting.pose.position.z = 0.0
              marker_voting.pose.orientation.x = 0.0
              marker_voting.pose.orientation.y = 0.0
              marker_voting.pose.orientation.z = 0.0
              marker_voting.pose.orientation.w = 0.0
              marker_voting.color.r = 0.0
              marker_voting.color.g = 0.0
              marker_voting.color.b = 1.0
              marker_voting.color.a = 1.0
              marker_voting.scale.x = 1.0
              marker_voting.scale.y = 0.1
              marker_voting.scale.z = 0.1
              self.marker_voting_cable_pub.publish(marker_voting)

              #cv2.namedWindow("color_image")
              #cv2.imshow("color_image", img)
              #cv2.waitKey(10)
              #self._pub_cable_point.publish(Float32MultiArray(data=[self.cable_data[max_index][0]-0.2, self.cable_data[max_index][1]-0.2]))
              #self._pub_cable_point.publish(Float32MultiArray(data=[self.cable_data[max_index][0], self.cable_data[max_index][1]]))
              self.latest_cable_x = self.cable_data[max_index][0]
              self.latest_cable_y = self.cable_data[max_index][1]
        return name, rx, ry, rw, rh
    
    def callback(self, image_msg,depth_data):
        print("img_get")
        self._pub_cable_point.publish(Float32MultiArray(data=[self.latest_cable_x, self.latest_cable_y]))
        # self._pub_apple_point.publish(Float32MultiArray(data=[self.apple_data[max_index][0], self.apple_data[max_index][1]]))
        # self._pub_banana_point.publish(Float32MultiArray(data=[self.banana_data[max_index][0], self.banana_data[max_index][1]]))
        global frame
        frame += 1
        if frame%1 == 0:
            try:
                depth_image = self._cv_bridge.imgmsg_to_cv2(depth_data, 'passthrough')
            except CvBridgeError, e:
                rospy.logerr(e)

            cv_image = self._cv_bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
            print("10img_get")
            #rclasses, rscores, rbboxes = self._process_image(cv_image)
            rscores, rbboxes = "0", "0"
            rclasses = { "apple":918.0, "banana":1112.0, "cable":102.0}

            face_rslt = bboxes_draw_on_img(cv_image, rclasses, rscores, rbboxes, self._colors)
            class_name, x, y, w, h = self.yolo_darknet(cv_image,depth_image,image_msg.header.stamp)
            print("#Delay:", (rospy.Time.now()-image_msg.header.stamp).to_sec())
            self._pub_img.publish(self._cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))

            if len(class_name) > 2:
                #bbox = rbboxes[0]
                self._pub_rslt.publish(
                    Float32MultiArray(data=[float(rclasses[class_name]), y, x, h, w]))
                
            else:
                self._pub_rslt.publish(
                    Float32MultiArray(data=[0., 0., 0., 0., 0.]))

            kashiwagi = 100
            shinohara = 101	
        
            if face_rslt == 0:
                #kashiwagi
                print("kashiwagi")

		with open('/home/ubuntu/SSD-Tensorflow/face_results.txt', mode='w') as f:
               	    f.write(str(face_rslt))

                self._pub_detect.publish(
                    Float32MultiArray(data=[kashiwagi]))
            elif face_rslt == 2:
                #shinohara
                print("shinohara")
		with open('/home/ubuntu/SSD-Tensorflow/face_results.txt', mode='w') as f:
               	    f.write(str(face_rslt))

                self._pub_detect.publish(
                    Float32MultiArray(data=[shinohara]))

	    elif face_rslt == 3:
                face_count += 1
           	if face_count % 7 == 0:
                    with open('/home/ubuntu/SSD-Tensorflow/face_results.txt', mode='w') as f:
                        f.write(str(face_rslt))


            # print rclasses
            # self._plt_bboxes(cv_image, rclasses, rscores, rbboxes)
            # rospy.loginfo('%d' % answer)

            # self._pub.publish(answer)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rostensorflow')
    tensor = RosTensorFlow()
    tensor.main()
