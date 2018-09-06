#!/usr/bin/env python
import rospy
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading, FSMState, AprilTagsWithInfos
from std_msgs.msg import String, Int32, Int16
from sensor_msgs.msg import Joy,CompressedImage
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import urllib2
import numpy as np
import math
import cv2
from duckietown_utils.jpg import image_cv_from_jpg
from __builtin__ import True
class porter_car_servo_node(object):
    print "porter_car_servo_node start"
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pub_warehouse_following = rospy.Publisher("~warehouse_following", String, queue_size=1)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_mode = rospy.Subscriber("fsm_node/mode",FSMState, self.mode, queue_size=1)
        self.sub_shelf_tag = rospy.Subscriber("apriltags_postprocessing_node/apriltags_out", AprilTagsWithInfos, self.shelf, queue_size=1)
	self.sub_warehouse_msg=rospy.Subscriber("random_april_tag_turns_node/warehouse_msg",String, self.warehouse, queue_size=1)
        self.sub_at_shelf_stop_line=rospy.Subscriber("stop_line_filter_node/at_shelf_stop_line",String, self.shelf_action, queue_size=1)
        self.pub_car_msg=rospy.Publisher("~car_warehouse_following", String, queue_size=1)
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
	self.sub_image=rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.ArmImage, queue_size=1)
        self.pub_joy_cmd = rospy.Publisher("joy_mapper_node/car_cmd",Twist2DStamped, queue_size=1)
        self.warehouse_data=list()
        self.stop_shelf_tag=""
        self.stop_shelf_color=""
        self.porter_info=""
        self.mode=""
	self.ArmImage_check=0
        self.camera_time=rospy.Time.now() #time.time()
        self.goal_machine_name=""
	self.goal_machine_tag=""
    def ArmImage(self,image_msg):
        if self.ArmImage_check==0:
            return #print "00000"
	if rospy.Time.now()-self.camera_time<rospy.Duration.from_sec(2.5):
            return
        if self.ArmImage_check==1:
                self.camera_time=rospy.Time.now() #time.time()
                image_cv=image_cv_from_jpg(image_msg.data)
                img_data=np.copy(image_cv)
                hsv=np.empty(0)
                hsv=cv2.cvtColor(img_data,cv2.COLOR_BGR2HSV)
		if self.stop_shelf_color=="blue":
                	low_blue=np.array([85,150,150])
                	high_blue=np.array([130,255,255])
		else :
			low_blue=np.array([50,100,100])
                        high_blue=np.array([77,255,255])

                mask=cv2.inRange(hsv,low_blue,high_blue)
                img=cv2.findContours(mask,1,2)
                large_blue_area=0
                for i in range(0,len(img[1])):
                        if cv2.contourArea(img[1][i])>large_blue_area:
                                large_blue_area=cv2.contourArea(img[1][i])
                                large_i=i
                #print large_blue_area
                if large_blue_area>300:
                        moments=cv2.moments(img[1][large_i])
                        if moments['m00']==0:
                                return
                        cx=moments['m10']/moments['m00']
                        cy=moments['m01']/moments['m00']
                        cy=450-cy
                        print cy
			if cy<20:
				self.ArmImage_check=0
				self.shelf_action2()
				return
                        cmd_msg=Twist2DStamped()
                        cmd_msg.v=0.5
                        cmd_msg.omega=0
                        wheel_time=rospy.Time.now() #time.time()
                        while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(0.0015*cy)):
                                self.pub_joy_cmd.publish(cmd_msg)
                        cmd_msg.v=0
                        cmd_msg.omega=0
                        self.pub_joy_cmd.publish(cmd_msg)
    def warehouse(self,warehouse_info):
        print warehouse_info.data
        print "get warehouse info"
        self.warehouse_data=warehouse_info.data.split(",")
        print(self.warehouse_data)
        self.stop_shelf_tag=self.warehouse_data[0]
        self.stop_shelf_color=self.warehouse_data[1]
	self.goal_machine_name=self.warehouse_data[2]
	self.goal_machine_tag=self.warehouse_data[3]
        self.porter_info="in_warehouse_lane_following"
        car_msg = String()
        car_msg.data = self.porter_info
        self.pub_car_msg.publish(car_msg)

        msg = String()
        msg.data =self.warehouse_data[1]+"_stop"
        self.pub_warehouse_following.publish(msg)
        
        lanefollowing_msg=BoolStamped()
        lanefollowing_msg.data=0
        self.pub_lanefollowing.publish(lanefollowing_msg)
    
    def mode(self,mode_msg):
        self.mode=mode_msg.state
    def cbJoy(self, joy_msg):
        
	if(joy_msg.buttons[0] == 1):
            msg = String()
            msg.data = "green_stop"
            self.pub_warehouse_following.publish(msg)
        elif (joy_msg.buttons[1] == 1):
            msg = String()
            msg.data ="red_stop"
            self.pub_warehouse_following.publish(msg)
    def shelf_action(self,shelf_msg):
	        e_stop_msg=BoolStamped()
                e_stop_msg.data=1
                self.pub_lanefollowing.publish(e_stop_msg)
	        self.ArmImage_check=1
    def shelf_action2(self):
                strhttp='http://192.168.0.100/tsp/read_shelf_action.php?shelf_tag_id='+str(self.stop_shelf_tag)
                req = urllib2.Request(strhttp)
                response = urllib2.urlopen(req)
                the_page = response.read()
                if the_page[0]=="0":
                    print "do something"
                elif the_page[0]=="1":
                    strhttp='http://192.168.0.100/tsp/shelf_record_action.php?shelf_action=2&shelf_tag_id='+str(self.stop_shelf_tag)
		    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
	            strhttp='http://192.168.0.100/tsp/read_shelf_action.php?shelf_tag_id='+str(self.stop_shelf_tag)
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
                    while(the_page[0]!="1"):
                        print "shelf " + str(self.stop_shelf_tag) + " not action" 
                   	strhttp='http://192.168.0.100/tsp/read_shelf_action.php?shelf_tag_id='+str(self.stop_shelf_tag)
               		req = urllib2.Request(strhttp)
                	response = urllib2.urlopen(req)
                	the_page = response.read()
                        time.sleep(0.5)
		    strhttp='http://192.168.0.100/tsp/machine_material_add.php?value=10&machine_name='+str(self.goal_machine_name)
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
		    strhttp='http://192.168.0.100/tsp/car_record_node.php?car_id=1&node_1='+str(self.goal_machine_tag)+'&node_2=2'
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
                    #servo is 450~650
                self.porter_info="in_warehouse_go_out"
        	car_msg = String()
        	car_msg.data ="in_warehouse_go_out"
        	self.pub_car_msg.publish(car_msg)
                #time.sleep(1)
                msg = String()
                msg.data ="red_stop"
                self.pub_warehouse_following.publish(msg)
                time.sleep(3)
	        lanefollowing_msg=BoolStamped()
        	lanefollowing_msg.data=0
        	self.pub_lanefollowing.publish(lanefollowing_msg)
    def shelf(self,shelf_tag):
        try:
            shelf_id=shelf_tag.infos[0].id
            print("------------------------------------------------------")
            print(shelf_id)
            print(self.stop_shelf_tag)
            if self.porter_info=="in_warehouse_go_out":
                print"in_warehouse_go_out"
            elif int(shelf_id)==int(self.stop_shelf_tag) or  int(shelf_id)==int(self.stop_shelf_tag)-1:
                print("in_warehouse_stop"+self.stop_shelf_color+"_line")
                self.porter_info=("in_warehouse_stop_line")
	        car_msg = String()
     	        car_msg.data = self.porter_info
                self.pub_car_msg.publish(car_msg)
        except:
            print " NO TAG "
if __name__=="__main__":
    rospy.init_node("porter_car",anonymous=False)
    porter_car_servo = porter_car_servo_node()
    rospy.spin()
