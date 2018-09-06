#!/usr/bin/env python
import rospy
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading, FSMState, AprilTagsWithInfos
from std_msgs.msg import String, Int32, Int16
from sensor_msgs.msg import Joy,CompressedImage
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
from duckietown_utils.jpg import image_cv_from_jpg
import time
import cv2
import urllib2
import numpy as np
import math
from __builtin__ import True
class car_charging_node(object):
    print "porter_car_servo_node start"
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pub_charger_following = rospy.Publisher("porter_car_servo/warehouse_following", String, queue_size=1)
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_mode = rospy.Subscriber("fsm_node/mode",FSMState, self.mode, queue_size=1)
        self.sub_shelf_tag = rospy.Subscriber("apriltags_postprocessing_node/apriltags_out", AprilTagsWithInfos, self.charger, queue_size=1)
	self.sub_charging_msg=rospy.Subscriber("random_april_tag_turns_node/charging_msg",String, self.charging, queue_size=1) #1-1
        self.sub_at_shelf_stop_line=rospy.Subscriber("stop_line_filter_node/at_charger_stop_line",String, self.charger_action, queue_size=1)
        self.pub_car_msg=rospy.Publisher("porter_car_servo/car_warehouse_following", String, queue_size=1)
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        self.pub_turn_type = rospy.Publisher("open_loop_intersection_control_node/turn_type",Int16, queue_size=1, latch=True)
        self.sub_image=rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.ArmImage, queue_size=1)
        self.pub_joy_cmd = rospy.Publisher("joy_mapper_node/car_cmd",Twist2DStamped, queue_size=1)
	self.charging_data=list()#1-3
        self.stop_charger_tag=""#1-4
        self.stop_charger_color=""#1-4
        self.porter_info=""
        self.mode=""
	self.ArmImage_check=0
	self.camera_time=rospy.Time.now() #time.time()
	self.check_turn_right=0
    def ArmImage(self,image_msg):
        if self.ArmImage_check==0:
            return#print "00000"
        if rospy.Time.now()-self.camera_time<rospy.Duration.from_sec(2.5):
            return
	if self.ArmImage_check==1:
     	  	self.camera_time=rospy.Time.now() #time.time()
        	image_cv=image_cv_from_jpg(image_msg.data)
        	img_data=np.copy(image_cv)
	        hsv=np.empty(0)
        	hsv=cv2.cvtColor(img_data,cv2.COLOR_BGR2HSV)
                if self.stop_charger_color=="blue":
       	 		low_blue=np.array([85,100,100])
        		high_blue=np.array([130,255,255])
		elif self.stop_charger_color=="green":
			low_blue=np.array([50,100,100])
                        high_blue=np.array([77,255,255])
        	mask=cv2.inRange(hsv,low_blue,high_blue)
        	img=cv2.findContours(mask,1,2)
        	large_blue_area=0
        	for i in range(0,len(img[1])):
	    		if cv2.contourArea(img[1][i])>large_blue_area:
                		large_blue_area=cv2.contourArea(img[1][i])
           	     		large_i=i
		if large_blue_area>50:
            		moments=cv2.moments(img[1][large_i])
            		if moments['m00']==0:
                		return
            		cx=moments['m10']/moments['m00']
            		cy=moments['m01']/moments['m00']
	    		cy=460-cy
			if self.check_turn_right==1:
				return
                        elif cy<30 :
                                cmd_msg=Twist2DStamped()
                                cmd_msg.v=0.15
                                cmd_msg.omega=-15
				wheel_time=rospy.Time.now()
                                while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(0.4)):
                                        self.pub_joy_cmd.publish(cmd_msg)
                                cmd_msg.v=0
                                cmd_msg.omega=0
                                self.pub_joy_cmd.publish(cmd_msg)
				self.check_turn_right=1
				self.ArmImage_check=2
				return
                        else:
	    			cmd_msg=Twist2DStamped()
            			cmd_msg.v=0.5
            			cmd_msg.omega=0
            			wheel_time=rospy.Time.now()
				cy1 = cy**0.5
            			while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(0.025*cy1)):
                			self.pub_joy_cmd.publish(cmd_msg)
            			cmd_msg.v=0
            			cmd_msg.omega=0
            			self.pub_joy_cmd.publish(cmd_msg)
	if self.ArmImage_check==2:
 		self.camera_time=rospy.Time.now()
                image_cv=image_cv_from_jpg(image_msg.data)
                img_data=np.copy(image_cv)
                hsv=np.empty(0)
                hsv=cv2.cvtColor(img_data,cv2.COLOR_BGR2HSV)
		if self.stop_charger_color=="blue":
                	low_blue=np.array([50,100,100])
                	high_blue=np.array([77,255,255])
		elif self.stop_charger_color=="green" :
			low_blue=np.array([85,100,100])
                        high_blue=np.array([130,255,255])
                mask=cv2.inRange(hsv,low_blue,high_blue)
                img=cv2.findContours(mask,1,2)
                large_blue_area=0
                for i in range(0,len(img[1])):
                        if cv2.contourArea(img[1][i])>large_blue_area:
                                large_blue_area=cv2.contourArea(img[1][i])
                                large_i=i
                if large_blue_area>50:
                        moments=cv2.moments(img[1][large_i])
                        if moments['m00']==0:
                                return
                        cx=moments['m10']/moments['m00']
                        cy=moments['m01']/moments['m00']
                        cx-=340
			cy=700-cy
			
                        cmd_msg=Twist2DStamped()
			if cx>15:
                        	cmd_msg.v=0.2
                        	cmd_msg.omega=-3.5
                        	wheel_time=rospy.Time.now()
                        	while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(cx/500)):
                                	self.pub_joy_cmd.publish(cmd_msg)
			elif cx<-15:	
                                cmd_msg.v=0.2
                                cmd_msg.omega=3.5
                                wheel_time=rospy.Time.now() 
                                while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(cx/-500)):
                                        self.pub_joy_cmd.publish(cmd_msg)
			else:
				cmd_msg.v=0.4
                        	cmd_msg.omega=0
                     	   	wheel_time=rospy.Time.now() 
                        	while((rospy.Time.now()-wheel_time)<rospy.Duration.from_sec(0.001*cy)):
                                	self.pub_joy_cmd.publish(cmd_msg)
                        cmd_msg.v=0
                        cmd_msg.omega=0
                        self.pub_joy_cmd.publish(cmd_msg)
           
    def charging(self,charging_msg):#1-2
        self.charging_data=charging_msg.data.split(",")
        self.stop_charger_tag=self.charging_data[0]
        self.stop_charger_color=self.charging_data[1]
        self.porter_info="in_charger_station_lane_following"
        car_msg = String()
        car_msg.data = self.porter_info
        self.pub_car_msg.publish(car_msg)#2-1

        msg = String()
        msg.data =self.charging_data[1]+"_stop"
        self.pub_charger_following.publish(msg)
        
        lanefollowing_msg=BoolStamped()
        lanefollowing_msg.data=0
        self.pub_lanefollowing.publish(lanefollowing_msg)
    
    def mode(self,mode_msg):
        self.mode=mode_msg.state
    def cbJoy(self, joy_msg):
        
	if(joy_msg.buttons[0] == 1):
            msg = String()
            msg.data = "blue_stop"
            self.pub_charger_following.publish(msg)
	    self.ArmImage_check=1
	    self.check_turn_right=0
        elif (joy_msg.buttons[1] == 1):
            self.ArmImage_check=0
            msg = String()
            msg.data ="red_stop"
            self.pub_charger_following.publish(msg)
    def charger_action(self,shelf_msg):
	        lanefollowing_msg=BoolStamped()
        	lanefollowing_msg.data=1
        	self.pub_lanefollowing.publish(lanefollowing_msg)

                self.ArmImage_check=1
		self.check_turn_right=0
		
                self.porter_info=("in_charger_charging")
                car_msg = String()
                car_msg.data = self.porter_info
                self.pub_car_msg.publish(car_msg)
		msg = String()
                msg.data ="red_stop"
                self.pub_charger_following.publish(msg)
		
    def charger(self,charger_tag):
        try: 
            if int(charger_tag.infos[0].id)==int(self.stop_charger_tag) or int(charger_tag.infos[0].id)==int(self.stop_charger_tag)-1:
                self.porter_info=("in_charger_stop_line")
	        car_msg = String()
     	        car_msg.data = self.porter_info
                self.pub_car_msg.publish(car_msg)
        except:
            print "NO TAG "
if __name__=="__main__":
    rospy.init_node("car_charging",anonymous=False)
    car_charging = car_charging_node()
    rospy.spin()
