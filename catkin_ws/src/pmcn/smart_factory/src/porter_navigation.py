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
class porter_car_navigation_node(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pwm=PWM(address=0x40,debug=False)
        self.pwm.setPWMFreq(60)
	self.pwm.setPWM(15,0,600)
	self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        self.pub_porter_stop = rospy.Publisher("porter_car_servo/warehouse_following", String, queue_size=1)
	self.pub_porter_stop_msg = rospy.Publisher("porter_car_servo/car_warehouse_following", String, queue_size=1)
        self.sub_at_factory_stop_line=rospy.Subscriber("stop_line_filter_node/at_factory_stop_line",String, self.factory_action, queue_size=1)
        self.pub_car_msg=rospy.Publisher("~car_warehouse_following", String, queue_size=1)
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
	self.sub_navigation=rospy.Subscriber("~navigation_msg",String, self.navigation, queue_size=1)
	self.port_stop_line=""
    def navigation(self, navigation_msg):
	self.port_stop_line=navigation_msg.data
	msg = String()
        msg.data =navigation_msg.data+"_stop"
        self.pub_porter_stop.publish(msg)
	msg = String()
        msg.data ="porter_stop_line"
	self.pub_porter_stop_msg.publish(msg)
    def factory_action(self, msg):
	self.pwm.setPWM(15,0,350)
        time.sleep(2)
        self.pwm.setPWM(15,0,600)
	msg = String()
        msg.data = "red_stop"
        self.pub_porter_stop.publish(msg)
	strhttp='http://192.168.0.100/tsp/read_car_charging_time.php?car_id=1'
        req = urllib2.Request(strhttp)
        response = urllib2.urlopen(req)
        the_page = response.read()
	if(int(the_page)>10):
		print "go charging"
		strhttp='http://192.168.0.100/tsp/car_record_node.php?car_id=1&node_1=2'
        	req = urllib2.Request(strhttp)
        	response = urllib2.urlopen(req)
        	the_page = response.read()
	else:
		print "go shelf"
		strhttp='http://192.168.0.100/tsp/car_record_node.php?car_id=1&node_1=1'
                req = urllib2.Request(strhttp)
                response = urllib2.urlopen(req)
                the_page = response.read()
	time.sleep(1)
        lanefollowing_msg=BoolStamped()
        lanefollowing_msg.data=0
        self.pub_lanefollowing.publish(lanefollowing_msg)
if __name__=="__main__":
    rospy.init_node("porter_car_navigation",anonymous=False)
    porter_car_navigation = porter_car_navigation_node()
    rospy.spin()
