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
class porter_car_info_node(object):
    def __init__(self):
        self.node_name = rospy.get_name()


	self.sub_stop_line_filter_color = rospy.Subscriber("porter_car_servo/warehouse_following", String, self.warehouse_following_mode, queue_size=1)
	self.stop_line_filter_color="red_stop" #red_stop blue_stop green_stop

	self.sub_porter_lane_following_info=rospy.Subscriber("porter_car_servo/car_warehouse_following",String, self.warehouse_info, queue_size=1)
	self.porter_lane_following_info=""  #in_warehouse_lane_following  in_charger_station_lane_following porter_stop_line in_warehouse_stop_line in_charger_stop_line


	self.sub_at_shelf_stop_line = rospy.Subscriber("stop_line_filter_node/at_shelf_stop_line"  ,String, self.shelf_action, queue_size=1)
	self.at_shelf_stop_line=""

	self.sub_at_factory_stop_line=rospy.Subscriber("stop_line_filter_node/at_factory_stop_line",String, self.factory_action, queue_size=1)
	self.at_factory_stop_line=""

	self.sub_at_charger_stop_line=rospy.Subscriber("stop_line_filter_node/at_charger_stop_line",String, self.charger_action, queue_size=1)
	self.at_charger_stop_line=""


	self.sub_navigation_msg=rospy.Subscriber("~navigation_msg",String, self.navigation, queue_size=1)
	self.navigation_msg=""
	self.sub_charging_msg=rospy.Subscriber("random_april_tag_turns_node/charging_msg",String, self.charging, queue_size=1)
	self.charging_msg=""
	self.sub_warehouse_msg=rospy.Subscriber("random_april_tag_turns_node/warehouse_msg",String, self.warehouse, queue_size=1)
	self.warehouse_msg=""

	self.info()
    def info(self):
	while True:
                time.sleep(1)
                print "===================================================================================="
                print "stop_line_filter_color  :  " + self.stop_line_filter_color
                print "porter_lane_following_info  :  " + self.porter_lane_following_info
                print "at_shelf_stop_line  :  " +   self.at_shelf_stop_line
                print "at_factory_stop_line  :  " + self.at_factory_stop_line
                print "at_charger_stop_line  :  " + self.at_charger_stop_line
                print "navigation_msg  :  " + self.navigation_msg
                print "charging_msg  :  " + self.charging_msg
                print "warehouse_msg  :  " +  self.warehouse_msg
    def warehouse_following_mode(self, msg):
        self.stop_line_filter_color=msg.data
    def warehouse_info(self, msg):
        self.porter_lane_following_info=msg.data
    def shelf_action(self, msg):
        self.at_shelf_stop_line=msg.data
    def factory_action(self, msg):
        self.at_factory_stop_line=msg.data
    def charger_action(self, msg):
        self.at_charger_stop_line=msg.data
    def navigation(self, msg):
        self.navigation_msg=msg.data
    def charging(self, msg):
	self.charging_msg=msg.data
    def warehouse(self, msg):
	self.warehouse_msg=msg.data
    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))
if __name__=="__main__":
    rospy.init_node("porter_car_info",anonymous=False)
    porter_car_info = porter_car_info_node()
    rospy.spin()
