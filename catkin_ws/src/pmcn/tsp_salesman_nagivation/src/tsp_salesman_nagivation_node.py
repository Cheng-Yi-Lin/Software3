#!/usr/bin/env python
import rospy
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading , AprilTagsWithInfos
from std_msgs.msg import String, Int32, Int16
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import urllib2
from __builtin__ import True

class tsp_salesman_nagivation_node(object):

    def __init__(self):
        
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        self.sub_tag_id = rospy.Subscriber("apriltags_postprocessing_node/apriltags_out", AprilTagsWithInfos, self.get_Apriltag, queue_size=1)
        self.sub_commodity = rospy.Subscriber("~commodity_info", String, self.salesman, queue_size=1)
        self.pub_at_stop_line = rospy.Publisher("stop_line_filter_node/at_stop_line",BoolStamped,queue_size=1)
        self.type_back=rospy.Publisher("open_loop_intersection_control_node/turn_type", Int16, queue_size=1)
        self.pub_stop_line_reading = rospy.Publisher("stop_line_filter_node/stop_line_reading", StopLineReading, queue_size=1)
        self.pub_at_stop_back = rospy.Publisher("~at_stop_back",BoolStamped,queue_size=1)
        
        self.flag = 0
        self.sound = ''
        self.n_stop = False
        self.path_commodity=list()
        self.path_commodity.append(0)
        self.car_action_check=1
        self.back_info=0
        self.last_commodity_tag=0
        self.car_read_action()
        self.last_action="" 
        self.car_read_action()

    def car_read_action(self):
        while True:
            strhttp='http://192.168.0.100/tsp/read_car_action.php?car_id=1'
            req = urllib2.Request(strhttp)
            response = urllib2.urlopen(req)
            the_page = response.read()
            
            if(self.car_action_check==1 and int(the_page)==0 and self.last_commodity_tag!=0):
                self.car_action_check=int(the_page)
                e_stop_msg=BoolStamped()
                e_stop_msg.data=int(the_page)
                self.pub_lanefollowing.publish(e_stop_msg)
                if self.last_action=="B":
                    stop_line_reading_msg = StopLineReading()
                    stop_line_reading_msg.stop_line_detected = True
                    stop_line_reading_msg.at_stop_line = 1
                    self.pub_stop_line_reading.publish(stop_line_reading_msg)
                    stop_msg=BoolStamped()
                    stop_msg.data=True
                    self.pub_at_stop_back.publish(stop_msg)
                    time.sleep(1)
                    self.turn=3
                    self.type_back.publish(self.turn)
            
            self.last_commodity_tag=0
            self.car_action_check=int(the_page)
            e_stop_msg=BoolStamped()
            e_stop_msg.data=int(the_page)
            self.pub_lanefollowing.publish(e_stop_msg)
            time.sleep(1)
            

    def salesman(self,Commodity):
        if(len(Commodity.data)):
            self.path_commodity=Commodity.data.split(" ")

    def get_Apriltag(self,Tag):
        try:
            tag_id = Tag.infos[0].id
            commodity_exist=0
            rospy.loginfo("----- TAG = %d-----" %(tag_id))
            
            i=0
            tag_node=int(tag_id/4)
            while(len(self.path_commodity)>i+1):
                sss=self.path_commodity[i]
                if(int(sss)==tag_node):
                    commodity_exist=1
                    self.path_commodity[i]=-1
                    break
                i=i+1
            print(commodity_exist)
            print(self.path_commodity)
            if  commodity_exist==1:
                strhttp='http://192.168.0.100/tsp/car_record_action.php?car_id=1&car_action=1'
                req = urllib2.Request(strhttp)
                response = urllib2.urlopen(req)
                the_page = response.read()
                
                strhttp='http://192.168.0.100/tsp/car_recode_navigation.php?car_id=1&tag_id='+str(tag_id)
                req = urllib2.Request(strhttp)
                response = urllib2.urlopen(req)
                the_page = response.read()
                self.last_action=the_page[0]
                self.last_commodity_tag=tag_id
            
        except:
            print("NO TAG")


if __name__  == "__main__":
    rospy.init_node("tsp_salesman_nagivation_node",anonymous=False)
    tsp_salesman= tsp_salesman_nagivation_node()
    rospy.spin()
