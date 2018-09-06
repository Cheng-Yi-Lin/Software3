#!/usr/bin/env python
import rospy
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, StopLineReading , AprilTagsWithInfos
from std_msgs.msg import String, Int32, Int16
import time
import thread
import urllib2
from __builtin__ import True

class server_node(object):
    def __init__(self):
        
        self.pub_lanefollowing=rospy.Publisher("joy_mapper_node/joystick_override",BoolStamped,queue_size=1)
        self.car_action=-1
        try:
            thread.start_new_thread( self.car_read,())
        except:
            print "Error: can't read server"
    def car_read(self):
        while True:
            strhttp='http://192.168.0.100/tsp/read_car_action.php?car_id=1'
            req = urllib2.Request(strhttp)
            response = urllib2.urlopen(req)
            the_page = response.read()
            if self.car_action != the_page:
                print "server car action change"
                lanefollowing_msg=BoolStamped()
                lanefollowing_msg.data=int(the_page)
                self.pub_lanefollowing.publish(lanefollowing_msg)
                self.car_action = the_page
    def onShutdown(self):
        self.loginfo("Shutdown.")
if __name__  == "__main__":
    rospy.init_node("server_node",anonymous=False)
    server= server_node()
    rospy.on_shutdown(server.onShutdown)
    rospy.spin()
