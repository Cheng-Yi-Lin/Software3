#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped
from std_msgs.msg import String, Int16 #Imports msg
import urllib2
class RandomAprilTagTurnsNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        self.pub_CommodityInfo = rospy.Publisher("tsp_salesman_nagivation/commodity_info",String, queue_size=1)
 	self.pub_porter_navigation = rospy.Publisher("porter_navigation/navigation_msg",String, queue_size=1)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)
        self.pub_charging_msg= rospy.Publisher("~charging_msg", String, queue_size=1)
        self.pub_warehouse_msg= rospy.Publisher("~warehouse_msg", String, queue_size=1)
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            rospy.loginfo("Turn type now: %i" %(self.turn_type))
            
    def cbTag(self, tag_msgs):
        if(self.fsm_mode == "INTERSECTION_CONTROL"):
            #loop through list of april tags
            for taginfo in tag_msgs.infos:
                print taginfo.id
                #rospy.loginfo("[%s] taginfo." %(taginfo))
                if int(taginfo.id)==201 or int(taginfo.id)==202 or int(taginfo.id)==203 or int(taginfo.id)==204:
                    print " shelf shelf shelf !!!!! "
                elif int(taginfo.id)==200:
		    print "stop in front of warehouse"
                    strhttp='http://192.168.0.100/tsp/warehouse_in.php'
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
                    print the_page
                    msg = String()
		    msg.data = the_page
		    self.pub_warehouse_msg.publish(msg)
                    print"wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww"
                elif int(taginfo.id)==301 or int(taginfo.id)==302 or int(taginfo.id)==303 or int(taginfo.id)==304:
                    print"charger charger charger"
                    #self.turn_type = 4
                    #self.pub_turn_type.publish(self.turn_type)
                elif int(taginfo.id)==300:
                    print "stop in front of charging"
                    strhttp='http://192.168.0.100/tsp/charging_in.php'
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
                    print the_page
                    msg = String()
                    msg.data = the_page
                    self.pub_charging_msg.publish(msg)
                    print"wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww"

		elif(taginfo.tag_type == taginfo.SIGN):
                    availableTurns = []
                    #go through possible intersection types
                    signType = taginfo.traffic_sign_type
                    strhttp='http://192.168.0.100/tsp/car_recode_navigation.php?car_id=1&tag_id='+str(taginfo.id)
                    req = urllib2.Request(strhttp)
                    response = urllib2.urlopen(req)
                    the_page = response.read()
                    print the_page
                    the_car_action=the_page.split(",")
                    print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    print the_car_action[0]
                    commodity_message=""
                    if(the_car_action[1]):
                        commodity_message=the_car_action[1]
		        self.pub_porter_navigation.publish(commodity_message)
                    #self.pub_CommodityInfo.publish(commodity_message)
                    if(the_car_action[0]!="!"):
                        if the_car_action[0]=="S":
                            chosenTurn=1
                        elif the_car_action[0]=="L":
                            chosenTurn=0
                        elif the_car_action[0]=="R":
                            chosenTurn=2
                        elif the_car_action[0]=="B":
                            chosenTurn=3
                        else:
                            return
                        self.turn_type = chosenTurn
                        self.pub_turn_type.publish(self.turn_type)
                        rospy.loginfo("possible turns %s." %(availableTurns))
                        rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('random_april_tag_turns_node', anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
