#!/usr/bin/env python

from __future__ import print_function

import sys
import random
import rospy
from std_msgs.msg import String
from limo_status_translator.srv import GetLimoStatus, GetLimoStatusRequest

topic = {1 : '/limo_status/vehicle_state', 2 : '/limo_status/control_mode', 
3 : '/limo_status/battery_voltage', 4 : '/limo_status/error_code', 5 : '/limo_status/motion_mode'}

def publish(msg):
    num = 1
    while num <= 5:
        pub = rospy.Publisher(topic[num], String, queue_size=50)
        num += 1
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            str = msg
            rospy.loginfo(str)
            pub.publish(str)
            rate.sleep()
            return
        

if __name__ == "__main__":

    rospy.init_node('client')
    service = rospy.ServiceProxy('service', GetLimoStatus)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        a = random.randint(0,4)
        rospy.loginfo("sending [%d] " % a)
        req = GetLimoStatusRequest()
        req.get_status = a
        rospy.wait_for_service('service')
        resp = service(req)
        rospy.loginfo("Recieved: " + resp.status_string)
        r.sleep()
        publish(resp.status_string)
    
    
