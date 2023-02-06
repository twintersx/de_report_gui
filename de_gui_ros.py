#!usr/bin/env

import rospy
from nrc_msgs.msg import GpsState

def gps_callback(msg):
    rospy.loginfo(str(msg.Latitude))

rospy.init_node('de_gui_node')
sub = rospy.Subscriber('/gps_state', GpsState, callback=gps_callback)

rospy.loginfo("Node has started!")
rospy.spin()