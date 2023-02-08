#!usr/bin/env/python

import rospy, os
from nrc_msgs.msg import GpsState

def gps_callback(msg):
    print(str(msg.Latitude) + ', ' + str(msg.Longitude))
    rospy.sleep(0.1)
    os.system("rosnode kill " + 'de_gui_node')
    
rospy.init_node('de_gui_node')
sub = rospy.Subscriber('/gps_state', GpsState, callback=gps_callback)

rospy.spin()