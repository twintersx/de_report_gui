#!usr/bin/env/python

import rospy, rosgraph, os
from nrc_msgs.msg import GpsState

def check_master():
    try:
        rosgraph.Master('/rostopic').getPid()
        return True
    except:
        return False
    
def gps_callback(msg):
    print(str(msg.Latitude) + 'abc123xyz' + str(msg.Longitude)) # abc123xyz is unique delimiter
    rospy.sleep(0.1)
    os.system("rosnode kill " + 'de_gui_node')
    
if check_master():
    rospy.init_node('de_gui_node')
    sub = rospy.Subscriber('/gps_state', GpsState, callback=gps_callback)
    rospy.spin()