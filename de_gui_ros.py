#!usr/bin/env

import rospy, csv
from nrc_msgs.msg import GpsState

def gps_callback(msg):
    rospy.loginfo(str(msg.Latitude))
    rospy.loginfo(str(msg.Longitude))

    with open('/home/leaf/Desktop/de_report_gui/reports.csv', 'a') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([str(msg.Latitude), str(msg.Longitude)])

    sub.unregister()

rospy.init_node('de_gui_node')
sub = rospy.Subscriber('/gps_state', GpsState, callback=gps_callback)

rospy.loginfo("Node has started!")
rospy.spin()