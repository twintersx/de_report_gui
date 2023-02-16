#!usr/bin/env/python

from rospy import get_published_topics, init_node, Subscriber, spin, sleep
from rosgraph import is_master_online
from nrc_msgs.msg import GpsState
from os import system
       
def gps_callback(msg):
    print(str(msg.Latitude) + 'abc123xyz' + str(msg.Longitude)) # abc123xyz is unique delimiter
    sleep(0.5)  #helps shutdown node
    system('rosnode kill ' + 'de_gui_node')

if is_master_online():
    topics = get_published_topics()
    if any('/gps_state' in sub for sub in topics):
        init_node('de_gui_node')
        sub = Subscriber('/gps_state', GpsState, callback=gps_callback)
        spin()