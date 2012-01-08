#!/usr/bin/env python

"""

color-tracker.py

tracks a blob of a specified color
publishes to a topic (blob_coord) the x,y coords of that blob

"""

import roslib; roslib.load_manifest('nomad_camera_tracking')
import rospy

from geometry_msgs.msg import Point

import settings

class ColorTracker():
    def __init__(self):
        return
    
    def find_blob(self):
        return Point(0, 0, 0)

if __name__ == '__main__':
    rospy.init_node('color_tracker')
    color_tracker = ColorTracker()
    pub_blob_coord = rospy.Publisher('blob_coord', Point)

    while not rospy.is_shutdown():
        blob_coord = color_tracker.find_blob()
        pub_blob_coord.publish(blob_coord)
