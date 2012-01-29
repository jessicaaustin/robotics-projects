#!/usr/bin/env python

"""

tracker_subscriber.py

This file subscribes to the '/TrackingCamera' topic.  This is a Point
message. Inside of the x and y coordinates of this message are the
current pan and tilt angles for the tracking rig.  We are going to
take those, and convert them to the proper tf messages, and then set
the tf tree to represent the current state of the pan-tilt rig.

"""

import roslib; roslib.load_manifest('nomad_camera_tracking')
import rospy
import tf

from geometry_msgs.msg import Point
from math import pi


## for now, let's define two global variables that represent the pan
## and tilt angles of the rig
pan = 0
tilt = 0


##################
# TIMER CALLBACK #
##################
def timer_callback():
    # from root to the camera_base frame this frame is located on the
    # table, directly below the camera lens the frame is in the same
    # orientation as root, so there is x,y offset but no rotation
    camera_base_br = tf.TransformBroadcaster()
    camera_base_br.sendTransform((0.78, 0.115, 0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 "camera_base",
                 "root")

    # from camera_base to camera_lens this frame has a rotation offset
    # from the base, plus a z offset, but no x,y offset
    camera_base_br = tf.TransformBroadcaster()
    camera_base_br.sendTransform((0, 0, 0.13),
                 tf.transformations.quaternion_from_euler(0.175, 0, 0.3),
                 rospy.Time.now(),
                 "camera_lens",
                 "camera_base")

    # from camera_lens to camera_lens_optical
    # rotate so that the z axis is coming out of the camera lens
    camera_base_br = tf.TransformBroadcaster()
    camera_base_br.sendTransform((0, 0, 0),
                 tf.transformations.quaternion_from_euler(-pi/2.0, 0, 0),
                 rospy.Time.now(),
                 "camera_lens_optical",
                 "camera_lens")
   

####################
# POINT SUBSCRIBER #
####################
def point_callback(data):
    """
    This function subscribes to the topic that is published by the
    tracking camera control node, and then it changes the values of
    the global variables for pan and tilt.
    """
    global pan, tilt


#########################
# SETUP ROS ENVIRONMENT #
#########################
def init_ros():
    rospy.init_node('tracker_subscriber')

    ## define the subscriber
    rospy.Subscriber("TrackingCamera", Point, point_callback)

    ## define a timer
    rospy.Timer(rospy.Rate(100), timer_callback)
    rospy.spin()



#################
# MAIN FUNCTION #
#################
if __name__ == '__main__':
    init_ros()
