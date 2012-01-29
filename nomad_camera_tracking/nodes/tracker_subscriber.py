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
def timer_callback(event):
    global pan, tilt

    pan_offset = 0
    tilt_offset = 0

    ## define transform from the root of the tree to the base of the
    ## tracker rig
    tracker_base_br = tf.TransformBroadcaster()
    tracker_base_br.sendTransform((0.05, 0.04, 0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 "tracker_base",
                 "root")

    ## now set the transform from the base, up to the first servo:
    tracker_base_br.sendTransform((0.15, 0, 0.12),
                 tf.transformations.quaternion_from_euler(
                                     tilt+tilt_offset, 0, 0),
                 rospy.Time.now(),
                 "tracker_tilt_servo",
                 "tracker_base")

    ## now set the transform from the base, up to the first servo:
    tracker_base_br.sendTransform((0.02, 0.02, 0.03),
                 tf.transformations.quaternion_from_euler(
                                     0, 0, pan+pan_offset),
                 rospy.Time.now(),
                 "tracker_pan_servo",
                 "tracker_tilt_servo")

    ## now set the transform from the pan servo to the tracker lens
    tracker_base_br.sendTransform((0, 0.02, 0.02),
                 tf.transformations.quaternion_from_euler(
                                     0, 0, 0),
                 rospy.Time.now(),
                 "tracker_lens",
                 "tracker_pan_servo")
 

    # from camera_lens to camera_lens_optical
    # rotate so that the z axis is coming out of the camera lens
    tracker_base_br.sendTransform((0, 0, 0),
                 tf.transformations.quaternion_from_euler(-pi/2.0, 0, 0),
                 rospy.Time.now(),
                 "tracker_lens_optical",
                 "tracker_lens")
   

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
    pan = data.x
    tilt = data.y


#########################
# SETUP ROS ENVIRONMENT #
#########################
def init_ros():
    rospy.init_node('tracker_subscriber')

    ## define the subscriber
    rospy.Subscriber("TrackingCamera", Point, point_callback)

    ## define a timer
    rospy.Timer(rospy.Duration(0.01), timer_callback)

    rospy.loginfo("Starting the tracker listener...")
    rospy.spin()



#################
# MAIN FUNCTION #
#################
if __name__ == '__main__':
    init_ros()
