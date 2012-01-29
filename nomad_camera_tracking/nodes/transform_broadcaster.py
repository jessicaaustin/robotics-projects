#!/usr/bin/env python

"""

transform_broadcaster.py

broadcasts both static and dynamic transforms for the project

"""

import roslib; roslib.load_manifest('nomad_camera_tracking')
import rospy

import tf

from math import pi

def broadcast_camera_static_frames():
    # from root to the camera_base frame
    # this frame is located on the table, directly below the camera lens
    # the frame is in the same orientation as root, so there is x,y offset but no rotation
    camera_base_br = tf.TransformBroadcaster()
    camera_base_br.sendTransform((0.78, 0.115, 0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 "camera_base",
                 "root")

    # from camera_base to camera_lens
    # this frame has a rotation offset from the base, plus a z offset, but no x,y offset
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
   

# TODO fill this in
def broadcast_pan_tilt_camera_frames():
    return


if __name__ == '__main__':
    rospy.init_node('transforms')

    while not rospy.is_shutdown():
        broadcast_camera_static_frames()
        rospy.sleep(1)

