# constants for ROS project
#
# all parameters are in SI units (meters, m/s, etc)
#

#
# color tracker constants
#

# the index of your camera. if /dev/videoN is your camera device, then MY_CAMERA=N
MY_CAMERA = 0
# how many previous positions to interpolate to find our current position. higher smoothness => slower tracking, but less jerkiness 
SMOOTHNESS = 4
# minimum and maximum threshold value in HSV (hue,sat,val)
# red
#MIN_THRESH, MAX_THRESH = (163.0, 85.5, 72.5, 0.0), (189.0, 244.5, 247.5, 0.0)
# green
#MIN_THRESH, MAX_THRESH = ( 60.5, 74.5, 73.5, 0.0), (109.5, 215.5, 206.5, 0.0)
# blue
#MIN_THRESH, MAX_THRESH = ( 75.0, 80.0, 80.0, 0.0), (125.0, 230.0, 230.0, 0.0)
# yellow tennis ball
MIN_THRESH, MAX_THRESH = (37.5, 74.0, 80.0, 0.0), (50.5, 224.0, 230.0, 0.0)
# 6.92 pixels == 1 cm
PIXELS_TO_CM = 6.92
