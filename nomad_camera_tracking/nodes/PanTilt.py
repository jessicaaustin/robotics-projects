#
# Pan and tilt a camera
#

"""
PanTilt module by Bill Mania <bill@manialabs.us>

Works with a LynxMotion SSC-32 and two servoes to aim
a pan/tilt rig
"""

import time
from SSC32 import *

class PanTilt:
	def __init__(self, serialPort):
		self.cameraPanTilt = SSC32(serialPort)
		self.panServo = 0
		self.tiltServo = 4
		self.defaultSpeed = 300
		self.panCenter = "#%d P1500 S%d" % (self.panServo, self.defaultSpeed)
		self.tiltCenter = "#%d P1500 S%d" % (self.tiltServo, self.defaultSpeed)
		self.panValue = 0
		self.tiltValue = 0
		self.panScale = 15
		self.tiltScale = 15

	def getCameraAim(self):
		return self.panValue, self.tiltValue

	def centerCamera(self):
		self.cameraPanTilt.commandNoResponse(self.panCenter)
		self.cameraPanTilt.commandNoResponse(self.tiltCenter)

		self.panValue = 0
		self.tiltValue = 0

		return

	def setSpeed(self, newSpeed):
		self.defaultSpeed = newSpeed

		return

	def exerciseCamera(self):
		self.centerCamera()
		time.sleep(1.0)
		self.tiltCamera(20)
		time.sleep(1.0)
		self.panCamera(20)
		time.sleep(1.0)
		self.tiltCamera(-40)
		time.sleep(1.0)
		self.panCamera(-40)
		time.sleep(1.0)
		self.tiltCamera(-40)
		time.sleep(1.0)
		self.centerCamera()
		
		return

	def tiltCamera(self, tiltFactor):
		if (tiltFactor < -50):
			tiltFactor = -50
		elif (tiltFactor > 50):
			tiltFactor = 50

		tilt = tiltFactor * self.tiltScale + 1500
		self.cameraPanTilt.commandNoResponse("#%d P%d S%d" % (self.tiltServo, tilt, self.defaultSpeed))
		self.tiltValue = tiltFactor

		return

	def panCamera(self, panFactor):
		if (panFactor < -50):
			panFactor = -50
		elif (panFactor > 50):
			panFactor = 50

		pan = panFactor * self.panScale - 1500
		self.cameraPanTilt.commandNoResponse("#%d P%d S%d" % (self.panServo, pan, self.defaultSpeed))
		self.panValue = panFactor

		return

	def aimCamera(self, pan, tilt):
		self.panCamera(pan)
		self.tiltCamera(tilt)

		return
