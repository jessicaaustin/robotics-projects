#
# manage interaction with a LynxMotion SSC-32 via a serial port
#

import serial
import time

lineEnd = '\r\n'

class SSC32:
	def __init__(self, serialPort):
		self.port = serial.Serial(port = serialPort,
								  baudrate = 9600,
								  timeout = 1)
		self.port.setRTS(level = True)
		self.port.setDTR(level = True)

		self.firmwareVersion = self.requestResponse('VER')

		return

	def sendCommand(self, command):
		if (len(command) > 0):
			self.port.write(command)

		self.port.write(lineEnd)
		time.sleep(0.05)

		return

	def getResponse(self):
		response = ''
		while (self.port.inWaiting() == 0):
			continue

		while (self.port.inWaiting() > 0):
			response += self.port.read()

		return response

	def requestResponse(self, request):
		self.sendCommand(request)

		return self.getResponse()

	def commandNoResponse(self, command):
		self.sendCommand(command)

		return
