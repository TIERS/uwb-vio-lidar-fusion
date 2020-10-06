#!/usr/bin/env python
""" For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
"""

__author__	 = "Carmen Martinez Almansa"
__version__	= "0.1"
__maintainer__ = "Carmen Martinez Almansa"
__email__	  = "camart@utu.fi"
__status__	 = "Development"


import rospy, time, serial, os
import math
import numpy as np
import random

from geometry_msgs.msg import Pose
from std_msgs.msg       import Float64

from dwm1001_apiCommands import DWM1001_API_COMMANDS
# initialize ros rate 10hz


# initialize serial port connections



class dwm1001_localizer:

	def __init__(self) :
		"""
		Initialize the node, open serial port
		"""
		
		# Init node
		rospy.init_node('DWM1001_Distance_Publisher', anonymous=False)
		
		# Set a ROS rate
		self.rate = rospy.Rate(1)
		
		# Empty dictionary to store topics being published
		self.topics = {}
		
		# Serial port settings
		self.dwm_port = rospy.get_param('~port')
		self.serialPortDWM1001 = serial.Serial(
			port = self.dwm_port,
			baudrate = 115200,
			parity = serial.PARITY_ODD,
			stopbits = serial.STOPBITS_TWO,
			bytesize = serial.SEVENBITS
		)

		#Number of anchors in the network
		self.num_anchors = 0

		#TAG POSITION mean and LSE
		self.mean_pos = [0,0]
		self.LSE_pos = [0,0]

	def main(self) :
		"""
		Initialize port and dwm1001 api
		:param:
		:returns: none
		"""

		# close the serial port in case the previous run didn't closed it properly
		self.serialPortDWM1001.close()
		# sleep for one sec
		time.sleep(1)
		# open serial port
		self.serialPortDWM1001.open()

		# check if the serial port is opened
		if(self.serialPortDWM1001.isOpen()):
			rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
		# start sending commands to the board so we can initialize the board
			self.initializeDWM1001API()
			# give some time to DWM1001 to wake up
			time.sleep(2)
			# send command lec, so we can get positions is CSV format
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
			self.serialPortDWM1001.write('S'.encode())
			rospy.loginfo("Reading DWM1001 coordinates")
		else:
			rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

		try:

			while not rospy.is_shutdown():
				# just read everything from serial port
				serialReadLine = self.serialPortDWM1001.read_until('\r\n')

				try:
					self.publishTagPositions(serialReadLine.decode())

				except IndexError:
					rospy.loginfo("Found index error in the network array!DO SOMETHING!")



		except KeyboardInterrupt:
			rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
			self.serialPortDWM1001.write('F'.encode())

		finally:
			rospy.loginfo("Quitting, and sending reset command to dev board")
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
			self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
			self.rate.sleep()
			serialReadLine = self.serialPortDWM1001.read_until()
			if "END" in serialReadLine:
				rospy.loginfo("succesfully closed ")
				self.serialPortDWM1001.close()
			

	def publishTagPositions(self, serialData):
		"""
		Publish anchors and tag in topics using Tag and Anchor Object
		:param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
		:returns: none
		"""
		
		try:
			arrayData = [x.strip() for x in serialData.strip().split(',')]
		except:
			rospy.loginfo("Something went wrong when splitting serial data ")
		
		self.num_anchors  = len(arrayData)/10 #10 bits per anchor (tag and anchor id, distance, variance and position+error)
		anchor2tag_dist = np.zeros(self.num_anchors)
		variances = np.zeros(self.num_anchors)
		interanchor_dists = np.zeros(self.num_anchors)
		
		
		for i in range(self.num_anchors):
			try:
				tag_id = arrayData[i*10]
				anchor_id = arrayData[i*10+1]
				anchor2tag_dist[i] = (float(arrayData[i*10 + 2]) * 256.0 + float(arrayData[i*10 + 3])-35.74) #substract offset
				variances[i] = float(arrayData[10*i+4]) * 256.0 + float(arrayData[10*i+5])
				#rospy.loginfo("anchor to tag "+str(anchor2tag_dist[i]))
				if(tag_id == str(0)):
					if anchor_id not in self.topics :	
						self.topics[anchor_id +"_dist"] = rospy.Publisher('/utuTIERS/tag/'+tag_id+'/to/anchor/'+anchor_id+"/distance", Float64, queue_size=100)
						self.topics[anchor_id +"_variance"] = rospy.Publisher('/utuTIERS/tag/'+tag_id+'/to/anchor/'+anchor_id+"/variance", Float64, queue_size=100)
					
					self.topics[anchor_id+"_dist"].publish(float(anchor2tag_dist[i]))
					self.topics[anchor_id+"_variance"].publish(float(variances[i]))
			except:
				rospy.loginfo("Error while creating topics ")
	

	def initializeDWM1001API(self):
		"""
		Initialize dwm1001 api, by sending sending bytes
		:param:
		:returns: none
		"""
		# reset incase previuos run didn't close properly
		self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
		# send ENTER two times in order to access api
		self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
		# sleep for half a second
		time.sleep(0.5)
		self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
		# sleep for half second
		time.sleep(0.5)
		# send a third one - just in case
		self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)


if __name__ == '__main__':
	try:
		dwm1001 = dwm1001_localizer()
		dwm1001.main()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
