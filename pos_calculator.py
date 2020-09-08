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

import message_filters
from message_filters import TimeSynchronizer, Subscriber
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
		rospy.init_node("DWM1001_POS_Calculator_{}".format(random.randint(0,100000)), anonymous=False)
		#Number of anchors in the network
		self.num_anchors = 4
		#Anchors' positions
		self.anchor_pos = [[47.0,-257.0],[47.0,195.0],[-598.0,197.0],[-598.0,-234.0]]
		self.anchor_pos = np.array(self.anchor_pos)
		
		#Vectors to store tag position mean and distance to each anchor
		self.mean_pos = [0,0]
		self.anchor2tag_dist = [0,0,0,0]

		# Empty dictionary to store topics being published
		self.topics = {}

		## Get raw distances to anchors
		self.d1_sub = message_filters.Subscriber("/utuTIERS/tag/0/to/anchor/1/distance",Float64)
		self.d2_sub = message_filters.Subscriber("/utuTIERS/tag/0/to/anchor/2/distance",Float64)
		self.d3_sub = message_filters.Subscriber("/utuTIERS/tag/0/to/anchor/3/distance",Float64)
		self.d4_sub = message_filters.Subscriber("/utuTIERS/tag/0/to/anchor/4/distance",Float64)
		ats = message_filters.ApproximateTimeSynchronizer([self.d1_sub,self.d2_sub,self.d3_sub,self.d4_sub], queue_size=10, slop=0.1,allow_headerless=True)
		ats.registerCallback(self.dist_cb)


	def run(self) :
		'''
			Infinite loop with 20 Hz sleep
		''' 
		# Set Rospy rate
		rate = rospy.Rate(20.0)

		# Infinite loop
		try:
			while not rospy.is_shutdown() :
				rate.sleep()

		except KeyboardInterrupt :
			 rospy.logerr('Keyboard Interrupt detected!')

		
	def dist_cb(self,d1,d2,d3,d4):
		self.anchor2tag_dist[0] = d1.data
		self.anchor2tag_dist[1] = d2.data
		self.anchor2tag_dist[2] = d3.data
		self.anchor2tag_dist[3] = d4.data
		self.calculateTagPosition()

	def calculateTagPosition(self):
		#Empty vector to store the distance between each pair of anchors
		interanchor_dists = [0,0,0,0]
	
		error_cont = 0
		self.topics["pos_mean"] = rospy.Publisher('/utuTIERS/tag/position/mean', Pose, queue_size=100)	
		
		tag_x_total = 0
		tag_y_total = 0
		#counter to check how many anchors were actually used to calculate the mean and flag to signal that
		anchors_used = 0
		pub = False
		
		for i in range(self.num_anchors):
			if(i != self.num_anchors - 1):
				interanchor_dists[i] = math.sqrt((self.anchor_pos[i+1][0] - self.anchor_pos[i][0])**2 + (self.anchor_pos[i+1][1] - self.anchor_pos[i][1])**2)
				if(self.anchor2tag_dist[i]**2+interanchor_dists[i]**2) > self.anchor2tag_dist[i+1]**2 :
						theta = math.acos((self.anchor2tag_dist[i]**2+interanchor_dists[i]**2-self.anchor2tag_dist[i+1]**2)/(2*self.anchor2tag_dist[i]*interanchor_dists[i]))
						#angle between the current coord system and the actual coord system considering the first anchor in the pair as the origin
						alpha = math.atan2(self.anchor_pos[i+1][1]-self.anchor_pos[i][1], self.anchor_pos[i+1][0]-self.anchor_pos[i][0]) 
						anchors_used += 1
						pub = True
			#same formulas but for the last anchor with respect to anchor0
			else:
				interanchor_dists[i] = math.sqrt((self.anchor_pos[i][0] - self.anchor_pos[0][0])**2 + (self.anchor_pos[i][1] - self.anchor_pos[0][1])**2) 
				if(self.anchor2tag_dist[i]**2+interanchor_dists[i]**2)>self.anchor2tag_dist[0]**2 :
					theta = math.acos((self.anchor2tag_dist[i]**2 + interanchor_dists[i]**2 - self.anchor2tag_dist[0]**2)/(2*self.anchor2tag_dist[i]*interanchor_dists[i]))
					alpha = math.atan2(self.anchor_pos[0][1] - self.anchor_pos[i][1], self.anchor_pos[0][0]-self.anchor_pos[i][0]) #angle between the current coord system and the actual coord system
					anchors_used += 1
					pub = True 
				
			#calculate tag position for every anchor
			if(pub == True) :	
				tag_x = (self.anchor2tag_dist[i]*math.cos(theta + alpha) + self.anchor_pos[i][0])/100
				tag_y = (self.anchor2tag_dist[i]*math.sin(theta + alpha) + self.anchor_pos[i][1])/100
				tag_x_total += tag_x
				tag_y_total += tag_y
				pub = False
				
		#calculate and publish the mean
		tag_x = tag_x_total/anchors_used 
		tag_y = tag_y_total/anchors_used  
		p = Pose()
		p.position.x = float(tag_x)
		p.position.y = float(tag_y)
		p.position.z = 0.0
		p.orientation.x = 0.0
		p.orientation.y = 0.0
		p.orientation.z = 0.0
		p.orientation.w = 1.0
		self.topics["pos_mean"].publish(p)
		self.mean_pos[0] = tag_x
		self.mean_pos[1] = tag_y
		rospy.loginfo("TAG POS : "+"("+str(tag_x) + "," + str(tag_y)+")")
					

		# except:
		# 	rospy.loginfo("ERROR CALCULATING ")
		


if __name__ == '__main__':
	try:
		dwm1001 = dwm1001_localizer()
		dwm1001.run()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
