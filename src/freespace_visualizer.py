#!/usr/bin/env python

import numpy as np 
from numpy import pi 
from math import sin, cos 
import rospy 

from sensor_msgs.msg import LaserScan 


from itertools import *                # groupby
from operator import itemgetter


import matplotlib.pyplot as plt 

# parameters
SCAN_TOPIC = rospy.get_param("/freespace_visualizer/scan_topic")
NUM_MEASUREMENTS = rospy.get_param("/freespace_visualizer/num_measurements")
ANGLE_MIN = rospy.get_param("/freespace_visualizer/angle_min")                # rad
ANGLE_MAX = rospy.get_param("/freespace_visualizer/angle_max")                # rad
ANGLE_INCREMENT = rospy.get_param("/freespace_visualizer/angle_increment")    #rad
RANGE_MIN = rospy.get_param("/freespace_visualizer/range_min")                # meter, m
RANGE_MAX = rospy.get_param("/freespace_visualizer/range_max")               # meter, m
DISTANCE_THRESHOLD = rospy.get_param("/freespace_visualizer/distance_threshold") # meter, m


# Global Variables
global cosVal, sinVal 
global printCount 

printCount = 0


# pre-compute sin,cos values as they will be the same for each scan
cosVal = np.cos(np.arange(ANGLE_MIN, ANGLE_MAX+ANGLE_INCREMENT, ANGLE_INCREMENT))
sinVal = np.sin(np.arange(ANGLE_MIN, ANGLE_MAX+ANGLE_INCREMENT, ANGLE_INCREMENT))

# create array that contains index value corresponding to each angular measurement
angleIndices = np.arange(NUM_MEASUREMENTS)



# laser data callback 
def free_space_scan_cb(msg):

   global printCount
   
   gapList = []
   xmin = []
   ymin = []
   xmax = []
   ymax = []
   minAngle = []
   maxAngle = []

   # initializing so that data is defined if plotting is called before laser callback
   # populates these values. Otherwise, will get an undefined error
   xdata = np.zeros(NUM_MEASUREMENTS)
   ydata = np.zeros(NUM_MEASUREMENTS)


   # convert polar coordinates: radius, angle to x,y
   # msg.ranges is a tuple
   xdata = (np.asarray(msg.ranges).T * cosVal).T
   ydata = (np.asarray(msg.ranges).T * sinVal).T


   # filter out values that are less than or equal to the threshold
   # assumption is that we will not run into anything by always
   # turning towards the largest opening
   distance_mask = ( (np.array(msg.ranges)) > DISTANCE_THRESHOLD)

   # thresholded indices contains index location of every distance
   # measurement that met threshold criteria
   thresholded_indices = list(angleIndices[distance_mask])

  
   # The following is python I would not be able to write myself. Thank goodness
   # for the above referenced github source code.

   # iterate through the thresholded distances to form groups by finding
   # a break in consecutive indices

   # enumerate(threshold_indices) creates a tuple, the loop index and the object
   # Example: loops through the theshold_indices list. Creates tuple (0,0)
   # loop index is 0, first object in threshold_indices happens to be 0.

   # lambda creates an anonymous function with two parameters: i,x
   # returns the value i - x 
   # for the tuple (0,0) returns 0-0

   # groupby will place all object values with the same i-x value in the same group

   # When the enumerate index and threshold_indices values are different, then i-x
   # will start a new group. All consecutive values will have the same i-x value 
   # until another gap in the indices is found.

   # Example: assume threshold_indices contains the following: 0,1,2,5,6,9,10,11
   # The first group contains [0,1,2], second group contains [5,6], and third group [9,10,11]
   # i-x for first group is 0, i-x for second group is -2 (3-5, 4-6). 
   # i-x for third group is -4 (5-9, 6-10, 7-11)
   for k, g in groupby(enumerate(thresholded_indices), lambda (i,x):i-x):
      gapList.append(map(itemgetter(1),g))


   # storee free space endpoints
   for g in gapList:
      minAngle.append(g[0]*((ANGLE_INCREMENT)*180/np.pi)) 
      maxAngle.append(g[-1]*((ANGLE_INCREMENT)*180/np.pi))

      # calculate corresponding xy values
      xmin.append(cosVal[g[0]] * msg.ranges[g[0]])
      ymin.append(sinVal[g[0]] * msg.ranges[g[0]])
      xmax.append(cosVal[g[-1]] * msg.ranges[g[-1]])
      ymax.append(sinVal[g[-1]] * msg.ranges[g[-1]])

   if( (printCount % 25) == 0):
      plt.clf()         # clear all figures
      plt.figure(1)

      # plot x y data points
      plt.plot(xdata, ydata, 'b.', label="scan data")

      # plot line for each free space region from min to max angle in region
      for i in range(len(xmin)):
         plt.plot([xmin[i], xmax[i]],[ymin[i], ymax[i]], linestyle="dashed", linewidth = 3, label=('freespace '+str(i)))
         
      plt.grid(True)
      plt.legend()

      plt.xlim([ -RANGE_MAX-1, RANGE_MAX+1])
      plt.ylim([ -RANGE_MAX-1, RANGE_MAX+1])
      plt.ylabel("y [m]")
      plt.xlabel("x [m]")
      plt.title("free space, distance threshold: " + str(DISTANCE_THRESHOLD))
      plt.pause(0.001)
      print(printCount)
      #plt.show(False) 
   printCount += 1
   


# initialize ros node and subsribe to topic
rospy.init_node('free_space_visualizer', anonymous = True)
rospy.Subscriber(SCAN_TOPIC, LaserScan, free_space_scan_cb)


try:
   while(not rospy.is_shutdown()):
      pass
   
   # calling show will keep the plot open after the keyboard interrupt
   # ctrl+c shuts ros down
   plt.show()

except KeyboardInterrupt:
   pass 

