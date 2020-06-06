#!/usr/bin/env python

import numpy as np 
from numpy import pi 
from math import sin, cos 
import rospy 

from sensor_msgs.msg import LaserScan 


import matplotlib.pyplot as plt 

# parameters
SCAN_TOPIC = rospy.get_param("/freespace_visualizer/scan_topic")
NUM_MEASUREMENTS = rospy.get_param("/freespace_visualizer/num_measurements")
ANGLE_MIN = rospy.get_param("/freespace_visualizer/angle_min")                # rad
ANGLE_MAX = rospy.get_param("/freespace_visualizer/angle_max")                # rad
ANGLE_INCREMENT = rospy.get_param("/freespace_visualizer/angle_increment")    #rad
RANGE_MIN = rospy.get_param("/freespace_visualizer/range_min")                # meter, m
RANGE_MAX = rospy.get_param("/freespace_visualizer/range_max")               # meter, m
#DISTANCE_THRESHOLD = rospy.get_param("/freespace_visualizer/distance_threshold")


# Global Variables
global xdata 
global ydata 
global cosVal
global sinVal 

# initializing so that data is defined if plotting is called before laser callback
# populates these values. Otherwise, will get an undefined error
xdata = np.zeros(NUM_MEASUREMENTS)
ydata = np.zeros(NUM_MEASUREMENTS)

# pre-compute sin,cos values as they will be the same for each scan
cosVal = np.cos(np.arange(ANGLE_MIN, ANGLE_MAX+ANGLE_INCREMENT, ANGLE_INCREMENT))
sinVal = np.sin(np.arange(ANGLE_MIN, ANGLE_MAX+ANGLE_INCREMENT, ANGLE_INCREMENT))

# laser data callback 
def free_space_scan_cb(msg):
   global xdata
   global ydata 
  

   # convert polar coordinates: radius, angle to x,y
   # msg.ranges is a tuple
   xdata = (np.asarray(msg.ranges).T * cosVal).T
   ydata = (np.asarray(msg.ranges).T * sinVal).T



# initialize ros node and subsribe to topic
rospy.init_node('free_space_visualizer', anonymous = True)
rospy.Subscriber(SCAN_TOPIC, LaserScan, free_space_scan_cb);


try:
   while(not rospy.is_shutdown()):
     
      plt.clf()         # clear all figures
      plt.figure(1)

      # plot x y data points
      plt.plot(xdata, ydata, 'b.')
      plt.grid(True)

      plt.xlim([ -RANGE_MAX-1, RANGE_MAX+1])
      plt.ylim([ -RANGE_MAX-1, RANGE_MAX+1])
      plt.ylabel("y [m]")
      plt.xlabel("x [m]")
      plt.title("Laser Scan, laser reference frame")
      plt.pause(0.01) 
   
   # calling show will keep the plot open after the keyboard interrupt
   # ctrl+c shuts ros down
   plt.show()

except KeyboardInterrupt:
   pass 

