#!/usr/bin/env python2

import numpy as np
import ros_numpy
import matplotlib.pyplot as plt

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan

# import custom message: pattern is package_name.msg import message name
from dwsubt.msg import Turn

from itertools import *
from operator import itemgetter


class LaserObstacleAvoid:
   # Import ROS parameters from the "params.yaml" file.
   # Access these variables in class functions with self:
   # i.e. self.CONSTANT
   SCAN_TOPIC = rospy.get_param("/laser_obstacle_avoid/scan_topic")
   ANGLE_MIN = rospy.get_param("/laser_obstacle_avoid/angle_min")                # rad
   ANGLE_MAX = rospy.get_param("/laser_obstacle_avoid/angle_max")                # rad
   ANGLE_INCREMENT = rospy.get_param("/laser_obstacle_avoid/angle_increment")    #rad
   RANGE_MIN = rospy.get_param("/laser_obstacle_avoid/range_min")                # meter, m
   RANGE_MAX = rospy.get_param("/laser_obstacle_avoid/range_max")               # meter, m
   DISTANCE_THRESHOLD = rospy.get_param("/laser_obstacle_avoid/distance_threshold")
   MIN_GAP = rospy.get_param("/laser_obstacle_avoid/min_gap")
   #NUM_MEASUREMENTS = rospy.get_param("/dwsubt/num_measurements")
  

   def __init__(self):
      
      # Initialize your publishers and
      # subscribers here
      self.laser_subscriber = rospy.Subscriber(self.SCAN_TOPIC, LaserScan,self.laser_callback, queue_size=1)

      self.turn_publisher = rospy.Publisher("/avoid_obstacle_turn", Turn, queue_size=1)
     
      # pre-compute sin,cos values as they will be the same for each scan
      self.cosVal = np.cos(np.arange(self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_INCREMENT))
      self.sinVal = np.sin(np.arange(self.ANGLE_MIN, self.ANGLE_MAX, self.ANGLE_INCREMENT))


      self.Kp = 0.05             # proportional control of turn angle

      self.printCount = 1

   def printstuff(self):
      print(self.SCAN_TOPIC)


   def laser_callback(self, data):
      # reference: https://github.com/vibhuthasak/Obstacle_Avoidance_ROS/blob/master/scripts/sensor_data_listener.py
      
      angleIndices = np.arange(len(data.ranges))
      distances = np.array(data.ranges)

      # filter out values that are less than or equal to the threshold
      # assumption is that we will not run into anything by always
      # turning towards the largest opening
      distance_mask = (distances > self.DISTANCE_THRESHOLD)

      # thresholded indices contains index location of every distance
      # measurement that met threshold criteria
      thresholded_indices = list(angleIndices[distance_mask])
      
      max_gap = 20  # degrees
      gap_list = []

      

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
         gap_list.append(map(itemgetter(1),g))


      # sort the list of lists. The longest list is the widest angular gap.
      gap_list.sort(key=len)
      largest_gap = gap_list[-1]

      # calculate the angle. This is in the laser's reference frame
      min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/np.pi), largest_gap[-1]*((data.angle_increment)*180/np.pi)
      
      average_gap = (max_angle - min_angle)/2

      # aim for the center of the gap
      turn_angle = min_angle + average_gap


      if average_gap < max_gap:
         angz = -0.5 
      else:
         angz = self.Kp * (-1)*(90-turn_angle)

      # publish turn angle
      turn_msg = Turn()
      turn_msg.header.stamp = rospy.Time.now()
      turn_msg.anglez = angz

      self.turn_publisher.publish(turn_msg)

      if self.printCount == 1:
         self.printCount += 1

         print("min angle, max_angle")
         print(min_angle, max_angle)
         
         print("max gap, average gap, turn angle")
         print(max_gap,average_gap,turn_angle)

         print("angz: " + str(angz) + str(" degrees"))



def main():
   rospy.init_node('laser_avoid_obstacle')
   laserObstacleAvoid = LaserObstacleAvoid()

   rate = rospy.Rate(10)   # Hz

   while not rospy.is_shutdown():
      rate.sleep()
  
   # rospy.spin is last line of code when a subscriber is involved
   # ensures subscribed messages will arrive
   #rospy.spin()
   

if __name__ == "__main__":
    main()
    
