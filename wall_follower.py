#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import as1cfgConfig

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        
        self.prev_error = self.curr_error
        self.curr_error = current_error
        
        #Calculating the integral error
        self.sum_error = self.sum_error + self.curr_error*self.dt

        #Calculating the derivative error
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        #Calculating the PID Control
        self.control = self.Kp * self.curr_error + self.Ti * self.sum_error + self.Td * self.curr_error_deriv


    def get_control(self):
        return self.control
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50 

        #Setting the PID
        self.controller=PID(-1,-1.2, 0.00001,0.02)

        #Creating a cte_pub Publisher that will publish the cross track error
        self.cte_pub = rospy.Publisher("/husky_1/cte", String, queue_size=50)

        #Creating cmd_pub Publisher that will publish a Twist msg to cmd_vel
        self.cmd_pub = rospy.Publisher("/husky_1/cmd_vel", Twist, queue_size=50)
        
        self.msg = Twist()
        self.msg.linear.x = self.forward_speed

        #Publish the msg
        self.cmd_pub.publish(self, self.msg)
        
        #Creating a Subscriber that will call laser_scan_callback every Laser Scan
        self.laser_sub = rospy.Subscriber("/husky_1/scan", LaserScan, self.laser_scan_callback)
        
        #Server
        srv = Server(as1cfgConfig, self.serverCall)

    def laser_scan_callback(self, msg):
   
        cross_track_error=0
        distance_from_wall=999999

        #Finding the closest distance from the wall the robot reaches
        #Divided by 2 so it goes to the left wall and not the one on the right
       	for i in range(len(msg.ranges)/2):
       		if msg.ranges[i] < distance_from_wall:
       			distance_from_wall = msg.ranges[i]

       	#Calculating the cross track error
       	cross_track_error = self.desired_distance_from_wall - distance_from_wall
        
       	#Updating the controller and publishing the cross track error
       	self.controller.update_control(cross_track_error)
       	self.cte_pub.publish(str(cross_track_error))

       	cmd = Twist()
        cmd.linear.x = self.forward_speed

  		#Getting the new cmd.angular.z
       	cmd.angular.z = self.controller.get_control()
       	
       	#Publishing the cmd.linear.x
       	self.cmd_pub.publish(cmd)
   
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

    #Function that will be called when values are changed
    def serverCall(self, config, level):
    	self.controller.Kp = config['Kp']
    	self.controller.Td = config['Td']
    	self.controller.Ti = config['Ti']
    	return config

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()


