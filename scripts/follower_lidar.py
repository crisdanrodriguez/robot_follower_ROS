#!/usr/bin/env python3

# Import ROS stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# Import the necessary packages
import numpy as np
import math


# This class will receive a laser scan, to transform that into
# velocity commands using a proportional controller for following the object.
class FollowerClass():
    def __init__(self):
       
        ###********** START NODE **********###
        rospy.init_node("follower_lidar") 

        ###********** CLOSE NODE **********###
        rospy.on_shutdown(self.cleanup)
        
        ###*********** VARIABLES **********###
        self.velocity = Twist() # The velocity to send to the robot
        self.closest_range = 0  # Distance to closest object detected
        self.closest_angle = 0  # Angle to closest object detected

        ###*********** PUBLISHERS **********###
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        ###*********** SUBSCRIBERS **********###
        rospy.Subscriber('front/scan', LaserScan, self.laser_cb)
        
        ###********** INIT NODE **********###
        self.r = rospy.Rate(10) # 10Hz 
        print("Node initialized 10hz")
        while not rospy.is_shutdown():
            self.velocity_controller()  # Control velocity to approach object
            self.r.sleep()
    
    def velocity_controller(self):
        kl = 0.25    # Proportional gain for linear speed
        kw = 0.35    # Proportional gain for angular speed

        self.velocity = Twist() # Reset velocity variable to zeros

        if self.closest_range > 0.5:    # Only change linear speed if object is not within range
            self.velocity.linear.x = kl * self.closest_range
        self.velocity.angular.z = kw * self.closest_angle   # Angular speed is always adjusting to center the object

        self.pub_vel.publish(self.velocity) # Publish velocity to the robot
    
    def laser_cb(self, msg):
        self.closest_range = min(msg.ranges)    # Obtain the closest object detected
        print('Range: ', self.closest_range)
        
        if self.closest_range == math.inf:  # If no object is detected set vairiables to cero
            self.closest_range = 0
            self.closest_angle = 0
            return
        
        closest_index = msg.ranges.index(self.closest_range)    # Obtain the index of the closest object
        self.closest_angle = (msg.angle_min + closest_index * msg.angle_increment) + math.radians(120)   # Compute the angle to the object
        # math.radians(120) is used to adjust the angular dispĺacement between the LiDAR's 'x' orientation and the robot's one 

    
    def stop(self):
        print('Stopping the robot')
        self.velocity = Twist() # Reset velocity variable to ceros
        self.pub_vel.publish(self.velocity) # Publish velocity to the robot

    def cleanup(self):
        self.stop() # Stop the robot

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__":
    try: FollowerClass()
    except rospy.ROSInterruptException: pass
