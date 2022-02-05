#!/usr/bin/env python3

# Import ROS stuff
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

# This class will receive the point and radius of the ball, to transform that into
# velocity commands using a proportional controller.
class FollowerClass():
    def __init__(self):
       
        ###********** START NODE **********###
        rospy.init_node("follower_vision") 

        ###********** CLOSE NODE **********###
        rospy.on_shutdown(self.cleanup)
        
        ###*********** VARIABLES **********###
        self.velocity = Twist() # The velocity to send to the robot
        self.img_half_x = 300 # Half of the image frame
        self.ball_x = 0 # Position in x of the ball inside frame
        self.radius = 0 # Radius of the ball
        self.kl = 3    # Proportional gain for linear speed

        ###*********** PUBLISHERS **********###
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        ###*********** SUBSCRIBERS **********###
        rospy.Subscriber('center', Point, self.center_cb)
        rospy.Subscriber('radius', Int32, self.radius_cb)
        rospy.Subscriber('kl', Int32, self.kl_cb)
        
        ###********** INIT NODE **********###
        self.r = rospy.Rate(10) # 10Hz 
        print("Node initialized 10hz")
        while not rospy.is_shutdown():
            if self.radius == 0:
                self.stop() # If no object is detected, stop the robot
            else:
                self.velocity_controller()  # Control velocity to approach ball
            self.r.sleep()
        
    
    def velocity_controller(self):
        kw = 0.001  # Proportional gain for angular speed

        x = self.img_half_x - self.ball_x   # Positive means its to the left of center frame
                                            # Negative means its to the right of center frame

        self.velocity = Twist() # Reset velocity variable to ceros
        if abs(x) > 15: # Only change angular speed if its not within 15px of center
            self.velocity.angular.z = kw*x  # Angular speed is proportional to the x distance
        if self.radius < 200 and self.radius > 50: # Only change linear speed if ball is further from range
            self.velocity.linear.x = self.kl * (1/self.radius)  # Linear speed is inversely proportional to the radius

        print('Linear X', self.velocity.linear.x)
        print('Angular Z', self.velocity.angular.z)

        self.pub_vel.publish(self.velocity) # Publish velocity to the robot
    
    def center_cb(self, _point):
        self.ball_x = _point.x  # Save x component of point to variable

    def radius_cb(self, msg):
        self.radius = msg.data  # Save radius of object to variable

    def kl_cb(self, msg):
        self.kl = msg.data  # Save kl of object to variable
    
    def stop(self):
        self.velocity = Twist() # Reset velocity variable to ceros
        self.pub_vel.publish(self.velocity) # Publish velocity to the robot

    def cleanup(self):
        self.stop() # Stop the robot

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__":
    try: FollowerClass()
    except rospy.ROSInterruptException: pass
