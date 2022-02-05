#!/usr/bin/env python3

# Import ROS stuff
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# Import the necessary packages
import numpy as np
import cv2
import imutils

# This class will receive a raw image, then calibrate the mask using HSV values
# to obtain the radius and center point of the object detected
class BallTracker():
    def __init__(self):
        
        ###********** START NODE **********###
        rospy.init_node("ball_tracker")

        ###********** CLOSE NODE **********###
        rospy.on_shutdown(self.cleanup)

        ###*********** VARIABLES **********###
        self.bridge_object = CvBridge() # Creates the bridge object between ROS and opencv images
        self.center_ros = Point()   # Stores the center point of the detected object
        self.radius_ros = 0 # Stores the radius of the object detected
        self.cnt_length = 0 # Stores how many contours where found (detected objects)

        ###*********** FUNCTIONS **********###
        self.init_trackbars()

        ###*********** PUBLISHERS **********###
        self.pub_center = rospy.Publisher('center', Point, queue_size=10)
        self.pub_radius = rospy.Publisher('radius', Int32, queue_size=10)
        self.pub_kl = rospy.Publisher('kl',Int32, queue_size=10)

        ###*********** SUBSCRIBERS **********###
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback)

        ###********** INIT NODE **********###
        ros_rate = rospy.Rate(10) #10Hz
        print("Node initialized 10hz")
        while not rospy.is_shutdown():
            if self.cnt_length == 0:    # If no object is detected, send all ceros
                self.center_ros = Point()
                self.radius_ros = 0
            
            # Publish the data
            self.pub_center.publish(self.center_ros)
            self.pub_radius.publish(self.radius_ros)
            
            ros_rate.sleep()

        # Close all windows
        cv2.destroyAllWindows()

    def camera_callback(self, data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)


        # Resize the frame, blur it, and convert it to the HSV
        # color space
        self.frame = imutils.resize(self.frame, width=600)
        blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Get trackbar values
        ilowH = cv2.getTrackbarPos('lowH', 'image')
        ihighH = cv2.getTrackbarPos('highH', 'image')
        ilowS = cv2.getTrackbarPos('lowS', 'image')
        ihighS = cv2.getTrackbarPos('highS', 'image')
        ilowV = cv2.getTrackbarPos('lowV', 'image')
        ihighV = cv2.getTrackbarPos('highV', 'image')
        erode = cv2.getTrackbarPos('erode', 'image')
        dilate = cv2.getTrackbarPos('dilate', 'image')

        # Get linear gain trackbar value and publish it
        kl = cv2.getTrackbarPos('kl', 'image')
        self.pub_kl.publish(kl)

        # Define the HSV range of the desired object to be detected
        hsvLower = (ilowH, ilowS, ilowV)
        hsvUpper = (ihighH, ihighS, ihighV)
        
        # Construct a mask for the color "red", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, hsvLower, hsvUpper)
        mask = cv2.erode(mask, None, iterations=erode)
        mask = cv2.dilate(mask, None, iterations=dilate)
        cv2.imshow("Mask", mask)

        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # Only proceed if at least one contour was found
        self.cnt_length = len(cnts)
        if self.cnt_length > 0:
            # Find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                self.cnt_length = 0
                pass

            # Only proceed if the radius meets a minimum size
            if radius > 10:
                # Save the center and radius of the detected object
                self.center_ros.x = float(x)
                self.center_ros.y = float(y)
                self.center_ros.z = 0 # As it is an image z is not used.
                self.radius_ros = int(radius)

                # Draw the circle and centroid on the frame
                cv2.circle(self.frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(self.frame, center, 5, (0, 0, 255), -1)

        # show the frame to our screen
        cv2.imshow("Frame", self.frame)
        key = cv2.waitKey(1) & 0xFF

    def init_trackbars(self):
        
        # Initialize window for trackbars
        cv2.namedWindow('image')

        # Initial track bar values
        ilowH = 0
        ihighH = 13

        ilowS = 66
        ihighS = 255

        ilowV = 115
        ihighV = 255

        erode = 5
        dilate = 10

        kl = 10

        # Create trackbars for HSV limits, erode, dilate, and linear gain
        cv2.createTrackbar('lowH','image',ilowH,255,self.callback)
        cv2.createTrackbar('highH','image',ihighH,255,self.callback)

        cv2.createTrackbar('lowS','image',ilowS,255,self.callback)
        cv2.createTrackbar('highS','image',ihighS,255,self.callback)

        cv2.createTrackbar('lowV','image',ilowV,255,self.callback)
        cv2.createTrackbar('highV','image',ihighV,255,self.callback)

        cv2.createTrackbar('erode','image',erode,10,self.callback)
        cv2.createTrackbar('dilate','image',dilate,10,self.callback)

        cv2.createTrackbar('kl','image',kl,30,self.callback)

    def callback(x, y): # Empty function needed for trackbar
        pass

    def cleanup(self):  # Function to run when node is closed
        pass


if __name__ == '__main__':
    try: BallTracker()
    except rospy.ROSInterruptException: pass
