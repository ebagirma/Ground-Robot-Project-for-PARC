#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.last_corner = 1
        self.count = 0
        self.target_locked = False
        self.goal_reached = False
        self.go_into_red_zone = False
        self.green = False

    def go_forward(self):
        twist_object = Twist()
        twist_object.linear.x = 0.5
        twist_object.angular.z = 0
        self.cmd_vel_pub.publish(twist_object)

    def turn_right(self):
        twist_object = Twist()
        twist_object.linear.x = 0
        twist_object.angular.z = -0.3
        self.cmd_vel_pub.publish(twist_object)


    def turn_left(self):
        twist_object = Twist()
        twist_object.linear.x = 0
        twist_object.angular.z = 0.3
        self.cmd_vel_pub.publish(twist_object)

    def stop(self):
        twist_object = Twist()
        twist_object.linear.x = 0
        twist_object.angular.z = 0
        self.cmd_vel_pub.publish(twist_object)

   
    def camera_callback(self,data):
        if self.goal_reached:
            rospy.loginfo("Target Reached !!")
            rospy.signal_shutdown("Target Reached")
            return
        if self.go_into_red_zone:
            self.go_forward()
            self.count += 1
            if self.count > 10:
                self.go_into_red_zone = False
                self.goal_reached = True
                self.stop()
                self.count = 0
            return
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
 
        

        # get image data
        height, width, channels = cv_image.shape

        target_range = 300

        crop_target = cv_image[height/2-150: height/2+150, 1:width]
        light_crop = cv_image[0:height/2, 1: width]

        target_hsv = cv2.cvtColor(crop_target, cv2.COLOR_BGR2HSV)
        light_hsv = cv2.cvtColor(light_crop, cv2.COLOR_BGR2HSV)

        lower_green = np.array([34,254,229])
        upper_green = np.array([80,255,230])

        lower_red = np.array([0,250,228])
        upper_red = np.array([6,255,229])

        lower_target = np.array([0,236,221])
        upper_target = np.array([7,253,246])

        target_mask = cv2.inRange(target_hsv, lower_target, upper_target)
        red_mask = cv2.inRange(light_hsv, lower_red, upper_red)
        green_mask = cv2.inRange(light_hsv, lower_green, upper_green)

        cv2.imshow("target", target_mask)
        cv2.waitKey(1)

        target_pixels = np.sum(target_mask == 255)
        red_pixels = np.sum(red_mask ==  255)
        green_pixels = np.sum(green_mask == 255)

        target_ratio = float(target_pixels)/(width*target_range) *100
        red_ratio = float(red_pixels)/(width*height/2) *100
        green_ratio = float(green_pixels)/(width*height/2) *100
        
        
        #sets the green flag
        if green_ratio > red_ratio:
            self.green = True
        
        if not self.green:
            rospy.loginfo("wating for green light")
            return
        else:
            rospy.loginfo("green light given, going to target")
        
        if target_ratio > 10:
            rospy.loginfo("making the final move to get into target")
            self.go_into_red_zone = True
            return
        m = cv2.moments(target_mask, False)
       
       #finding center of the masked image x and y
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except:
            cx, cy = height/2,width/2
        
        error_x = cx - width / 2;

        twist_object = Twist();
        twist_object.linear.x = .8;
        if abs(error_x) > width/10:
            twist_object.angular.z = 0.1 if error_x < 0 else -0.1
        self.cmd_vel_pub.publish(twist_object)

    def clean_up(self):
        cv2.destroyAllWindows()
    

def main():
    rospy.init_node('traffic_sign_detection_and_recognition', anonymous=True)
    line_follower_object = LineFollower()
    


    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("shutdown")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()