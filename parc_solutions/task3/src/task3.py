#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import numpy as np

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.count = 0
        self.target_locked = False
        self.goal_reached = False
        self.go_into_red_zone = False

    def go_forward(self):
        twist_object = Twist()
        twist_object.linear.x = 0.3
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

    def laser_callback(self, data):
        if self.target_locked:
            return
        regions = {
            'fright': min(min(data.ranges[120:150]), 6),
            'front':  min(min(data.ranges[150:250]), 6),
            'fleft':  min(min(data.ranges[250:280]), 6),
            'left':  min(min(data.ranges[281:320]), 6),
            'right':   min(min(data.ranges[80:119]), 6),
        }
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''


        if  regions['left'] < 1.5 and regions['front'] > 2:
            state_description = 'turning right, avoiding obstacle'
            linear_x = 0.5
            angular_z = -0.1
        elif regions['right'] < 1.5 and regions['front'] > 2:
            state_description = 'turning left, avoiding obstacle'
            linear_x = 0.5
            angular_z = 0.1
        elif regions['left'] < 3 and regions['front'] > 2:
            state_description = 'turning left, following path'
            linear_x = 0.5
            angular_z = 0.1
        elif regions['right'] < 3 and regions['front'] > 2:
            state_description = 'turning right, follwing path'
            linear_x = 0.5
            angular_z = -0.1
        elif regions['front'] < 1:
            linear_x = 0
            if min(regions['fright'], regions['right']) > min(regions['fleft'], regions['left']):
                angular_z = -0.1
                state_description = "turning right, avoiding obstacle infront"
            elif min(regions['fright'], regions['right']) < min(regions['fleft'], regions['left']):
                angular_z = 0.1
                state_description = "turning left, avoiding obstacle infront"
        elif regions['front'] > 1:
            linear_x = 0.6
            state_description = 'going forward'

        rospy.loginfo(state_description)
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def camera_callback(self,data):

        if self.goal_reached:
            rospy.loginfo("Target Reached !!")
            rospy.signal_shutdown("Target Reached")
            return
        if self.go_into_red_zone:
            self.go_forward()
            self.count += 1
            if self.count > 5:
                self.go_into_red_zone = False
                self.goal_reached = True
                self.stop()
                self.count = 0
            return
 
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, 
                desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
 
        

        height, width, channels = cv_image.shape

        target_range = 300
        crop_target = cv_image[height/2-200: height/2+100, 1:width]
       
        target_hsv = cv2.cvtColor(crop_target, cv2.COLOR_BGR2HSV)

        lower_target = np.array([0,216,214])
        upper_target = np.array([18,255,242])

        target_mask = cv2.inRange(target_hsv, lower_target, upper_target)

        cv2.imshow("target", target_mask)

        cv2.waitKey(1)

        target_pixels = np.sum(target_mask == 255)


        target_ratio = float(target_pixels)/(width*target_range) *100
        
        # print("target_ratio:", target_ratio)
        if target_ratio > 80:
            self.go_into_red_zone = True
            return

    
        if target_ratio > 3:
            rospy.loginfo("target locked")
            self.target_locked = True
        if self.target_locked:
            m = cv2.moments(target_mask, False)

            #finding center of the masked image x and y
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except:
                return

            error_x = cx - width/2

            twist_object = Twist();
            twist_object.linear.x = .6;
            if abs(error_x) > width/17:
                twist_object.angular.z = 0.05 if error_x < 0 else -0.05
            self.cmd_vel_pub.publish(twist_object)
            return

    def clean_up(self):
        cv2.destroyAllWindows()
    

def main():
    rospy.init_node('go_to_goal_navigation', anonymous=True)
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