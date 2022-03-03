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
        self.left_sum = 0
        self.right_sum = 0

        self.target_locked = False
        self.goal_reached = False
        self.go_into_red_zone = False
        self.start = True
        self.wait = False
        self.laser_control = False

    def go_forward(self):
        twist_object = Twist()
        twist_object.linear.x = 0.4
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
        if self.goal_reached:
            rospy.loginfo("Target Reached !!")
            rospy.signal_shutdown("Target Reached")
            return

        regions = {
        'fright': min(min(data.ranges[140:190]), 6),
        'front':  min(min(data.ranges[150:250]), 6),
        'fleft':  min(min(data.ranges[210:260]), 6),
        'left':  min(min(data.ranges[261:320]), 6),
        'right':   min(min(data.ranges[70:139]), 6),
        }
        msg = Twist()
        linear_x = 0
        angular_z = 0

        state_description = ''

        self.left_sum += regions['left']
        self.right_sum += regions['right']

        if regions['front'] > 1:
            state_description = 'case 1 - nothing'
            linear_x = 0.5
            angular_z = 0
            self.laser_control = False;
        elif regions['front'] < 1:
            linear_x = 0
            if regions['fright'] > regions['fleft']:
                angular_z = -0.1
                state_description = 'case 2 - turning right to avoide obstacle'
            elif regions['fright'] < regions['fleft']:
                angular_z = 0.1
                state_description = 'case 3 - turning left to avoid obstacle'
            self.laser_control = True;
        else:
            state_description = 'unknown case'
            self.laser_control = False;

       

        if self.laser_control:
            rospy.loginfo(state_description)
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_pub.publish(msg)

    def camera_callback(self,data):

        # called when final destination is reached 
        if self.goal_reached:
            rospy.loginfo("Target Reached !!")
            rospy.signal_shutdown("Target Reached")
            return

        # moves the robot into the red zone
        if self.go_into_red_zone:
            self.go_forward()
            self.count += 1
            if self.count > 4:
                self.stop()
                self.go_into_red_zone = False
                self.goal_reached = True
                self.count = 0
            return

        # we don't need to do anything here while the laser callback is moving the robot 
        if self.laser_control:
            return
        if self.wait:
            self.count += 1
            if self.count > 1:
                self.wait = False
                self.stop()
                self.count = 0
            return
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, 
                desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
 
        

        # get image data
        height, width, channels = cv_image.shape

        descentre = 100
        target_range = 300
        rows_to_watch = 20

        # crop the image
        crop_img = cv_image[(height)/2+descentre-rows_to_watch:height/2+descentre,1:width]
        crop_target = cv_image[height/2-150: height/2+150, 1:width]

        
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        target_hsv = cv2.cvtColor(crop_target, cv2.COLOR_BGR2HSV)

        # applying range of colors to filter in hsv
        lower_gray = np.array([100,1,75])
        upper_gray = np.array([130,3,109])

        lower_target = np.array([0,216,214])
        upper_target = np.array([18,255,242])


        # Applying a mask
        mask = cv2.inRange(hsv, lower_gray, upper_gray)
        target_mask = cv2.inRange(target_hsv, lower_target, upper_target)

        cv2.imshow("side walk", mask)
        cv2.imshow("target", target_mask)
        cv2.waitKey(1)

        # counting number of pixels that matched our filter color
        pixels = np.sum(mask == 255)
        target_pixels = np.sum(target_mask == 255)

        
        ratio = float(pixels)/width*rows_to_watch *100
        target_ratio = float(target_pixels)/(width*target_range) *100

        #these two if blocks handle the white image sent at the start of gazebo 
        if  ratio > 0:
            self.start = False
        if  self.start and ratio == 0:
            return

        # initiates the move into the red zone
        if target_ratio > 71:
            self.go_into_red_zone = True
            return

        # handles turning at corners if the robot passed the side walk and intered the road
        if ratio < 20 and not self.target_locked:
            self.wait = True
            if self.left_sum > self.right_sum:
                self.turn_right()
                rospy.loginfo("TURNING RIGHT")
                self.left_sum = 0
                self.right_sum = 0
            else:
                self.turn_left()
                rospy.loginfo("TURNING LEFT")
                self.left_sum = 0
                self.right_sum = 0
            return

        # if target is not locked we use the center of the sidewalk to stear the robot         
        if target_ratio > 10:
            m = cv2.moments(target_mask, False)
            rospy.loginfo("locked target")
            self.target_locked = True
        else:
            m = cv2.moments(mask, False)
       
            
        #finding center of the masked image x and y
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except:
            cx, cy = height/2,width/2

        error_x = cx - width / 2;

        
        twist_object = Twist();
        twist_object.linear.x = .7;
        if abs(error_x) > width/15:
            twist_object.linear.x = .4;
            twist_object.angular.z = 0.08 if error_x < 1 else -0.08

        self.cmd_vel_pub.publish(twist_object)

    def clean_up(self):
        cv2.destroyAllWindows()
    

def main():
    rospy.init_node('side_walk_follower', anonymous=True)
    line_follower_object = LineFollower()


    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("shutting down")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rate.sleep()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()