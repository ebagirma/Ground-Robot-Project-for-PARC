# TEAM 1: PARC Engineers League 

## Introduction

In this section, you should give an overview of the competition task, briefly describe your understanding of delivery robotics and how relevant it is in an African context (*This should be only 4-5 sentences*)

**Team Country:** African country

**Team Member Names:**

* Bereket Kassahun Degefu
* Ebasa G. Temesgen 
* Abel Kidanemariam Teka



## Dependencies

**Packages needed are:**

* `ros_control`: ROS packages including controller interfaces, controller managers, transmissions, etc.
    * `$ sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`
* `CvBridge`: The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
    * `$ sudo apt-get install ros-melodic-cv-bridge'`
* `opencv`: Open-source computer vision library.
    * `$ sudo apt-get install python-opencv`
* `numpy`: Fast mathematical operations over arrays 
    * `$ pip install numpy`
* `CvBridge`: It contains the Python interface for CvBridge, which converts between ROS Image messages and OpenCV images.
    * `$ apt install python-cv-bridge`

## Task 1

The goal of this task is to autonomously control the delivery robot from the robot start position to a goal location on the sidewalk. To do that we based our lane detaction based on the image we subscribed from the robot and the laser sensor. we used 
the lane for stearing the robot and the laser from /scan topic to avoid obstacles. The robot stops when pixels of the target are significantly large.

the following is the command to run task 1
`roslaunch task1 task1_solution.launch `



## Task 2

The goal of this task is to cross the the road, we  detected three different objects from the image we got, the red light, the green light, and the target. The robot starts movinig whenever pixels of green light are greater than the red. The robot stops when pixels of the target are significantly large.

the following is the command to run task 2
`roslaunch task2 task2_solution.launch`



## Task 3

The goal of this task is to naviagate to the goal with minimum distance. we have used only laser information to stear the robot untill it locks on target.The robot stops when pixels of the target are significantly large.

the following is the command to run task 2
`roslaunch task3 task3_solution.launch `



## Challenges Faced

1. the laser scan was present at /scan topic but it was not sending the real data as the gazebo was configured to use gpu laser, we beleive this was an issue for us because our pc didn't have GPU.We solved it by changing some gazebo files.
2. We beleive that we have a slower computers so some of the assumptions we take could be wrong, for example to make the final move into the red zone we call go forward upto certain time, the result could greatly vary depending on the perfomance the computers.

