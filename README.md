# ROS_Gazebo_Simulator_with_Opticalflow
Controlling turtlebot with optical flow provided by OpenCV in Gazebo simulator

![image](https://user-images.githubusercontent.com/50229148/149880334-b706dbff-9f82-476b-a101-934ddc42cbd8.png)

## Setup and Requirements
* Environment : Ubuntu 16.04(VMware) / ROS_kinetic 
* ROS packages
1) OpenCV<br>
```Installation command : $ sudo apt install ros-kinetic-opencv*```
2) Usb_cam<br>
```Installation command : $ sudo apt install ros-kinetic-usb-cam```<br>
```Execution command : $ rosrun usb_cam usb_cam_node```

## Abstract
Subscribing image-messages that Camera node published, define angular and velocity with optical flow that provided in ROS_OpenCV.<br>
With angular and velocity, publish geometry messages being used in controlling turtlebot.<br>
Instead of using turtlebot in real world, I used Gazebo simulator to execute my project.  

## Results
Moving the book up/down, left/right in screen, turtlebot moves toward the direction equal to book.
![image](https://user-images.githubusercontent.com/50229148/149881295-2306e655-09dc-4d47-9f9a-5bf374d91ee7.png)


## Reference
* GitHub - tzutalin/ros_sample_image_transport: ROS Receive and publish different topics about image
* turtlebot_apps/turtlebot_teleop at hydro · turtlebot/turtlebot_apps · GitHub
* https://95mkr.tistory.com/entry/ROS6 
