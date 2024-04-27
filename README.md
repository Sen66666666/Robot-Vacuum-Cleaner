# Robot-Vacuum-Cleaner
## Introduction
Droidium aims to develop a realistic simulation of their mobile platform quickly and cost-effectively. I recommend leveraging their existing compatibility with ROS and using Gazebo. The robot hardware is based on an existing robot vacuum cleaner chassis. This approach allows the company to benefit from economies of scale, as the factory does not install the vacuum components but retains the sensing and actuation subsystems.
## Procedure
1.Download coursework2 and iodine_description in your ros2 workspace.

2.Compile your ros2 workspace
  ```
  colcon build
  source install/setup.bash
  ```  
3. Launch file
  ```
  ros2 launch iodine_description iodine_sim_bringup.launch.py
  ```
## Model Picture
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/vacuum_cleaner_model.png)
## Model in rviz and Gazebo
### In rviz 
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/urdf_model.png)
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/Performance_of_lidar_and%20camera.png)
Since the model is published /model/iodine/scan and /model/iodine/depth,it is able to detect its surroundings and allows it to 'see' obstacles
### In Gazebo
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/Vacuum%20cleaner%20in%20gazebo.png)

## Control
You can use your keyboard to control the vacuum cleaning robot to explore its surroundings.
[![Watch the video](https://img.youtube.com/vi/GLNqCRScQQM/0.jpg)](https://www.youtube.com/watch?v=GLNqCRScQQM)
