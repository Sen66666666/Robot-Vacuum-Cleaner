# Robot-Vacuum-Cleaner
## Introduction
Droidium aims to develop a realistic simulation of their mobile platform quickly and cost-effectively. I recommend leveraging their existing compatibility with ROS and using Gazebo. The robot hardware is based on an existing robot vacuum cleaner chassis. This approach allows the company to benefit from economies of scale, as the factory does not install the vacuum components but retains the sensing and actuation subsystems.
## Model Picture
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/vacuum_cleaner_model.png)
## Model in rviz and Gazebo
### In rviz 
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/urdf_model.png)
![image](https://github.com/Sen66666666/Robot-vacuum-cleaner/blob/main/Performance_of_lidar_and%20camera.png)
Since the model is published /model/iodine/scan and /model/iodine/depth,it is able to detect its surroundings and allows it to 'see' obstacles
### In Gazebo

