<h1 align="center">
  Skid Steering Mobile Robot Odometry
</h1>

Project for Robotics course 2020/2021

<h3>High-Level Goals:</h3>
1.  compute odometry using skid steering (approx) kinematics: <br />
&nbsp;  -  using Euler and Runge-Kutta integration; <br />
&nbsp;  -  ROS parameter specifies initial pose. <br />
2.  use dynamic reconfigure to select between integration method <br />
3.  write 2 services to reset the odometry to (0,0) or to a pose(x,y,θ) <br />
4.  publish a custom message with odometry value and type of integration <br />

<h3>Files:</h3>

 <b>cfg </b>
-  parameter.cfg: allows you to switch between "Euler" and "Runge-Kutta" for odometry integration ("Euler" is default).
-  initial_pose.yaml: contains the initial pose for both bags (for the 2nd one is just a default one), later parsed in odometry.cpp.

<b> msg </b>
-  CustomOdometry.msg: message for odometry: essentally nav_msgs/Odometry plus the name of the integration method currently used.
-  MotorSpeed.msg: provided msg to read the wheel's speeds.
   
<b>src</b> 
-  baseline_calculation_node.cpp: synchronizes wheel's speed topics to estimated the gear ratio and the apparent baseline;
-  publish_speed.cpp: synchronizes wheel's speed topics and computes linear and angular speeds;
-  odometry.cpp: computes odometry, creates the tf, publishes messages, allows dynamic reconfigure, allows the services to set the robot's pose to either zero or a defined value.

<b>srv</b>
-  SetToPose.srv: service to set the robot's pose to a defined value. 

<h3>How to launch files:</h3>
Set rviz to true/false if you want to launch RViz as well when computing the odometry.
Both launch files use bag1.

```shell
roslaunch robotics_hw1 odometry.launch
```
```shell
roslaunch robotics_hw1 baseline_calculation.launch
```
