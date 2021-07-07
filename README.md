# Using Moveit with a Real Robot Tutorials


1) first create a moveit configuration by following the tutorial at: https://www.youtube.com/watch?v=9aK0UDBKWT8
- this will result in 2 packages: aubo_description and aubo_moveit_config. Note that neither of these package need any modifing. they are ready to use directly as genetated.

2) Now it is time to connect to the actual robot hardware. We begin by assuming you have a robot with a microcontroller connected that exposes some method of sending joint commands a well as makes available a continious stream of robot joint feedback. This could be a ROS publish and Subsscribe interface, but it could also be low level serial or ethernet control or other protocol.
