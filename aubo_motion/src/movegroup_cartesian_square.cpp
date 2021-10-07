#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "MoveGroupInterface_cartesian_square");
  ros::NodeHandle n;
 

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::MoveGroupInterface setEndEffectorLink (const std::string &tool0);

  ROS_INFO("First position robot in a home like pose");
  ROS_INFO("Begin Planning...");


  std::vector<geometry_msgs::Pose> waypoints;
  moveit::planning_interface::MoveGroupInterface::Plan plan;



  // get current pose
  geometry_msgs::Pose store_pose=move_group.getCurrentPose().pose;
  
  //move_group.setStartStateToCurrentState();

  //use orientation from current pose but redefine position
  store_pose.position.x = -0.1;
  store_pose.position.y = -0.3;
  store_pose.position.z = 0.4; 
  move_group.setPoseTarget(store_pose); 
  move_group.move();
  

  store_pose.position.x = 0.1; 
  waypoints.push_back(store_pose);

  store_pose.position.y = -0.5; 
  waypoints.push_back(store_pose);

  store_pose.position.x = -0.1; 
  waypoints.push_back(store_pose);

  store_pose.position.y = -0.3; 
  waypoints.push_back(store_pose);


  moveit_msgs::RobotTrajectory trajectory;
  move_group.setPlanningTime(10.0);

  move_group.setMaxVelocityScalingFactor(0.01); // not doing anything?


  // note if the path is only partially completeing then make sure no joints are near a limit, especially J6
  double fraction = move_group.computeCartesianPath(waypoints,
  0.005, // eef_step
  0.0,// jump_threshold
  trajectory, true);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  plan.trajectory_=trajectory;

  // velocities
  std::cout<< plan.trajectory_ <<std::endl;

  //trajectory->velocities;
  move_group.execute(plan);
}












