#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "movegroup_jointspace");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // creating planning group
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


  // get current joint angles
  std::vector<double> current_joint_states;
  current_joint_states=move_group.getCurrentJointValues();
   

  // flip sign for new goal
  for(int i=0; i<current_joint_states.size(); i++){ current_joint_states[i]=current_joint_states[i]*-1; }
 
  move_group.setJointValueTarget(current_joint_states);
  move_group.move();

  // delay
  ros::Duration(1).sleep();

  // go to a new random pose
  move_group.setJointValueTarget(move_group.getRandomJointValues());
 
  move_group.move();

 
}
