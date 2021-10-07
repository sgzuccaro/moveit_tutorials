#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>



int main(int argc, char **argv)
{

  ros::init(argc, argv, "movegroup_RPY");
  ros::NodeHandle n;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("manipulator");


	// set max jont velocity and acceleration 0-1
	double max_vel=0.05;
	double max_accel=0.05;
	move_group.setMaxVelocityScalingFactor(max_vel);
	move_group.setMaxAccelerationScalingFactor(max_accel);



	ROS_INFO("Begin Planning...");



	moveit::planning_interface::MoveGroupInterface setEndEffectorLink (const std::string &wrist3_joint);
	

	double deg_to_rad=3.1416/180;


	double Yaw;
	double Pitch;
	double Roll;

	tf::Matrix3x3 torch_angle;

	Yaw=0*deg_to_rad; // blue axis z 
	Pitch=0*deg_to_rad; // geen axis y
	Roll=-180*deg_to_rad; // red axis x
	torch_angle.setEulerYPR(Yaw,Pitch,Roll);

	tf::Quaternion q_tf;
	torch_angle.getRotation(q_tf);

	geometry_msgs::Pose pose1;
	pose1.orientation.w = q_tf.getW();
	pose1.orientation.x = q_tf.getX();
	pose1.orientation.y = q_tf.getY();
	pose1.orientation.z = q_tf.getZ();
	pose1.position.x = 0.1;
	pose1.position.y = -0.3;
	pose1.position.z = 0.1;
	move_group.setPoseTarget(pose1);
	move_group.move();

 
}












