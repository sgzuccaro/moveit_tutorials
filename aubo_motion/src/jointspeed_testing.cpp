#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#define DEG_TO_RAD 0.01745329251


int main(int argc, char **argv)
{

    ros::init(argc, argv, "jointspeed_testing");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // creating planning group
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


    // get current joint angles
    std::vector<double> current_joint_states;
    current_joint_states={0,0,0,0,0,0};

    move_group.setJointValueTarget(current_joint_states);
    move_group.move();

    ROS_INFO("reached home");
    ros::Duration(1).sleep();

    ROS_INFO("moving single joint");
    int joint_num=0;
    double goal_angle=170;

    current_joint_states[joint_num] = goal_angle*DEG_TO_RAD;


    move_group.setJointValueTarget(current_joint_states);
    move_group.move();
 

    // max velocity 60 deg/s goal 90deg
 
}
