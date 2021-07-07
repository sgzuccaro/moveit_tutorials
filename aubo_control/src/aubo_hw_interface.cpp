

#include <aubo_control/aubo_hw_interface.h>

namespace aubo_control_ns
{

auboHWInterface::auboHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) 
 : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{


  telemetry_sub = nh.subscribe("/telemetry", 1, &auboHWInterface::telemetryCallback, this);

  cmd_pub= nh.advertise<aubo_control::cmd>("/command", 1);
  ROS_INFO("auboHWInterface declared.");
}



void auboHWInterface::telemetryCallback(const aubo_control::telem::ConstPtr &msg){
     

    for(int i=0; i<num_joints_; i++){
      joint_velocity_[i] = msg->vel[i]; // declared in GenericHWInterface
      joint_position_[i] = msg->pos[i];
    }

}


void auboHWInterface::init()
{
  // Call parent class version of this function
  ros_control_boilerplate::GenericHWInterface::init();

  // Resize vectors
  //joint_position_prev_.resize(num_joints_, 0.0);

  ROS_INFO("auboHWInterface initiated.");
}


void auboHWInterface::read(ros::Duration &elapsed_time)
{
  ros::spinOnce(); // fire callback telem
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR READ COMMAND FROM USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void auboHWInterface::write(ros::Duration &elapsed_time)
{
  static aubo_control::cmd cmd_;

  /*
    Header header
    float32[6] pos
  */

  for(int i=0; i<num_joints_; i++){
    cmd_.pos[i]=joint_position_command_[i];
  }

  cmd_pub.publish(cmd_);
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  //for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
  //  joint_position_[joint_id] += joint_position_command_[joint_id];
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}


void auboHWInterface::enforceLimits(ros::Duration &period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace
