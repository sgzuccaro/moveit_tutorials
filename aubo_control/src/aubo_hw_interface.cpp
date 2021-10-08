

#include <aubo_control/aubo_hw_interface.h>

namespace aubo_control_ns
{

auboHWInterface::auboHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model) 
 : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{


  telemetry_sub = nh.subscribe("/teensy/auboTelemetry", 1, &auboHWInterface::telemetryCallback, this);

  cmd_pub= nh.advertise<aubo_control::armCmd>("/teensy/armCmd", 3);
  ROS_INFO("auboHWInterface declared.");
}



void auboHWInterface::telemetryCallback(const aubo_control::auboTelemetry::ConstPtr &msg){
     
     /*
    #Header header 
    float32[6] angle # degrees
    float32[6] vel # deg/s
    float32[6] current # amps
    #time armReadTimestamp 
    time startSyncTime 
    uint32 isrTicks # this would overflow if the robot is left on for 497 days straight at 100 hz 
    uint8 bufferHealth
    */

    for(int i=0; i<num_joints_; i++){
      joint_velocity_[i] = msg->vel[i]*DEG_TO_RAD; // declared in GenericHWInterface
      joint_position_[i] = msg->angle[i]*DEG_TO_RAD;
    }

    bufferHealth=msg->bufferHealth;

}


void auboHWInterface::init()
{
  // Call parent class version of this function
  /*
  this looks at controller yaml "hardware" namespace to get "joints". from this list the number of joints is known so hardware interfaces are initialized.
  it starts a joint_state, position, velocity and effort iterface. joint limits are also grabbed from parameter server urdf if urdf=NULL.
  */
  ros_control_boilerplate::GenericHWInterface::init();
  
  // array for storing previous state (for velocity calculation)
  joint_position_prev_.resize(joint_position_.size());

  ROS_INFO("auboHWInterface initiated.");
}


void auboHWInterface::read(ros::Duration &elapsed_time)
{
  //ros::spinOnce(); //is not required here because of asyncspinner
 
}

void auboHWInterface::write(ros::Duration &elapsed_time)
{
  static aubo_control::armCmd cmd_;

  /*
  float32[6] current #amps
  float32[6] accel #deg/s^2
  float32[6] vel #deg/s
  float32[6] angle #deg
  uint32 msg_ctr # count sent msgs to detected missed messages

  // Available Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  */

  /*
  caculate at a much higher rate then needed. then only send ones needed to fill buffer.

  */

  // only publish a msg if it has a change
  bool change_detected=false;
  for(int i=0; i<num_joints_; i++){
    if(joint_position_prev_[i] != joint_position_command_[i]){
      change_detected=true;
      i=num_joints_; // exit loop
    }
  }

  // if a new msg is available then send it
  if(change_detected){
    for(int i=0; i<num_joints_; i++){
      cmd_.angle[i]=joint_position_command_[i]*RAD_TO_DEG;
      cmd_.vel[i]= (fabs(joint_position_command_[i]-joint_position_prev_[i])*RAD_TO_DEG)/elapsed_time.toSec(); // (must be positive for aubo) joint_velocity_command_[i]*RAD_TO_DEG; joint_velocity_command_[i] calculate my own velocities
      cmd_.accel[i]=4*RAD_TO_DEG; // a max acceleration limit (must be positive for aubo)

      
      //cmd_.eff[i]=joint_effort_command_[i]; 

      joint_position_prev_[i]=joint_position_command_[i];
    }
 
    // if this point is needed then send it
    if(bufferHealth<DESIRED_BUFFERED_POINTS){
      cmd_.msg_ctr=cmd_.msg_ctr+1;
      cmd_pub.publish(cmd_);
    }

  } // changed detected

} // write


void auboHWInterface::enforceLimits(ros::Duration &period)
{
  // Enforces position and velocity
  //pos_jnt_sat_interface_.enforceLimits(period);
}


}  // namespace
