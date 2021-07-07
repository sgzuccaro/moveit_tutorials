

#ifndef AUBO_HW_INTERFACE_H
#define AUBO_HW_INTERFACE_H
 
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <aubo_control/telem.h>
#include <aubo_control/cmd.h>

namespace aubo_control_ns
{

/// \brief Hardware interface for a robot
class auboHWInterface : public ros_control_boilerplate::GenericHWInterface
{
private:

  ros::Subscriber telemetry_sub;
  void telemetryCallback(const aubo_control::telem::ConstPtr &msg);

  ros::Publisher cmd_pub;

public:
  // brief Constructor \param nh - Node handle for topics. 
  auboHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  // brief Initialize the robot hardware interface 
  virtual void init();

  // brief Read the state from the robot hardware. REQUIRED or wont compile
  virtual void read(ros::Duration &elapsed_time);

  // brief Write the command to the robot hardware.  REQUIRED or wont compile  
  virtual void write(ros::Duration &elapsed_time);

  //  REQUIRED or wont compile
  virtual void enforceLimits(ros::Duration &period);


};  // class

}  // namespace
 
#endif
