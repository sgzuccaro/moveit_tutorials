#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <aubo_control/aubo_hw_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aubo_hw_interface");
  ros::NodeHandle nh; 


  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  std::shared_ptr<aubo_control_ns::auboHWInterface> aubo_hw_interface_instance 
   (new aubo_control_ns::auboHWInterface(nh));
  aubo_hw_interface_instance->init();


  // Start the control loop
  ros_control_boilerplate::GenericHWControlLoop control_loop(nh, aubo_hw_interface_instance);
  control_loop.run(); // Blocks until shutdown signal recieved

  return 0;
}
