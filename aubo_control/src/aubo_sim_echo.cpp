#include <ros/ros.h>
#include <aubo_control/auboTelemetry.h>
#include <aubo_control/armCmd.h> 
 
//// current position callback ////
// for subscribing to telemetry
ros::Publisher telem_pub;
aubo_control::auboTelemetry telem_echo;
void cmdCallback(const aubo_control::armCmd::ConstPtr &msg){
 
    for(int i=0; i<msg->angle.size(); i++){
        telem_echo.angle[i]=msg->angle[i]; 
    }

    telem_pub.publish(telem_echo);
}

 


//// main ////
int main(int argc, char **argv) {
    ros::init(argc, argv, "aubo_sim_echo");

    // prep for ROS communcation
    ros::NodeHandle n; 

    ros::Subscriber cmd_sub = n.subscribe("/teensy/armCmd", 10, cmdCallback); // robot feedback

    telem_pub = n.advertise<aubo_control::auboTelemetry>("/teensy/auboTelemetry", 10);
 
 
    ros::spin();
 

} // end main

 
















