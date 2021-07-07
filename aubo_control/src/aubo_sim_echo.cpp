#include <ros/ros.h>
#include <aubo_control/telem.h>
#include <aubo_control/cmd.h> 
 
//// current position callback ////
// for subscribing to telemetry
ros::Publisher telem_pub;
aubo_control::telem telem_echo;
void cmdCallback(const aubo_control::cmd::ConstPtr &msg){
 
    for(int i=0; i<msg->pos.size(); i++){
        telem_echo.pos[i]=msg->pos[i]; 
    }

    telem_pub.publish(telem_echo);
}

 


//// main ////
int main(int argc, char **argv) {
    ros::init(argc, argv, "aubo_sim_echo");

    // prep for ROS communcation
    ros::NodeHandle n; 

    ros::Subscriber cmd_sub = n.subscribe("/command", 10, cmdCallback); // robot feedback

    telem_pub = n.advertise<aubo_control::telem>("/telemetry", 10);
 
 
    ros::spin();
 

} // end main

 
















