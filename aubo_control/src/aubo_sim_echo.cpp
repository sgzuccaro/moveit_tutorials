#include <ros/ros.h>
#include <aubo_control/auboTelemetry.h>
#include <aubo_control/armCmd.h> 
#include <deque> 

std::deque<aubo_control::armCmd> cmd_queue;

//// current position callback ////
// for subscribing to telemetry
ros::Publisher telem_pub;

void cmdCallback(const aubo_control::armCmd::ConstPtr &msg){
 
    cmd_queue.push_back(*msg);


}


aubo_control::armCmd current_cmd;
aubo_control::auboTelemetry telem;
void ISRCallback(const ros::TimerEvent&)
{	

    if( !cmd_queue.empty() ){
        current_cmd=cmd_queue.front(); // read from queue


        for(int i=0; i<current_cmd.angle.size(); i++){
            telem.angle[i]=current_cmd.angle[i];
            telem.vel[i]=current_cmd.vel[i]; 
        }


        cmd_queue.pop_front(); // remove (pop off) the first element of the queue

    }

    telem.bufferHealth=cmd_queue.size();
    telem_pub.publish(telem);

		


}


 
// create a timer callback they fires at 181 hz . if points are recieved then add them to a buffer. read from this buffer from the timer. 

//// main ////
int main(int argc, char **argv) {
    ros::init(argc, argv, "aubo_sim_echo");

    // prep for ROS communcation
    ros::NodeHandle n; 

    ros::Subscriber cmd_sub = n.subscribe("/teensy/armCmd", 10, cmdCallback); // robot feedback

    telem_pub = n.advertise<aubo_control::auboTelemetry>("/teensy/auboTelemetry", 10);

    ros::Timer isr_timer = n.createTimer(ros::Duration(0.0055), ISRCallback, false); // true/false oneshot
	
 
    for(int i=0; i<6; i++){
        telem.angle[i]=0;
        telem.vel[i]=0; 
    }

 
    ros::spin();
 

} // end main

 
















