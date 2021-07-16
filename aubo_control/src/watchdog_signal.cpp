#include <ros/ros.h>
#include <std_msgs/Bool.h> 
 
ros::Publisher watchdog_pub;

 
std_msgs::Bool watch_dog;
void watchDogCallback(const ros::TimerEvent&)
{	

	if( ros::ok() ){
		watch_dog.data = !watch_dog.data;
		watchdog_pub.publish(watch_dog);
	}

}


//// main ////
int main(int argc, char **argv) {
ros::init(argc, argv, "watchdog_signal");

	// prep for ROS communcation
	ros::NodeHandle n; 

	watchdog_pub = n.advertise<std_msgs::Bool>("/watch_dog", 1);

	ros::Timer watchdog_timer = n.createTimer(ros::Duration(0.025), watchDogCallback, false); // true/false oneshot
	 
	watch_dog.data=false;
 
    ros::spin();

} // end main






















