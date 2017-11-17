#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
class transform{
	public:
		transform(ros::NodeHandle&); //  n
	private:
		double sampleMethod(); //declaration of a private example method
		float ar_pos_x,ar_pos_y,ar_pos_z;
		

	
};
//Initialization for subscribers,publishers, variables and constants
transform::transform(ros::NodeHandle& n){
	ar_pos_x=0;
	ar_pos_y=0;
	ar_pos_z=0;

}
//Sample method
double transform::sampleMethod()
{
	return 0;
}
void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&)
{
  //ROS_INFO("ID: [%i]", ar_pose_marker.id.c_str());
}
int main(int argc, char **argv)
{
  ar_track_alvar_msgs::AlvarMarkers msg; 
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle n;
  ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1000, chatterCallback);
  ros::spin();

  return 0;
}
