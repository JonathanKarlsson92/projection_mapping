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
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&msg)
{
	if(!msg->markers.empty()){
		ROS_INFO("position_x: [%f]", msg->markers[0].pose.pose.position.x); //Dist from camera
		//ROS_INFO("position_y: [%f]", msg->markers[0].pose.pose.position.y); //??
		//ROS_INFO("position_z: [%f]", msg->markers[0].pose.pose.position.z); //??

		ROS_INFO("orientation_x: [%f]", msg->markers[0].pose.pose.orientation.x);
		//ROS_INFO("orientation_y: [%f]", msg->markers[0].pose.pose.orientation.y);
		//ROS_INFO("orientation_z: [%f]", msg->markers[0].pose.pose.orientation.z);
	}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle n;
  ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1000, poseCallback);
  ros::spin();

  return 0;
}

///Todo
// initialize pose to avoid huge values
// subscribe to relative projector pose
// enable tracking of multiple cubes
// 
