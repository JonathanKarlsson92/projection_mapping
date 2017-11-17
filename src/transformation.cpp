#include "ros/ros.h"
#include "std_msgs/String.h"

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
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle n;
  ros::Subscriber ar_pose = n.subscribe("tf", 1000, chatterCallback);
  ros::spin();

  return 0;
}
