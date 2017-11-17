#include <ros/ros.h>
//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/CollisionObject.h>
//#include <control_msgs/FollowJointTrajectoryActionResult.h>
//#include <vector>
#include <std_msgs/String.h>
#include <sstream>
//#include <iiwa_msgs/JointPosition.h>
//#include <iiwa_msgs/SmartServoMode.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include <tf/transform_listener.h>
//#include <iiwa_msgs/JointTorque.h>


// main class where public and private variables and parameters are defined
class transform{
	public:
		transform(ros::NodeHandle&); //  n
	private:
		double sampleMethod(); //declaration of a private example method
	
};


//Add subscribers and publishers
transform::transform(ros::NodeHandle& n){

}

//Sample method
double transform::sampleMethod()
{
	return 0;
}

//Main method
int main(int argc, char **argv)
{
	ros::init(argc, argv, "transformation");
	ros::NodeHandle n;
 	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  	ros::Rate loop_rate(10);

  	int count = 0;
	while (ros::ok())
	{	
		//test
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);

		//endtest
		ros::spinOnce();
	    	loop_rate.sleep();
	    	++count;

		transform t = transform(n);
	}

	return 0;
}
