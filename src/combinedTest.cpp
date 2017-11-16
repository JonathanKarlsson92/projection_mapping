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
#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include <tf/transform_listener.h>
//#include <iiwa_msgs/JointTorque.h>


// main class where public and private variables and parameters are defined
class transform{
	public:
		transform(ros::NodeHandle&); //  n
		void helloWorld(int count);
	private:
		double sampleMethod(); //declaration of a private example method
		ros::Publisher chatter_pub;
		ros::Subscriber sub;
};


//Add subscribers and publishers
transform::transform(ros::NodeHandle& n){
//Not used yet, move from main t0 here
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
void helloWorld(int count,ros::Publisher chatter_pub)
{
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());
	chatter_pub.publish(msg);

	
}

//Main method
int main(int argc, char **argv)
{
	ros::init(argc, argv, "transformation");
	//node handle
	ros::NodeHandle n;

	//what to publish
 	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	//what to subscribe	
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  	ros::Rate loop_rate(1);

  	int count = 0;
	while (ros::ok())
	{	
		//helloWorld
		++count;
		helloWorld(count,chatter_pub);
		

		ros::spinOnce();
	    	loop_rate.sleep();
	    	

		//transform t = transform(n);
		//t.run(n);
	}
	return 0;
}
