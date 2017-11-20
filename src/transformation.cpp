#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include <Vector3.h>

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
//std::list <Vector3> transform::createPoints(Vector3 centerPoint, Quaternion rotation)
//{
	//list=new list <Vector3>();
	//list.Add(new Vector3(1,1,1));
	//return list;
//}
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&msg)
{
	const float AR_WIDTH =0.055;
	if(!msg->markers.empty()){
		tf::Vector3 position(msg->markers[0].pose.pose.position.x, msg->markers[0].pose.pose.position.y, msg->markers[0].pose.pose.position.z);
		tf::Quaternion quaternion(msg->markers[0].pose.pose.orientation.x,
		msg->markers[0].pose.pose.orientation.y,
		msg->markers[0].pose.pose.orientation.z,
		msg->markers[0].pose.pose.orientation.w);
		
		//Creating the 4 corners of a box
		//quatRotate rotates the vector according to the quaternion
		//
		tf::Vector3 pTopLeft=position+tf::quatRotate(quaternion, tf::Vector3(AR_WIDTH,AR_WIDTH,0));	//Signs have to be checked!!
		tf::Vector3 pTopRight=position+tf::quatRotate(quaternion, tf::Vector3 (-AR_WIDTH,AR_WIDTH,0));
		tf::Vector3 pBottomLeft=position+tf::quatRotate(quaternion, tf::Vector3 (-AR_WIDTH,-AR_WIDTH,0));
		tf::Vector3 pBottomRight=position+tf::quatRotate(quaternion, tf::Vector3 (AR_WIDTH,-AR_WIDTH,0));


		ROS_INFO("--------------------------------");
		ROS_INFO("position_x: [%f]", position.x()); //Dist from camera
		ROS_INFO("position_y: [%f]", position.y()); //height
		ROS_INFO("position_z: [%f]", position.z()); //side wise

		/*ROS_INFO("orientation_x: [%f]", quaternion.x());
		ROS_INFO("orientation_y: [%f]", quaternion.x());
		ROS_INFO("orientation_z: [%f]", quaternion.x());
		ROS_INFO("orientation_z: [%f]", quaternion.x());*/

		//topleftpoint
		ROS_INFO("--------");
		ROS_INFO("ptopleftx: [%f]", pTopLeft.x());
		ROS_INFO("ptoplefty: [%f]", pTopLeft.y());
		ROS_INFO("ptopleftz: [%f]", pTopLeft.z());
	}
}
void lookUp()
{
     
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping_node");
	ros::NodeHandle n;
	ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1000, poseCallback); //subscribing to position and orientation from Alvar

	// Listener
	tf::Vector3 marker_pos_world;
	tf::Quaternion marker_orient_world;
	tf::TransformListener listener;
	lookUp();
	tf::StampedTransform world_to_ar_marker_6; //pose relative to world
	try
	{
		ros::Time now = ros::Time::now(); //What??
		listener.waitForTransform("/world", "/ar_marker_6", now, ros::Duration(3.0));
		listener.lookupTransform("/world", "/ar_marker_6",  ros::Time(0), world_to_ar_marker_6);
		marker_pos_world=world_to_ar_marker_6.getOrigin(); //x position of relative to world frame
		marker_orient_world=world_to_ar_marker_6.getRotation();

		//ROS_INFO("position_x in world frame: [%f]", marker_pos_world.x());	//Why does this not work??
		//ROS_INFO("position_y in world frame: [%f]", marker_pos_world.getY());
		//ROS_INFO("position_z in world frame: [%f]", marker_pos_world.getZ());
		//ROS_INFO("orientation_x in world frame: [%f]", marker_orient_world.x());
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	// end listener
	ros::spin();

	return 0;
}

///Todo
// initialize pose to avoid huge values
// subscribe to relative projector pose
// enable tracking of multiple cubes
// solve hard coded marker 6
