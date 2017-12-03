#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Twist.h> 
#include <math.h> 
//Custom message
#include "projection_mapping/Point2D.h"
#include "projection_mapping/Transformed_marker.h"
#include "projection_mapping/Ar_projection.h"
//End custom message

#define view_angle_x 3.14/2
#define view_angle_y 3.14/3
#define resolution_x 1024
#define resolution_y 720
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//Global variable declaration (Not good!)
tf::Vector3 marker_2D_p1;
ros::Publisher ar_projection_pub;

//--------------

class transform{
	public:
		transform(ros::NodeHandle&); //  n
		int markerID;
		
		
		//Parameters for projector
		//const static float view_angle_x=3.14/2;
		//const static float view_angle_y=3.14/3;
		//const static float resolution_x=1024;
		//const static float resolution_y=720;

		//parameter for artificial projection plane
		//float distance_z=1;
		
	private:
		double sampleMethod(); //declaration of a private example method
		float ar_pos_x,ar_pos_y,ar_pos_z;
		//void lookUp();
		

	
};
//Initialization for subscribers,publishers, variables and constants
transform::transform(ros::NodeHandle& n){

}
tf::Vector3 from3dTo2d(tf::Vector3 corner)
{
	tf::Vector3 Point2d;

	//position in x
	float proj_const_x=resolution_x/std::tan(view_angle_x);		
	Point2d.setX(proj_const_x*corner.x()/corner.z());

	//position in y
	float proj_const_y=resolution_y/std::tan(view_angle_y);		
	Point2d.setY(proj_const_y*corner.y()/corner.z());

	return Point2d; //currently 3d but nothing stored in z
}


// callback method for what to do with every message recieved
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&msg)
{
	ros::NodeHandle r;
	ar_projection_pub = r.advertise<geometry_msgs::Pose>("ar_projection", 1000);	//projected points

	//std::list<int>
	const float AR_WIDTH =0.055; //size of ar-cube
	int nrOfCubes=msg->markers.size(); //number of found cubes (-1?)

	// Initializing difference between camera and projector hard coded, possibly find a way to do this dynamically
	// assuming camera is on projector
	//tf::Vector3 camera_to_projector_vector(0,0,0.2); //estimated difference in height
	//tf::Quaternion camera_to_projector_quaternion(tf::Quaternion(0, 0, 0)); //Same orientation x y z?
	
	// Begin Listener
	//tf::Vector3 marker_pos_world;
	//tf::Quaternion marker_orient_world;
	
	//Listener projector
	tf::Vector3 marker_pos_proj;
	tf::Quaternion marker_orient_proj;

	tf::TransformListener listener;
	
	tf::StampedTransform world_to_ar_marker_6; //pose relative to world
	tf::StampedTransform projector_to_ar_marker_6; //pose relative to world

	try
	{
		ros::Time now = ros::Time::now(); //What??
		//listener.waitForTransform("/world", "/ar_marker_6", now, ros::Duration(3.0));
		//listener.lookupTransform("/world", "/ar_marker_6",  ros::Time(0), world_to_ar_marker_6);

		//Assigning marker id8 as projector
		listener.waitForTransform("/ar_marker_8", "/ar_marker_6", now, ros::Duration(3.0));
		listener.lookupTransform("/ar_marker_8", "/ar_marker_6",  ros::Time(0), projector_to_ar_marker_6);

		//marker_pos_world=world_to_ar_marker_6.getOrigin(); //x position of relative to world frame
		//marker_orient_world=world_to_ar_marker_6.getRotation();

		marker_pos_proj=projector_to_ar_marker_6.getOrigin(); //position of relative to projector
		marker_orient_proj=projector_to_ar_marker_6.getRotation();
		
		//ROS_INFO("position_x in world frame: [%f]", marker_pos_world.x());	//Why does this not work??
		//ROS_INFO("position_y in world frame: [%f]", marker_pos_world.getY());
		//ROS_INFO("position_z in world frame: [%f]", marker_pos_world.getZ());
		//ROS_INFO("orientation_x in world frame: [%f]", marker_orient_world.x());

		//ROS_INFO("position_x in world frame: [%f]", marker_pos_proj.x());	//Why does this not work??
		//ROS_INFO("position_y in world frame: [%f]", marker_pos_proj.y());
		//ROS_INFO("position_z in world frame: [%f]", marker_pos_proj.z());

		if(!msg->markers.empty()){
		//for(int i=0; i<nrOfCubes;i++) //repeat the same number as cubes
		for(int i=0; i<nrOfCubes;i++) //repeat the same number as cubes
		{
			int markerID=msg->markers[i].id; // id of detected marker i
			if(markerID!=8) //if ar-code is not the projector
			{
				//get the position and orientation of cube nr [i]
				tf::Vector3 position(msg->markers[i].pose.pose.position.x, msg->markers[i].pose.pose.position.y, msg->markers[i].pose.pose.position.z);
				tf::Quaternion quaternion(msg->markers[i].pose.pose.orientation.x,
				msg->markers[i].pose.pose.orientation.y,
				msg->markers[i].pose.pose.orientation.z,
				msg->markers[i].pose.pose.orientation.w);
		
				//Creating the 4 corners of a box
				//quatRotate rotates the vector according to the quaternion


				// THIS HAS TO BE VERIFIED
				tf::Vector3 pTopLeft=marker_pos_proj+tf::quatRotate(quaternion, tf::Vector3(AR_WIDTH,AR_WIDTH,0));
				tf::Vector3 pTopRight=marker_pos_proj+tf::quatRotate(quaternion, tf::Vector3 (-AR_WIDTH,AR_WIDTH,0));
				tf::Vector3 pBottomLeft=marker_pos_proj+tf::quatRotate(quaternion, tf::Vector3 (-AR_WIDTH,-AR_WIDTH,0));
				tf::Vector3 pBottomRight=marker_pos_proj+tf::quatRotate(quaternion, tf::Vector3 (AR_WIDTH,-AR_WIDTH,0));


				//ROS_INFO("---------Projector-----------");
				//ROS_INFO("position_x: [%f]", position.x()); //Dist from camera
				//ROS_INFO("position_y: [%f]", position.y()); //height
				//ROS_INFO("position_z: [%f]", position.z()); //side wise

				/*ROS_INFO("orientation_x: [%f]", quaternion.x());
				ROS_INFO("orientation_y: [%f]", quaternion.x());
				ROS_INFO("orientation_z: [%f]", quaternion.x());
				ROS_INFO("orientation_z: [%f]", quaternion.x());*/

				//topleftpoint
				//ROS_INFO("--------");
				//ROS_INFO("ptopleftx: [%f]", pTopLeft.x());
				//ROS_INFO("ptoplefty: [%f]", pTopLeft.y());
				//ROS_INFO("ptopleftz: [%f]", pTopLeft.z());

				ROS_INFO("markerID: [%i]", msg->markers[i].id);
				//Convert a point to 2D(for projector) test


				marker_2D_p1 = from3dTo2d(marker_pos_proj);


				//marker_2D_p1.setX(pTopLeft.getX());
				//marker_2D_p1.setY(pTopLeft.getY());
				//marker_2D_p1.setZ(0);
				//marker_2D_p1.setZ(pTopLeft.getZ());
				//ROS_INFO("mapped point x: [%f]", marker_2D_p1.getX());


				
				geometry_msgs::Pose ar_msg;
				ar_msg.position.x = marker_2D_p1.getX();
				ar_msg.position.y = marker_2D_p1.getY();
				ar_msg.position.z = 0;

				ar_msg.orientation.x = 0;
				ar_msg.orientation.y = 0;
				ar_msg.orientation.z = 0;
				ar_msg.orientation.w = 0;


				ar_projection_pub.publish(ar_msg);
				//ROS_INFO("MAPPED POINT X: [%f]", ar_msg_x.data.c_str());

				/*std_msgs::String ar_msg_y;
				ar_msg_y.data = "hello";
				ar_projection_pub_y.publish(ar_msg_y);
				ROS_INFO("MAPPED POINT Y: [%s]", ar_msg_y.data.c_str());*/
			}
		}
		//ROS_INFO("Number of cubes: [%i]", nrOfCubes);
		//ROS_INFO("--------------------------------");
	}
		
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}




	// end listener
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping_node");
	ros::NodeHandle n;
	//ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1000, poseCallback); //subscribing to position and orientation from Alvar
	//ros::Publisher testing_points = n.advertise<geometry_msgs::Pose>("ar_projection", 1);	//test
	ros::Publisher ar_publisher=n.advertise<projection_mapping::Transformed_marker>("ar_projection",100); // test of message
       
	//poseCallback may only be called 1 time since 
	//possible fix
	//poseCallback(ar_pose)
	

	ros::Rate loop_rate(25);
	int v_x=0;
	int v_y=0;
	while (ros::ok())
  	{
 		ros::spinOnce();
		loop_rate.sleep();

		//test(without camera)
		//geometry_msgs::Pose ar_points;
		//ar_points.position.x = test_var_x;
		//ar_points.position.y = test_var_y;
		/*ar_points.position.z = 0;

		ar_points.orientation.x = 0;
		ar_points.orientation.y = 0;
		ar_points.orientation.z = 0;
		ar_points.orientation.w = 0;
		testing_points.publish(ar_points);*/
		v_x=v_x+5;
		v_y=v_y+5;
		if(v_x>200){
			v_x=0;
			v_y=0;
		}
		//end test

		//begin custom message test

		//projection_mapping::Ar_projection points;
		projection_mapping::Transformed_marker marker;
		marker.id=1;
		marker.p1.X=1+v_x;
		marker.p1.Y=10+v_y;
		
		marker.p2.X=40+v_x;
		marker.p2.Y=10+v_y;

		marker.p3.X=40+v_x;
		marker.p3.Y=100+v_y;
		
		marker.p4.X=1+v_x;
		marker.p4.Y=100+v_y;
		/*points.markers[0].id=4;

		points.markers[0].p1.X=1;
		points.markers[0].p1.Y=1;

		points.markers[0].p2.X=10;
		points.markers[0].p2.Y=10;

		points.markers[0].p3.X=10;
		points.markers[0].p3.Y=40;

		points.markers[0].p4.X=50;
		points.markers[0].p4.Y=20;*/


		
		//ROS_INFO("MAPPED POINT p1X: [%i]", points.markers[0].p1.X);
		ar_publisher.publish(marker);
		//ar_publisher.publish(points);
		
		//end custom message test
	
	}
	

	return 0;
}

///Todo
//Fix error loading manifest problem
// initialize pose to avoid huge values
// enable tracking of multiple cubes
// solve hard coded marker 6
