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

// broadcaster
#include <tf/transform_broadcaster.h>

// parameters that have to be set for the projector that is used
#define view_angle_x 3.14/6
#define view_angle_y 3.14/8
#define resolution_x 1920
#define resolution_y 1080
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//Global variable declaration (Not optimal!)
tf::Vector3 marker_2D_p1;
tf::Vector3 marker_2D_p2;
tf::Vector3 marker_2D_p3;
tf::Vector3 marker_2D_p4;
//ros::Publisher ar_projection_pub;
ros::Publisher ar_publisher;
//--------------

class transform{
	public:
		transform(ros::NodeHandle&); //  n
		int markerID;
		

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

	//offset in calibration
	float offset_x=0.08;
	float offset_y=0.08;
	//position in x
	float proj_const_x=resolution_x/std::tan(view_angle_x);		
	Point2d.setX(proj_const_x*(corner.x()+offset_x)/corner.z());

	//position in y
	float proj_const_y=resolution_y/std::tan(view_angle_y);		
	Point2d.setY(proj_const_y*(corner.y()+offset_y)/corner.z());

	Point2d.setZ(0);
	return Point2d; //currently 3d but nothing stored in z
}


// callback method for what to do with every message recieved
void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&msg)
{
	ros::NodeHandle r;
	ar_publisher=r.advertise<projection_mapping::Transformed_marker>("ar_projection",10); // publishing message
	//ar_projection_pub = r.advertise<geometry_msgs::Pose>("ar_projection", 1000);	//projected points
	//ros::Publisher ar_publisher=r.advertise<projection_mapping::Transformed_marker>("ar_projection",100); // test of message
       
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
		//listener.waitForTransform("/ar_marker_4", "/ar_marker_2", now, ros::Duration(3.0));
		//listener.lookupTransform("/ar_marker_4", "/ar_marker_2",  ros::Time(0), projector_to_ar_marker_6);
		listener.waitForTransform( "/camera_depth_optical_frame","/ar_marker_4", now, ros::Duration(0.5));			//Add "if marker x is seen"
		listener.lookupTransform( "/camera_depth_optical_frame","/ar_marker_4",  ros::Time(0), projector_to_ar_marker_6);

		//marker_pos_world=world_to_ar_marker_6.getOrigin(); //x position of relative to world frame
		//marker_orient_world=world_to_ar_marker_6.getRotation();

		marker_pos_proj=projector_to_ar_marker_6.getOrigin(); //position of relative to projector
		marker_orient_proj=projector_to_ar_marker_6.getRotation();
		

		//ROS_INFO("position_x in world frame: [%f]", marker_pos_proj.x());	//Why does this not work??
		//ROS_INFO("position_y in world frame: [%f]", marker_pos_proj.y());
		//ROS_INFO("position_z in world frame: [%f]", marker_pos_proj.z());

		if(!msg->markers.empty()){
		//for(int i=0; i<nrOfCubes;i++) //repeat the same number as cubes
		for(int i=0; i<nrOfCubes;i++) //repeat the same number as cubes
		{
			int markerID=msg->markers[i].id; // id of detected marker i
			if(markerID==4) //if ar-code is not the projector
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
				tf::Vector3 pTopLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3(AR_WIDTH,AR_WIDTH,0));		//Is the error here?
				tf::Vector3 pTopRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,AR_WIDTH,0));
				tf::Vector3 pBottomRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,-AR_WIDTH,0));
				tf::Vector3 pBottomLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (AR_WIDTH,-AR_WIDTH,0));

				//tf::Vector3 pTopLeft=marker_pos_proj+tf::Vector3(AR_WIDTH,AR_WIDTH,0);		//Is the error here?
				//tf::Vector3 pTopRight=marker_pos_proj+tf::Vector3(-AR_WIDTH,AR_WIDTH,0);
				//tf::Vector3 pBottomRight=marker_pos_proj+tf::Vector3(-AR_WIDTH,-AR_WIDTH,0);
				//tf::Vector3 pBottomLeft=marker_pos_proj+tf::Vector3(AR_WIDTH,-AR_WIDTH,0);

				/*ROS_INFO("ptopleftx: [%f]", pTopLeft.x());
				ROS_INFO("ptoplefty: [%f]", pTopLeft.y());
				ROS_INFO("ptopleftz: [%f]", pTopLeft.z());

				ROS_INFO("pbottomrightx: [%f]", pBottomRight.x());
				ROS_INFO("pbottomrighty: [%f]", pBottomRight.y());
				ROS_INFO("pbottomrightz: [%f]", pBottomRight.z());
				*/

				//ROS_INFO("markerID: [%i]", msg->markers[i].id);


				marker_2D_p1 = from3dTo2d(pTopLeft); //Is the error here?
				marker_2D_p2 = from3dTo2d(pTopRight);
				marker_2D_p3 = from3dTo2d(pBottomRight);
				marker_2D_p4 = from3dTo2d(pBottomLeft);


				ROS_INFO("--------------------------------------");
				ROS_INFO("ptopleftx: [%f]", from3dTo2d(pTopLeft).x());
				ROS_INFO("ptoplefty: [%f]", from3dTo2d(pTopLeft).y());
				ROS_INFO("ptopleftz: [%f]", from3dTo2d(pTopLeft).z());

				ROS_INFO("pbottomrightx: [%f]", from3dTo2d(pBottomRight).x());
				ROS_INFO("pbottomrighty: [%f]", from3dTo2d(pBottomRight).y());
				ROS_INFO("pbottomrightz: [%f]", from3dTo2d(pBottomRight).z());

				//stuff for the publisher
				projection_mapping::Transformed_marker marker;
				marker.id=0;
				marker.p1.X=marker_2D_p1.x()+resolution_x/2;
				marker.p1.Y=marker_2D_p1.y()+resolution_y/2;

				marker.p2.X=marker_2D_p2.x()+resolution_x/2;
				marker.p2.Y=marker_2D_p2.y()+resolution_y/2;

				marker.p3.X=marker_2D_p3.x()+resolution_x/2;
				marker.p3.Y=marker_2D_p3.y()+resolution_y/2;

				marker.p4.X=marker_2D_p4.x()+resolution_x/2;
				marker.p4.Y=marker_2D_p4.y()+resolution_y/2;
				
				ar_publisher.publish(marker);

			}

		}
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
       	ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 10, poseCallback); //subscribing to position and orientation from Alvar
	
	// Adding projector to tree
	tf::TransformBroadcaster br;
  	tf::Transform transform;

	ros::Rate loop_rate(20);
	while (ros::ok())
  	{
 		ros::spinOnce();

		//ROS_INFO("MAPPED POINT p1X: [%f]", marker_2D_p1.x());
		//ROS_INFO("MAPPED POINT p1Y: [%f]", marker_2D_p1.y());
		//ROS_INFO("MAPPED POINT p1Z: [%f]", marker_2D_p1.z());

		loop_rate.sleep();

	}
	return 0;
}

///Todo
//Fix error loading manifest problem (Fixed, remember to use $source devel/setup.bash)

//----------------------------------------------------------------
//Fix a transformation from projector marker to lens
// initialize pose to avoid huge values
// enable tracking of multiple cubes
// solve hard coded marker 6
