#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Twist.h> 
#include <math.h> 
#include <list>
//Custom message
#include "projection_mapping/Point2D.h"
#include "projection_mapping/Transformed_marker.h"
#include "projection_mapping/Ar_projection.h"
//End custom message

// broadcaster
#include <tf/transform_broadcaster.h>

// parameters that have to be set for the projector that is used
#define view_angle_x 3.14/4
#define view_angle_y 3.14/6
//#define resolution_x 1920 //Moved to html file
//#define resolution_y 1080 //Moved to html file
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

// Debug variables


//Global variable declaration (Not optimal!)
//visible markers
std::list<int> visible_markers;  //visible markers(id)
bool visible_projector=false;	//keeps track of when projector is visible

class visibleMarker{
	public:
		visibleMarker(ros::NodeHandle&); //  n
		int markerID;
	private:
		double sampleMethod(); //declaration of a private example method
		float ar_pos_x,ar_pos_y,ar_pos_z;
			
};



// callback method for what to do with every message recieved
void ar_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr&msg)
{
        try
	{
		visible_markers.clear();
		visible_projector=false;
		if(!msg->markers.empty()){
			//ROS_INFO("[%i]", msg->markers.size());
			std::cout <<"Number of cubes: "<< msg->markers.size()<<'\n';
			for(int i=0; i<	msg->markers.size(); i++){
				if(msg->markers[i].id!=0){
					visible_markers.push_back(msg->markers[i].id);
				}else{
					visible_projector=true;
				}
			}
		}
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("------ERROR_CALLBACK-------------");
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

//mapping from 3D to 2D
tf::Vector3 from3dTo2d(tf::Vector3 corner)
{
	tf::Vector3 Point2d;

	//offset in calibration
	float offset_x=0;
	float offset_y=0;
	//position in x
	float proj_const_x=1/std::tan(view_angle_x);		
	Point2d.setX(proj_const_x*(corner.x()+offset_x)/corner.z());

	//position in y
	float proj_const_y=1/std::tan(view_angle_y);		
	Point2d.setY(proj_const_y*(corner.y()+offset_y)/corner.z());

	Point2d.setZ(0);
	return Point2d; //currently 3d but nothing stored in z
}

//get transform name from id
std::string marker_string(int id){
	return "/ar_marker_"+std::to_string(id);		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapping_node");
	ros::NodeHandle n;
	//publisher
	ros::Publisher ar_publisher=n.advertise<projection_mapping::Ar_projection>("ar_projection",1000); 
	projection_mapping::Transformed_marker tr_marker;
	projection_mapping::Ar_projection msg;

	ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, ar_callback);
	// Adding projector to tree
	tf::TransformBroadcaster br;
  	tf::Transform transform;

	//pose relative to world
	tf::StampedTransform projector_to_ar_marker; 

	//Listener projector
	tf::TransformListener listener;
	
	//Position and orientation
	tf::Vector3 marker_pos_proj;
	tf::Quaternion marker_orient_proj;

	//corner points 3D
	tf::Vector3 pTopLeft;
	tf::Vector3 pTopRight;
	tf::Vector3 pBottomRight;
	tf::Vector3 pBottomLeft;

	//corner points in 2D
	tf::Vector3 marker_2D_p1;
	tf::Vector3 marker_2D_p2;
	tf::Vector3 marker_2D_p3;
	tf::Vector3 marker_2D_p4;

	//Loop rate
	ros::Rate loop_rate(10);

	const float AR_WIDTH =0.055; //size of ar-cube

	//Add projector lens relative to camera in tf
	transform.setOrigin( tf::Vector3(0.055, -0.04, -0.11) ); //Translation to projector lens	
	transform.setRotation(tf::createQuaternionFromRPY(-3.1415/2,0,3.1415));	//Rotation of coordinate system
	while (ros::ok())
  	{
		try{
			//set ros time
			ros::Time now = ros::Time(0);

			//check from projector
			if(visible_projector==true){
				//broadcast lens position to tf
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "projector_lens"));	//has to be measured
				//iterate over all visible markers(i.e id)
				for(int i:visible_markers){
					//look in tf
					listener.waitForTransform( "/projector_lens",marker_string(i), now, ros::Duration(1));
					listener.lookupTransform( "/projector_lens",marker_string(i),  now, projector_to_ar_marker);

					//get position and orientation
					marker_pos_proj=projector_to_ar_marker.getOrigin(); //position of relative to projector
					marker_orient_proj=projector_to_ar_marker.getRotation();

					//get corners
					pTopLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3(AR_WIDTH,AR_WIDTH,0));		
					pTopRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,AR_WIDTH,0));
					pBottomRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,-AR_WIDTH,0));
					pBottomLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (AR_WIDTH,-AR_WIDTH,0));

					//transform to 2D
					marker_2D_p1 = from3dTo2d(pTopLeft); 
					marker_2D_p2 = from3dTo2d(pTopRight);
					marker_2D_p3 = from3dTo2d(pBottomRight);
					marker_2D_p4 = from3dTo2d(pBottomLeft);

					//printing 2 corner points
					ROS_INFO("--------------------------------------");
					/*ROS_INFO("ptopleftx: [%f]", from3dTo2d(pTopLeft).x());
					ROS_INFO("ptoplefty: [%f]", from3dTo2d(pTopLeft).y());
					ROS_INFO("ptopleftz: [%f]", from3dTo2d(pTopLeft).z());

					ROS_INFO("pbottomrightx: [%f]", from3dTo2d(pBottomRight).x());
					ROS_INFO("pbottomrighty: [%f]", from3dTo2d(pBottomRight).y());
					ROS_INFO("pbottomrightz: [%f]", from3dTo2d(pBottomRight).z());*/

					//stuff for the publisher
					tr_marker.id=i;
					tr_marker.p1.X=marker_2D_p1.x();
					tr_marker.p1.Y=marker_2D_p1.y();

					tr_marker.p2.X=marker_2D_p2.x();
					tr_marker.p2.Y=marker_2D_p2.y();

					tr_marker.p3.X=marker_2D_p3.x();
					tr_marker.p3.Y=marker_2D_p3.y();

					tr_marker.p4.X=marker_2D_p4.x();
					tr_marker.p4.Y=marker_2D_p4.y();
				
					//publish corner and id
					msg.markers.push_back(tr_marker);
					
				}
				//publish list of all seen markers
				ar_publisher.publish(msg);
				//clear list for next iteration			
				msg.markers.clear();
			}
			else{ //check from camera
				std::cout <<"Error, projector not seen by the camera, make sure that nothing is in the way or change the lightning conditions"<<'\n';
				//iterate over all visible markers(i.e id)
				for(int i:visible_markers){
					//look in tf
					listener.waitForTransform( "/camera_depth_optical_frame",marker_string(i), now, ros::Duration(1));
					listener.lookupTransform( "/camera_depth_optical_frame",marker_string(i),  now, projector_to_ar_marker);

					//get position and orientation
					marker_pos_proj=projector_to_ar_marker.getOrigin(); //position of relative to projector
					marker_orient_proj=projector_to_ar_marker.getRotation();

					//get corners
					pTopLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3(AR_WIDTH,AR_WIDTH,0));		
					pTopRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,AR_WIDTH,0));
					pBottomRight=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (-AR_WIDTH,-AR_WIDTH,0));
					pBottomLeft=marker_pos_proj+tf::quatRotate(marker_orient_proj, tf::Vector3 (AR_WIDTH,-AR_WIDTH,0));

					//transform to 2D
					marker_2D_p1 = from3dTo2d(pTopLeft); 
					marker_2D_p2 = from3dTo2d(pTopRight);
					marker_2D_p3 = from3dTo2d(pBottomRight);
					marker_2D_p4 = from3dTo2d(pBottomLeft);

					//printing 2 corner points
					ROS_INFO("--------------------------------------");
					/*ROS_INFO("ptopleftx: [%f]", from3dTo2d(pTopLeft).x());
					ROS_INFO("ptoplefty: [%f]", from3dTo2d(pTopLeft).y());
					ROS_INFO("ptopleftz: [%f]", from3dTo2d(pTopLeft).z());

					ROS_INFO("pbottomrightx: [%f]", from3dTo2d(pBottomRight).x());
					ROS_INFO("pbottomrighty: [%f]", from3dTo2d(pBottomRight).y());
					ROS_INFO("pbottomrightz: [%f]", from3dTo2d(pBottomRight).z());*/

					//stuff for the publisher
					tr_marker.id=i;
					tr_marker.p1.X=marker_2D_p1.x();
					tr_marker.p1.Y=marker_2D_p1.y();

					tr_marker.p2.X=marker_2D_p2.x();
					tr_marker.p2.Y=marker_2D_p2.y();

					tr_marker.p3.X=marker_2D_p3.x();
					tr_marker.p3.Y=marker_2D_p3.y();

					tr_marker.p4.X=marker_2D_p4.x();
					tr_marker.p4.Y=marker_2D_p4.y();
				
					//publish corner and id
					msg.markers.push_back(tr_marker);
					
				}
				//publish list of all seen markers
				ar_publisher.publish(msg);
				//clear list for next iteration			
				msg.markers.clear();
				//END TEST
			}
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("-----------ERROR-------------");
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		ros::spinOnce(); //to get list of visible ar_markers
		loop_rate.sleep();

	}
	return 0;
}

///Todo
//Fix error loading manifest problem (Fixed, remember to use $source devel/setup.bash)

//----------------------------------------------------------------
//kill marker if not seen in a while
//Fix a transformation from projector marker to lens
// initialize pose to avoid huge values
// enable tracking of multiple cubes
// solve hard coded marker 6
