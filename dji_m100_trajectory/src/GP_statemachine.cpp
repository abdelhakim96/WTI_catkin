#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <airsim_ros_wrapper.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <ros/spinner.h>
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
//#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>
//#include <mavros_msgs/CommandTOL.h>
//#include <mavros_msgs/CommandLong.h>
//#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
//using namespace std
#include "geometry_msgs/TwistStamped.h"

ros::Publisher local_pos_pub;
ros::Publisher mesh_pos_pub;
ros::Publisher vel_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::Subscriber dronevelocity_sub;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;



//mavros_msgs::State current_state;
//mavros_msgs::State current_state_g;
geometry_msgs::PoseStamped current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;
geometry_msgs::PoseStamped point_g;
geometry_msgs::TwistStamped move;




double v_d;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 
int phase=0;

float hakim;





struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};



struct gnc_api_point{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
};


/*void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}
*/




void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_g.pose.position=msg->pose.position;
  current_pose_g.pose.orientation=msg->pose.orientation;
  //current_pose_g=*msg;
  float q0 = current_pose_g.pose.orientation.w;
  float q1 = current_pose_g.pose.orientation.x;
  float q2 = current_pose_g.pose.orientation.y;
  float q3 = current_pose_g.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  //current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}



void drone_v_cb(const std_msgs::Float64::ConstPtr& msg){
  v_d=msg->data;
}




void set_heading(float x, float y,float px,float py)
{ 
  float heading;
  //float a1=x*px+y*py;
  float a1=(px-x);
  float  a2=pow((pow((px-x),2)+pow((py-y),2)),0.5);
  if (px>x){
  heading= -acos(a1/a2);	 
  }
  
  else{
   heading=acos(a1/a2);	
  } 
  //heading=180/M_PI;
  
  
  float yaw = heading;
  ROS_INFO("set heading %f ", yaw*(180/M_PI));
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}










void set_destination(float x, float y, float z, float psi)
{

	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;
	//waypoint_g.pose.orientation.yaw = psi;
    //set_heading(psi);
	local_pos_pub.publish(waypoint_g);
}



void set_point(float x, float y, float z)
{

	//ROS_INFO("Point of view set to x: %f y: %f z: %f origin frame", x*10, y*10, z*10);
	


	point_g.pose.position.x = x;
	point_g.pose.position.y = y;
	point_g.pose.position.z = z;
	//waypoint_g.pose.orientation.yaw = psi;
    //set_heading(psi);
	mesh_pos_pub.publish(point_g);
	
}



int check_waypoint_reached(float pos_tolerance=2, float heading_tolerance=10)
{
	//local_pos_pub.publish(waypoint_g);
	
	//check for correct position 
	float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.position.y);
    float deltaZ = abs(waypoint_g.pose.position.z - current_pose_g.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );

    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    //float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );



    //if( dMag < pos_tolerance && headingErr < heading_tolerance)
	if( dMag < pos_tolerance)
	{
		return 1;
	}else{
		return 0;
	}
	ROS_INFO("Dmag: %f" , dMag);
}






int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/WP_GP").c_str(), 10);  
	mesh_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/point_to_view_traj").c_str(), 10);  
	currentPos = controlnode.subscribe<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/mocap/pose").c_str(), 10, pose_cb); 
	dronevelocity_sub = controlnode.subscribe<std_msgs::Float64>((ros_namespace + "/drone_vel").c_str(), 10, drone_v_cb); 

	return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GP_node");
	ros::NodeHandle gnc_node("~");

	init_publisher_subscriber(gnc_node);

    std::cout << "hakim";
  
    //wait4connect();
	//wait4start();mavros_msgs::State current_state;

    std::vector<gnc_api_waypoint> waypointList;
	std::vector<gnc_api_point> pointList;
	gnc_api_waypoint nextWayPoint;
	gnc_api_point pointm;
    //geometry_msgs::PointStamped pointm;            


    std::vector<double> vecX, vecY,vecZ, vec1,vec2,vec3;
    double wp_x, wp_y,wp_z,y1,y2,y3;
	double p_x,p_y,p_z;
    //std::vector<int> myVector = {1, 2, 3, 4, 5, 6};
    std::ifstream inputFile("/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/GP_output/path_2500.txt");
	//std::ifstream inputFile("/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/GP_output/interpolatedwps.txt");

    while (inputFile >> wp_x >> wp_y >> wp_z >> y1 >> y2 >> y3)
    {
    vecX.push_back(wp_x/1.0);
    vecY.push_back(wp_y/1.0);
    vecZ.push_back(wp_z/1.0);
	vec1.push_back(y1/1.0);
    vec2.push_back(y2/1.0);
    vec3.push_back(y3/1.0);
	
    }
   
    

	for(int i(0); i < vecX.size(); i++){
	nextWayPoint.x = vecX[i];
	nextWayPoint.y =  vecY[i];
	nextWayPoint.z = vecZ[i];
	//nextWayPoint.psi = vec3[i]*180/M_PI;
	waypointList.push_back(nextWayPoint);	
	}

 
   
    std::vector<double> meshX1, meshX2,meshX3, meshY1, meshY2,meshY3,meshZ1, meshZ2,meshZ3,vx,vy,vz;
    
    std::ifstream inputFilex("/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/GP_output/px_2500.txt");  //meshfile
    std::ifstream inputFiley("/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/GP_output/py_2500.txt");  //meshfile
	std::ifstream inputFilez("/home/hakim/catkin_ws/src/WTI_catkin/dji_m100_trajectory/src/GP_output/pz_2500.txt");
    while (inputFilex >> p_x )
    {
    meshX1.push_back(p_x);
    meshX2.push_back(p_y);
    meshX3.push_back(p_z);
	vx.push_back((p_x)/1.0);
	
    }


	while (inputFiley >> p_y  )
    {
    meshY1.push_back(p_x);
    meshY2.push_back(p_y);
    meshY3.push_back(p_z);
	vy.push_back((p_y)/1.0);
    }


	while (inputFilez >> p_z )
    {
    meshZ1.push_back(p_z);
    meshZ2.push_back(p_z);
    meshZ3.push_back(p_z);
	vz.push_back((p_z)/1.0);
    }
   
    

	for(int i(0); i < vx.size(); i++){
	pointm.x = vx[i];
	pointm.y =  vy[i];
	pointm.z = vz[i];
	pointList.push_back(pointm);
	}

	ros::Rate rate(5);
    
	ros::Time last_request = ros::Time::now();

	float tx=0.1;
	float ty=10;
    //set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
	//set_heading( current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, pointList[counter].x, pointList[counter].y);
    //set_point(pointList[counter].x,pointList[counter].y,pointList[counter].z);
	//set_heading( current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, tx, ty);
    //set_point(tx,ty,current_pose_g.pose.pose.position.z);
    int counter = 1;
	set_destination(waypointList[1].x,waypointList[1].y,waypointList[1].z, waypointList[1].psi);
	set_point(-80.0,0.0,current_pose_g.pose.position.z);
	//int v_d=1;
	int c;
	c=1;
	int n;
	while(ros::ok())
	{   
	    ros::spinOnce();
		rate.sleep();
        //set_destination(waypointList[2*counter].x,waypointList[2*counter].y,waypointList[2*counter].z, waypointList[2*counter].psi);
	    //ROS_INFO("current x %f", current_pose_g.pose.pose.position.x);
        //set_heading( current_pose_g.pose.position.x,current_pose_g.pose.position.y, tx, ty);
        //set_point(pointList[counter].x,pointList[counter].y,current_pose_g.pose.pose.position.z);
				
		//
        //set_point(-8.0,0.0,current_pose_g.pose.position.z);
		//{  

			//set_point(-8,0,current_pose_g.pose.position.z);
			
			n=counter;

	
            if(counter == 1 && check_waypoint_reached(1,1) != 1)
			
			 {
               set_destination(waypointList[1].x,waypointList[1].y,waypointList[1].z, waypointList[1].psi);
			   
			   //set_point(pointList[1].x,pointList[1].y,pointList[1].z);
			
			}
			
			else{
	         //ROS_INFO("reached w.p");
			
			if (counter < waypointList.size()-1)
			{   
				
                
			  	set_destination(waypointList[n].x,waypointList[n].y,waypointList[n].z, waypointList[n].psi);
                
   
				set_point(pointList[n].x,pointList[n].y,pointList[n].z);
				ROS_INFO("x %f", waypointList[n].x);
				ROS_INFO("y %f", waypointList[n].y);
				ROS_INFO("py %f", pointList[n].y);
				ROS_INFO("n %d", n);
				counter++;
	
                
				//}
			  //ROS_INFO("counter");
			 
			}

			
			//else{
                 //set_destination(waypointList[waypointList.size()-10].x,waypointList[waypointList.size()-10].y,waypointList[waypointList.size()-10].z, waypointList[waypointList.size()-10].psi);
			//}
			}
        //ROS_INFO("x %f", waypointList[counter-1].x);
		//ROS_INFO("y %f", waypointList[counter-1].y);
		//ROS_INFO("z %f", waypointList[counter-1].z);     
        //ROS_INFO("Drone velocity %f", v_d);
		//ROS_INFO("counter %d", n);
	   // }
		/*
		ROS_INFO("current x %f", current_pose_g.pose.pose.position.x);
		ROS_INFO("current y %f", current_pose_g.pose.pose.position.y);
		ROS_INFO("current z %f", current_pose_g.pose.pose.position.z);
	    ROS_INFO("yaw current %f", hakim*180.0/M_PI );
		ROS_INFO("x %f", waypointList[counter-1].x);
		ROS_INFO("y %f", waypointList[counter-1].y);
		ROS_INFO("z %f", waypointList[counter-1].z);
	    ROS_INFO("next yaw angle: %f", waypointList[counter-1].psi );
		*/
	}
	return 0;
}
