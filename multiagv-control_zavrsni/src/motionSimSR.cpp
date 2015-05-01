#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <agv_control/Path.h>
#include <std_msgs/String.h>
#include <sstream>
#include <skilled_lgv_daci/MotorData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#define Td 0.05
using namespace std;

vector<string> vehNames;
vector<tf::Transform> base_link_transform;
vector<geometry_msgs::PoseStamped> vehPose;

double* myMap = NULL;
int mapWidth = 0;
int mapHeight = 0;
float mapResolution = 0;
vector<ros::Publisher> pubOdometryData;
double p1=0,p2=0;

void refOdometryDataReceivedCallback(const skilled_lgv_daci::MotorData::ConstPtr& msg, const std::string &topic)
{
	skilled_lgv_daci::MotorData md;
	static ros::Time last_time=ros::Time::now();
	int positionscaling=3450;//4520;//*1.1;
	int velocityscaling=1842;//1450*1.05;
	double a=1.1;
	double v1,v2,vx,vy,vth;
	ros::Time current_time=ros::Time::now();
	double dt=(current_time-last_time).toSec();
	last_time=current_time;
	if (dt<1 && dt>0)
	{ 
		v1=(double) msg->vel1/velocityscaling;
		v2=msg->vel2;
		p2=p2+v2*dt/positionscaling;
		p1=p1+v1*dt;
		double angle=p2;//+20./4500;
		vx=cos(angle)*v1;
		vy=0;
		vth=v1/(a/sin(angle));	

	}
	else return;
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(topic.find(vehNames[i]) != string::npos)
		{
			tf::Quaternion q1;
			tf::quaternionMsgToTF(vehPose[i].pose.orientation, q1);
			double yaw = tf::getYaw(q1);

			double delta_yaw = vth *dt;// Td;
			double new_yaw = yaw + delta_yaw;
			tf::Quaternion new_quat = tf::createQuaternionFromYaw( new_yaw );
			vehPose[i].pose.orientation = tf::createQuaternionMsgFromYaw( new_yaw );
			ROS_INFO("vx vth dt %.2f %.2f %.2f",vx,vth,Td);
			
			double delta_x =  (vx * dt/*Td*/) * cos(new_yaw);
			double delta_y =  (vx * dt/*Td*/) * sin(new_yaw);

			vehPose[i].pose.position.x += delta_x;
			vehPose[i].pose.position.y += delta_y;
			vehPose[i].pose.position.z = 0;

			base_link_transform[i].setOrigin( tf::Vector3(vehPose[i].pose.position.x,  vehPose[i].pose.position.y, vehPose[i].pose.position.z) );
			base_link_transform[i].setRotation( new_quat );

			md.vel1=v1*velocityscaling;
			md.vel2=v2;
			md.pos1=p1*velocityscaling;
			md.pos2=p2*positionscaling;
			pubOdometryData[i].publish(md);

		}
	}
		ros::spinOnce();	
}

/** Callback invoked on map received event */
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapWidth =  msg->info.width;
	mapHeight = msg->info.height;

  	mapResolution = msg->info.resolution; // [m/cell]

  	myMap = (double*)malloc(mapWidth * mapHeight * sizeof(double));

  	for (int i = 0; i < mapWidth * mapHeight; i++)
  	{
	//	if (msg->data[i]==0 || msg->data[i]==100)
//			printf("%d %d\n",msg->data[i],i);
		if (msg->data[i] != 0) 	//(msg->data[i] == 100 || msg->data[i] == -1)  -> occupied or unknown cell in OccupancyGrid message
			myMap[i] = -1; 		// occupied cell in pathPlanner's map representation
		else					// (msg->data[i] == 0) -> unoccupied cell in OccupancyGrid message
			myMap[i] = 1;		// unoccupied cell in pathPlanner's map representation
//		if (myMap[i]==1) printf("%d ",i);
  	}

  ROS_INFO("simulator received a %d X %d map @ %.3f m/pix", mapWidth, mapHeight, mapResolution);
}

// ******************* INITIAL POSE CALLBACK ************************
void initialPoseReceivedCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(msg->header.frame_id.find(vehNames[i]) != string::npos)
		{
			base_link_transform[i].setOrigin( tf::Vector3(msg->pose.pose.position.x,  msg->pose.pose.position.y, msg->pose.pose.position.z) );
			
			tf::Quaternion q1;
			tf::quaternionMsgToTF(msg->pose.pose.orientation, q1);
			base_link_transform[i].setRotation( q1 );
		}
	}
}
// ******************************************************************

// ********************** CMD_VEL CALLBACK **************************
void cmdVelReceivedCallback(const geometry_msgs::Twist::ConstPtr &msg, const std::string &topic)
{
	return;
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(topic.find(vehNames[i]) != string::npos)
		{
			tf::Quaternion q1;
			tf::quaternionMsgToTF(vehPose[i].pose.orientation, q1);
			double yaw = tf::getYaw(q1);

			double delta_yaw = msg->angular.z * Td;
			double new_yaw = yaw + delta_yaw;
			tf::Quaternion new_quat = tf::createQuaternionFromYaw( new_yaw );
			vehPose[i].pose.orientation = tf::createQuaternionMsgFromYaw( new_yaw );
			
			
			double delta_x =  (msg->linear.x * Td) * cos(new_yaw);
			double delta_y =  (msg->linear.x * Td) * sin(new_yaw);

			vehPose[i].pose.position.x += delta_x;
			vehPose[i].pose.position.y += delta_y;
			vehPose[i].pose.position.z = 0;

			base_link_transform[i].setOrigin( tf::Vector3(vehPose[i].pose.position.x,  vehPose[i].pose.position.y, vehPose[i].pose.position.z) );
			base_link_transform[i].setRotation( new_quat );
		}
	}
}
// ****************************************************************
void findScans(double x,double y,double yaw,double min,double max,double increment,double *scan)
{
	double xn,yn,phi;
	int cc=0;
	double dist;
		xn=x/mapResolution;
		yn=y/mapResolution;
//		printf("boja=%d\n",myMap[(int) yn*mapWidth+(int) xn]);
	for (double i=min;i<max;i+=increment)
	{
		phi=yaw+i;//180*3.14159;
		dist=1;
		xn=x/mapResolution;
		yn=y/mapResolution;
		while (1) 
		{
			xn=x/mapResolution+dist*cos(phi);
			yn=y/mapResolution+dist*sin(phi);
			
//			printf("%.4f %.4f %d %d %.2f\n",xn,yn,mapWidth,mapHeight,mapResolution);
			if (xn<0 || xn>mapWidth || yn<0 || yn>mapHeight)
			{
				scan[cc]=100;//dist*mapResolution;
				break;
			}
			if (myMap[(int) yn*mapWidth+(int) xn]==-1)
			{
				scan[cc]=dist*mapResolution;
				break;
			}
			dist+=1;
		}
		cc++;
	}
}
void publish_scan(ros::Publisher *pub,float angle_min,float angle_max,int scan_count,geometry_msgs::PoseStamped pose,tf::StampedTransform lasertf,std::string frame_id)
{
	if (myMap==NULL) {
		ROS_ERROR("Sim No map");
		return;
	}
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  double *ranges=new double[scan_count];
  bool inverted=false;
  if (inverted) { // assumes scan window at the bottom
    scan_msg.angle_min = angle_max;
    scan_msg.angle_max = angle_min;
  } else {
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
  }
  double x,y,phi,dx,dy,dphi;
  dx=lasertf.getOrigin().x();
  dy=lasertf.getOrigin().y();
  dphi=tf::getYaw(lasertf.getRotation());
ROS_INFO("laser tf%.2f %.2f %.2f",dx,dy,dphi);
  x=pose.pose.position.x;
  y=pose.pose.position.y;
  tf::Quaternion q1;
  tf::quaternionMsgToTF(pose.pose.orientation, q1);
  double yaw = tf::getYaw(q1)+dphi;
  x=x+dx*cos(yaw)+dy*sin(yaw);
  y=y+dy*cos(yaw)+dx*sin(yaw);
  
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(scan_count-1);
 printf("%.2f %.2f %.2f\n",x,y,yaw);
  findScans(x,y,yaw,angle_min,angle_max,scan_msg.angle_increment,ranges);
  
	ros::Time start= ros::Time::now();
  scan_msg.scan_time = 0.1;//scan_time;
  scan_msg.time_increment = scan_msg.scan_time/*scan_time*/ / scan_count;
  scan_msg.range_min = 0.1;
  scan_msg.range_max = 250.;
  scan_msg.ranges.resize(scan_count);
  scan_msg.header.stamp = start;
  for (size_t i = 0; i < scan_count; i++) {
    scan_msg.ranges[i] = ranges[i];
  }
  scan_msg.intensities.resize(scan_count);
  for (size_t i = 0; i < scan_count; i++) {
    scan_msg.intensities[i] = 0;//(float)intensity_values[i];
  }
  pub->publish(scan_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "p3dxSim");
	ros::NodeHandle n;
    ros::NodeHandle np("~");
	ROS_INFO("Sim start");
  	vector<ros::Subscriber> subInitialPose;
  	vector<ros::Subscriber> subCmdVel;

  	vector<ros::Subscriber> subOdometryRef;

  	vector<ros::Publisher> pubPose;
   	std::string scan, map_frame, base_link_frame, base_link_true, laser_frame;
	vector<ros::Publisher> scan_pub ;
  	XmlRpc::XmlRpcValue vehicleNames;

// Subscriber za primanje karte prostora:
  	ros::Subscriber subMap = n.subscribe("map", 1, mapReceivedCallback);


  	// Getting vehicle names from the ROS Parameter Server
  	np.getParam("/vehicleNames", vehicleNames);
    np.param("map_frame", map_frame,(std::string)"map");
    np.param("base_link_frame", base_link_frame,(std::string)"base_link");
    np.param("base_link_true", base_link_true,(std::string)"base_link_true"); // Ground truth
    np.param("laser_frame", laser_frame,(std::string)"laser");
  	ROS_ASSERT(vehicleNames.getType() == XmlRpc::XmlRpcValue::TypeArray);

  	// Petlja po imenima svih vozila:
  	for (int32_t i = 0; i < vehicleNames.size(); ++i)
  	{
    		ROS_ASSERT(vehicleNames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    		string vehicleName = static_cast<string>(vehicleNames[i]);
    	
    		vehNames.push_back(vehicleName);
    	
    		// Subscribers for receiving individual vehicle's initial pose
	    	string topicName = "/" + vehicleName + "/initialpose";
	    	subInitialPose.push_back(n.subscribe(topicName, 1, initialPoseReceivedCallback));
		
		//Subscribers for receiving vehicle odometry reference

    		topicName = "/" + vehicleName + "/refodometrydata";
	    	subOdometryRef.push_back(n.subscribe<skilled_lgv_daci::MotorData>	(topicName, 1, boost::bind(refOdometryDataReceivedCallback, _1, topicName)));

		/*	// Subscribers for receiving individual vehicle's "cmd_vel"
	    	topicName = "/" + vehicleName + "/cmd_vel";
	    	subCmdVel.push_back(n.subscribe<geometry_msgs::Twist>(topicName, 1, 	boost::bind(cmdVelReceivedCallback, _1, topicName)));
	    	*/
	    	// Publishers for publishing simulated vehicle poses (used only for displaying vehicles within RViz by arrows)
	    	topicName = "/" + vehicleName + "/simPose";
	    	pubPose.push_back(n.advertise<geometry_msgs::PoseStamped>(topicName, 1));
            scan="/"+vehicleName+"/scan";
            scan_pub.push_back(n.advertise<sensor_msgs::LaserScan>(scan, 1));
            pubOdometryData.push_back(n.advertise<skilled_lgv_daci::MotorData>("/"+vehicleName+"/simulated_odometry", 1));
  	}
	ROS_INFO("Sim init");

	tf::Vector3 vect = tf::Vector3(0.0, 0.0, 0.0);
	tf::Quaternion quat = tf::Quaternion(0, 0, 0);
	tf::Transform transform;
	transform.setOrigin(vect);
	transform.setRotation(quat);
	tf::Vector3 vect1 = tf::Vector3(0.4, 0.0, 0.0);
	tf::Quaternion quat1 = tf::Quaternion(0, 0, 0);
	tf::StampedTransform lasertf;


	for(unsigned int i = 0; i < vehNames.size(); i++)
	{	
		geometry_msgs::PoseStamped pose;
		vehPose.push_back(pose);
		vehPose[i].header.frame_id = string(vehNames[i]+"/" + map_frame);
		
		base_link_transform.push_back(transform);
	}
	
	static tf::TransformBroadcaster br;
	ros::Rate loop_rate(20);
	int pp=0;
	ROS_INFO("Sim loop");
	tf::TransformListener listener;
	/*
    for(unsigned int i = 0; i < vehNames.size(); i++)
	{
	
		br.sendTransform(tf::StampedTransform(base_link_transform[i], ros::Time::now(), string(vehNames[i]+"/" + base_link_frame), string(vehNames[i]+"/" + laser_frame)));
	}
    */
	while (ros::ok())
  	{
		pp++;
  		for(unsigned int i = 0; i < vehNames.size(); i++)
		{
			// transform broadcasting
  			br.sendTransform(tf::StampedTransform(base_link_transform[i], ros::Time::now(), string(vehNames[i]+"/" + map_frame), string(vehNames[i]+"/" + base_link_true)));
  			//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", string(vehNames[i]+"/map")));

  			// publishing simulated vehicle poses (used only for the purpose of representing vehicles within RViz by arrows)
  			tf::pointTFToMsg(base_link_transform[i].getOrigin(), vehPose[i].pose.position);
			tf::quaternionTFToMsg(base_link_transform[i].getRotation(), vehPose[i].pose.orientation);
			vehPose[i].header.stamp = ros::Time::now();
  			pubPose[i].publish(vehPose[i]);
  			if (pp%2==0)
  			{
				const std::string str=string(vehNames[i]+"/"+base_link_frame);
				const std::string str1=string(vehNames[i]+"/"+laser_frame);
				try
				{
                    listener.lookupTransform(str, str1, ros::Time(0), lasertf);
					publish_scan(&scan_pub[i],-3.14159,3.14159,1440,vehPose[i],lasertf,string(vehNames[i]+"/"+laser_frame));
				}
				catch (tf::TransformException &e)
				{
                    cout << e.what() << endl;
				}

			}
  		}
		//ROS_INFO("Sim spin");
	  	ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
