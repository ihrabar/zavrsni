#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <agv_control/Path.h>
#include <std_msgs/String.h>
#include <sstream>

#define Td 0.05
using namespace std;

vector<string> vehNames;
vector<tf::Transform> base_link_transform;
vector<geometry_msgs::PoseStamped> vehPose;

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
// ******************************************************************

int main(int argc, char **argv)
{
	ros::init(argc, argv, "p3dxSim");
	ros::NodeHandle n;

  	vector<ros::Subscriber> subInitialPose;
  	vector<ros::Subscriber> subCmdVel;
  	  	
  	vector<ros::Publisher> pubPose;

  	XmlRpc::XmlRpcValue vehicleNames;

  	// Getting vehicle names from the ROS Parameter Server
  	n.getParam("/vehicleNames", vehicleNames);
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
		
		// Subscribers for receiving individual vehicle's "cmd_vel"
    	topicName = "/" + vehicleName + "/cmd_vel";
    	subCmdVel.push_back(n.subscribe<geometry_msgs::Twist>(topicName, 1, boost::bind(cmdVelReceivedCallback, _1, topicName)));
    	
    	// Publishers for publishing simulated vehicle poses (used only for displaying vehicles within RViz by arrows)
    	topicName = "/" + vehicleName + "/simPose";
    	pubPose.push_back(n.advertise<geometry_msgs::PoseStamped>(topicName, 1));
  	}

	tf::Vector3 vect = tf::Vector3(0.0, 0.0, 0.0);
	tf::Quaternion quat = tf::Quaternion(0, 0, 0);
	
	tf::Transform transform;
	transform.setOrigin(vect);
	transform.setRotation(quat);

	for(unsigned int i = 0; i < vehNames.size(); i++)
	{	
		geometry_msgs::PoseStamped pose;
		vehPose.push_back(pose);
		vehPose[i].header.frame_id = string("/"+vehNames[i]+"/map");
		
		base_link_transform.push_back(transform);
	}
	
	static tf::TransformBroadcaster br;
	ros::Rate loop_rate(20);

	while (ros::ok())
  	{
  		for(unsigned int i = 0; i < vehNames.size(); i++)
		{
			// transform broadcasting
  			br.sendTransform(tf::StampedTransform(base_link_transform[i], ros::Time::now(), string("/"+vehNames[i]+"/map"), string("/"+vehNames[i]+"/base_link")));
  			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", string("/"+vehNames[i]+"/map")));

			// publishing simulated vehicle poses (used only for the purpose of representing vehicles within RViz by arrows)
  			tf::pointTFToMsg(base_link_transform[i].getOrigin(), vehPose[i].pose.position);
			tf::quaternionTFToMsg(base_link_transform[i].getRotation(), vehPose[i].pose.orientation);
			vehPose[i].header.stamp = ros::Time::now();
  			pubPose[i].publish(vehPose[i]);
  		}

	  	ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
