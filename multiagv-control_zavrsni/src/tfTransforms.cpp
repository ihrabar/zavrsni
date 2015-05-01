#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

using namespace std;

vector<string> vehNames;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tfTransforms");
	ros::NodeHandle n;

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
  	}
  	
	ros::Rate loop_rate(10);

	static tf::TransformBroadcaster br;
  	tf::Transform transform;
  	
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	
	while (ros::ok())
  	{
  		for(unsigned int i = 0; i < vehNames.size(); i++)
		{
			// transform broadcasting
	  		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", string("/"+vehNames[i]+"/map")));
	  	}

	  	ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
