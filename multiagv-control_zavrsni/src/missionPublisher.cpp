#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/String.h"
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iostream>

using namespace std;

double xPos, yPos;
tf::Quaternion quat;

vector<tf::Quaternion> quatVector;
vector<float> xArray;
vector<float> yArray;

vector<string> vehNames;
vector<ros::Publisher> missionPublishers;
vector<vector<geometry_msgs::PoseStamped> > vehMissions;	// each element of this vector is a pose vector which contains individual vehicle's missions 

bool repeat = false;
bool randomMission = false;

// Mission publishing
void publishMissions()
{	
	for(unsigned int i = 0; i < vehNames.size(); i++)
	{
		if(vehMissions[i].size() > 0)
		{
			vector<ros::Publisher>::iterator it;
			for (it = missionPublishers.begin(); it != missionPublishers.end(); ++it)
			{
				if(it->getTopic().find(vehNames[i]) != string::npos)
				{
					int index = 0;
					
					// If the "-rand" option is enabled, the current mission is picked randomly from the given mission sequence
					if(randomMission)
					{
						unsigned int lowest = 0; 
						unsigned int highest = vehMissions[i].size() - 1;
						unsigned int range = (highest - lowest) + 1;
	
						index = lowest + (rand() % (int)range);
					}
					
					it->publish(vehMissions[i][index]);
					
					if(repeat)
						vehMissions[i].push_back(vehMissions[i][index]);
					
					vehMissions[i].erase(vehMissions[i].begin() + index);
					
					if(vehMissions[i].size() == 0)
						cout << "All missions for vehicle " << vehNames[i] << " have been published." << endl;
						 
					break;
				}
			}
		}
	}
}

int loadMissionsFromFile(string filePath)
{	
	// *************** LOADING MISSIONS FROM FILE ****************
	ifstream missionsFile;
	missionsFile.open (filePath.c_str());
  	
  	if (missionsFile.is_open())
  	{
  		string line;
		geometry_msgs::PoseStamped targetPose;
		targetPose.pose.position.z = 0;
		
		while (getline(missionsFile, line))
		{
			if (!line.length())	// Ignore empty lines
            	continue;

        	if (line[0] == '#') // Ignore a line that starts with #
            	continue;
			
			istringstream iss(line, istringstream::in);
			
			string vehName = "";
			double qx, qy, qz, qw;
			
  			iss >> vehName >> targetPose.pose.position.x >> targetPose.pose.position.y >> qx >> qy >> qz >> qw;
  			
  			if(vehName == "") continue;
  			
  			tf::Quaternion qOrientation(qx, qy, qz, qw);
  			tf::quaternionTFToMsg(qOrientation, targetPose.pose.orientation);
  		
  			targetPose.header.frame_id = "/" + vehName + "/map";
			targetPose.header.stamp = ros::Time::now();
			
  			bool exist = false;
  			for(unsigned int i = 0; i < vehNames.size(); i++)
  			{
  				if(vehNames[i] == vehName)
  				{
  					exist = true;
  					vehMissions[i].push_back(targetPose);
  				}
  			}
  			
  			if(!exist)
  			{
  				vehNames.push_back(vehName);
  				vehMissions.push_back(vector<geometry_msgs::PoseStamped>());
  				vehMissions.back().push_back(targetPose);
  			}
		}	
		
		cout << "Vehicle missions succesfully loaded"<< endl;
    	missionsFile.close();
  	}

  	else 
  	{
  		ROS_ERROR("Error opening file \"%s\"!", filePath.c_str());
  		return 0;
  	}
  	
  	return 1;
	// ************************************************************
}

static void show_usage(std::string name)
{
    std::cerr << "\nUsage: " << name << " <option(s)> arguments\n\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-mf MISSIONS_FILE_PATH\tSpecify the missions file path\n"
              << "\t-rate RATE\t\tSpecify the mission publishing rate [Hz]\n"
              << "\t-repeat\t\t\tRepeat the given mission sequence\n"
              << "\t-rand\t\t\tPublish missions randomly from the given seq.\n"
              << std::endl;
}

int main(int argc, char **argv)
{
	if (argc < 2) 
	{
		ROS_FATAL("Missions file not specified!");
        show_usage(argv[0]);
        return 0;
    }
    
    string missionsFilePath = "";
	float lRate = 0.1;
    
	for (int i = 1; i < argc; ++i) 
	{
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) 
        {
            show_usage(argv[0]);
            return 0;
        } 
        else if (arg == "-mf") 
        {
            if (i + 1 < argc) 
            { 
            	missionsFilePath = argv[++i];
            } 
            else
            {	
            	std::cerr << "-mf option requires one argument." << std::endl;
                return 0;
            }  
        }
        else if (arg == "-rate" || arg == "--rate") 
        {
            if (i + 1 < argc) 
            { 
            	lRate = atof(argv[++i]);
            } 
            else
            {
            	std::cerr << "-rate option requires one argument." << std::endl;
                return 0;
            }  
        }
        else if (arg == "-repeat" || arg == "--repeat") 
        {
        	repeat = true;
        }
        else if (arg == "-rand" || arg == "--rand") 
        {
        	randomMission = true;
        }
	}
	
	if(missionsFilePath == "")
	{
		ROS_FATAL("Missions file not specified!");
        show_usage(argv[0]);
        return 0;
	}
		
	ros::init(argc, argv, "MissionPublisher");
	ros::NodeHandle n;	
	
	XmlRpc::XmlRpcValue vehicleNames;

	// Getting vehicle names from the ROS Parameter Server
  	n.getParam("/vehicleNames", vehicleNames);
  	ROS_ASSERT(vehicleNames.getType() == XmlRpc::XmlRpcValue::TypeArray);
  	
  	// Creating a separate mission publisher for each individual vehicle
  	for (int32_t i = 0; i < vehicleNames.size(); ++i)
  	{
    	ROS_ASSERT(vehicleNames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    	string vehicleName = static_cast<string>(vehicleNames[i]);
    	
    	string topicName = "/" + vehicleName + "/mission";
    	missionPublishers.push_back(n.advertise<geometry_msgs::PoseStamped>(topicName, 1));
    }

	if(loadMissionsFromFile(missionsFilePath) == 0)
		return 0;
	
	srand((unsigned)time(0));
	
	ros::Rate loop_rate(lRate);
	
	sleep(5); // to ensure that the first missions will be received by the vehicles
	
	while (ros::ok())
  	{
		publishMissions();

	  	ros::spinOnce();
		loop_rate.sleep();
  	}

  return 0;
}
