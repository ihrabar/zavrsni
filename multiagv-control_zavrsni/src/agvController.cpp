#include <vector>
#include <time.h>
#include <pthread.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include "std_msgs/String.h"
#include "matio.h"
#include <agv_control/vehInfo.h>
#include <agv_control/GetMyPlan.h>
#include <agv_control/Removal.h>
#include <agv_control/CommStatus.h>
#include <agv_control/stateInfo.h>
#include <agv_control/arrayData.h>

using namespace std;

const int POINTS_PER_SEGMENT = 100;
int node_dist = 5;

string myName = "pioneer";			// vehicle name
int myPriority = 1;				// vehicle priority
agv_control::vehInfo myInfo;			// vehicle information to be published to other vehicles
agv_control::planData myPlan;			// the overall result of the planning process
nav_msgs::Path myPath;				// current path information
vector<agv_control::vehInfo> vehInfoList;	// vector for storing data from other vehicles
vector<agv_control::gridData> vehGridList;	// auxiliary vector for storing other vehicles plans

vector<geometry_msgs::PoseStamped> myMissions;

// global ROS publisher handles
ros::Publisher path_pub, vehInfo_pub, polygon_pub, gridCells_pub, cmd_vel_pub, lookAhead_pub, state_pub, goal_reached_pub;

ros::ServiceClient planClient;

void* tfListPointer;

//sem_t mutex;
pthread_mutex_t vehInfo_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mission_mutex = PTHREAD_MUTEX_INITIALIZER;

int mapWidth = 0;
int mapHeight = 0;
float mapResolution = 1;

double lookAheadDistance, linVelocity, maxAngVelocity, posTolerance;

// flags
bool waiting = false;
bool idle = false;
bool removeFlag = false;
bool mapReceived = false;

// recording of statistical data
int numOfStops = 0;
int numOfRemovals = 0;
float shortestPath = 0;
float traveledPath = 0;
float estTimeRequired = 0;

agv_control::stateInfo sInfo;

unsigned int lastMissionSeq = -1;

string base_frame_id;
string global_frame_id;

ros::ServiceServer remServer;

// a vector containing ServiceClients for sending removal requests to other vehicles
vector<ros::ServiceClient> remReqClients;

// a vector containing Subscribers for receiving information from other vehicles
vector<ros::Subscriber> subVehInfo;

vector<vector<int> > lx;
vector<vector<int> > ly;

vector<string> vehBlockedByMe;

geometry_msgs::PoseStamped myCarobst;

void connectToOtherVehicles(ros::NodeHandle n);
bool colCheck(const agv_control::vehInfo& myInfo, const vector<agv_control::vehInfo>& vehInfoList, int mWidth, int mHeight, float mapRes);
bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel, nav_msgs::Path& global_plan, nav_msgs::Path& transformed_plan);
bool isGoalReached();
void initializePurePursuit(double lookAheadDist, double maxAngVel, double maxLinVel, double posTolerance);
void GenerateWrongXY(const vector<string>& vehBlockedByMe, const vector<agv_control::vehInfo>& WrongRobots, \
					int mapWidth, int mapHeight, int dist, vector<int> &WrongX, vector<int> &WrongY);

// ************************* PUBLISHING *****************************
void publishGridCells(nav_msgs::GridCells gCells)
{
	gridCells_pub.publish(gCells);
}

void publishPolygon(ros::Publisher pub, const geometry_msgs::PoseStamped& poly)
{
	double Xposition = poly.pose.position.x;
	double Yposition = poly.pose.position.y;

	/*tf::Quaternion q1;
	tf::quaternionMsgToTF(poly.pose.orientation, q1);
	double yaw = tf::getYaw(q1);*/

	geometry_msgs::PolygonStamped polygon;

	polygon.header.stamp = ros::Time::now();
	polygon.header.frame_id = global_frame_id;

	geometry_msgs::Point32 out_pt;

  	double r = mapResolution * 2;
	double Xpoints[5] = {Xposition - r, Xposition + r, Xposition + r, Xposition, Xposition - r};
	double Ypoints[5] = {Yposition - r, Yposition - r, Yposition + r, Yposition + 2*r, Yposition + r};

	for(int i = 0; i < 5; i++)
	{
		out_pt.x = Xpoints[i];//  * mapResolution;
		out_pt.y = Ypoints[i];//  * mapResolution;
		out_pt.z = 0;
		polygon.polygon.points.push_back(out_pt);
	}

	pub.publish(polygon);
}

void publishLine(ros::Publisher pub, double x1, double y1, double x2, double y2)
{
	geometry_msgs::PolygonStamped line;

	line.header.stamp = ros::Time::now();
	line.header.frame_id = global_frame_id;

	geometry_msgs::Point32 out_pt;

	double Xpoints[2] = {x1, x2};
	double Ypoints[2] = {y1, y2};

	for(int i = 0; i < 2; i++)
	{
		out_pt.x = Xpoints[i];
		out_pt.y = Ypoints[i];
		out_pt.z = 0;
		line.polygon.points.push_back(out_pt);
	}

  	pub.publish(line);
}

void publishMyInfo()
{
	if(vehInfo_pub)
		vehInfo_pub.publish(myInfo);
}
// ******************************************************************

// ************************** SERVICES ******************************
// Sending removal requests to other vehicles
void sendRemovalRequest(string vehicleName)
{
	agv_control::Removal remService;

  	remService.request.vehicleName = myName;
  	remService.request.blockedVehicles = vehBlockedByMe;

	const tf::TransformListener& tfList = *((tf::TransformListener*) tfListPointer);

	tf::StampedTransform transf;
	try
	{
		tfList.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transf);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}

	// Insert vehicle's own pose into request
	tf::pointTFToMsg(transf.getOrigin(), remService.request.reqVehPose.pose.position);
	tf::quaternionTFToMsg(transf.getRotation(), remService.request.reqVehPose.pose.orientation);

  	vector<ros::ServiceClient>::iterator it;

  	// Searching for the blocking vehicle's client service and sending removal request
	for (it = remReqClients.begin(); it != remReqClients.end(); ++it)
	{
		if(it->getService().find(vehicleName) != string::npos)
		{
			if(it->call(remService))
  			{
   				//ROS_INFO("%s accepted removal request from %s", vehicleName.c_str(), myName.c_str());
  			}
  			/*else
  			{
    			ROS_INFO("%s: refused removal request from %s", vehicleName.c_str(), myName.c_str());
  			}*/
			return;
		}
	}
}
// ******************************************************************

// ************************** CALLBACKS *****************************
/** Callback invoked on mission received event */
void missionReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pthread_mutex_lock (&mission_mutex);
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.orientation, q);
	double yaw = tf::getYaw(q);

	ROS_INFO("%s received mission %d: [x y yaw] = [%.2f %.2f %.2f].", myName.c_str(), msg->header.seq, msg->pose.position.x, msg->pose.position.y, yaw);

	myMissions.push_back(*msg);
	pthread_mutex_unlock (&mission_mutex);
}

/** Callback for receiving data from other vehicles */
void vehInfoReceivedCallback(const agv_control::vehInfo::ConstPtr& msg)
{
	pthread_mutex_lock (&vehInfo_mutex);

	for(unsigned int i = 0; i < vehInfoList.size(); i++)
	{
		if(vehInfoList[i].vehicleName == msg->vehicleName)
		{
			unsigned int currSeq = vehInfoList[i].currPathSeq;
			vehInfoList[i] = *msg;

			if(msg->grid.xGrid.size() != 0)
			{
				vehGridList[i] = msg->grid;
			}
			else
			{
				agv_control::gridData gData = vehGridList[i];
				unsigned int currSeg = vehInfoList[i].currPathSeg;
				
				if(currSeg < gData.xGrid.size())
				{
					if(msg->currPathSeq == currSeq)
					{
						for(unsigned int j = currSeg; j < gData.xGrid.size(); j++)
						{
							vehInfoList[i].grid.xGrid.push_back(gData.xGrid[j]);
							vehInfoList[i].grid.yGrid.push_back(gData.yGrid[j]);
							vehInfoList[i].grid.timeInGrid.push_back(gData.timeInGrid[j]);
							vehInfoList[i].grid.timeOutGrid.push_back(gData.timeOutGrid[j]);
						}
					}
					// else
					// ROS_FATAL("%s: Missing %s's trajectory data! Waiting to recive a new trajectory.", myName.c_str(), vehInfoList[i].vehicleName.c_str());
				}
			}

			pthread_mutex_unlock (&vehInfo_mutex);
			return;
		}
	}

	vehInfoList.push_back(*msg);
	vehGridList.push_back(msg->grid);
	pthread_mutex_unlock (&vehInfo_mutex);
}

/** Callback invoked on map received event */
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapWidth =  msg->info.width;
	mapHeight = msg->info.height;

	mapResolution = msg->info.resolution; // [m/cell]
	mapReceived = true;

	ROS_INFO("%s received a %d X %d map @ %.3f m/pix", myName.c_str(), mapWidth, mapHeight, mapResolution);
}

/** Callback invoked on removal request */
bool remReqCallback(agv_control::Removal::Request  &req, agv_control::Removal::Response &res)
{
  	if(waiting || idle)
  	{
  		myCarobst = req.reqVehPose;

  		bool exist = false;
		// Za svako vozilo koje se nalazi u "blockedVehicles" listi vozila koje je poslalo zahtjev za "removingom"..
		// provjerava se da li se to vozilo također nalazi u listi "vehBlockedByMe".. ako se ne nalazi dodaje se..
		for (unsigned int i = 0; i < req.blockedVehicles.size(); i++)
		{
			exist = false;

			for (unsigned int j = 0; j < vehBlockedByMe.size(); j++)
			{
				if (req.blockedVehicles[i] == vehBlockedByMe[j])
				{
					exist = true;
					break;
				}
			}

			if (!exist)
				vehBlockedByMe.push_back(req.blockedVehicles[i]);
		}

		exist = false;

		// Provjerava se da li se vozilo koje je poslalo zahtjev za "removingom"..
		// već nalazi u listi "vehBlockedByMe".. ako se ne nalazi onda se dodaje..
		for (unsigned int j = 0; j < vehBlockedByMe.size(); j++)
		{
			if (req.vehicleName == vehBlockedByMe[j])
			{
				exist = true;
				break;
			}
		}

		// .. ako se već ne nalazi u listi onda se dodaje
		if (!exist)
			vehBlockedByMe.push_back(req.vehicleName);

		removeFlag = true;

  		ROS_INFO("%s accepted removal request from %s", myName.c_str(), req.vehicleName.c_str());
  		return true;
  	}
  	else
  	{
  		//ROS_INFO("%s refused to perform removal action requested by %s", myName.c_str(), req.vehicleName.c_str());
  		return false;
  	}
}

// This callback disables and enables communication with other vehicles
// in order to simulate communication loss and reestablishment 
bool commCallback(agv_control::CommStatus::Request &req, agv_control::CommStatus::Response &res)
{
	if(req.action == "disable")	// close connections
	{
		for(unsigned int i = 0; i < subVehInfo.size(); i++)
		{
			subVehInfo[i].shutdown();
			remReqClients[i].shutdown();
		}
		
		subVehInfo.clear();
		remReqClients.clear();
		
		// stop publishing on vehInfo topic
		vehInfo_pub.shutdown();
		
		// stop accepting removal requests
		remServer.shutdown();
		
		fprintf(stderr, "%s - Communication system disabled!\n", myName.c_str()); 
		
		res.response = true;
	}
	else if(req.action == "enable")	// reestablish connections
	{
		ros::NodeHandle n;

  		// reestablish connections to other vehicles
		connectToOtherVehicles(n);
		
		// readvertise vehInfo topic
		vehInfo_pub = n.advertise<agv_control::vehInfo>("vehInfo", 1);
		
		// readvertise removal request service
		remServer = n.advertiseService("remReqSrv", remReqCallback);
		
		fprintf(stderr, "%s - Communication system enabled!\n", myName.c_str()); 
		
		res.response = true;
	}
	return true;
}
// ******************************************************************

/* communication failures detection */
void checkCommFailures(bool* myCommFailure, bool* otherCommFailure)
{
	*myCommFailure = false;		// failure in my own communication system
	*otherCommFailure = false;	// failure in other vehicle's communication system
	
	for(unsigned int i = 0; i < subVehInfo.size(); i++)
	{
		string topicName = subVehInfo[i].getTopic();
		string vehName = topicName.substr(1, topicName.find("/vehInfo") - 1);

		for(unsigned int j = 0; j < vehInfoList.size(); j++)
		{
			if(vehInfoList[j].vehicleName == vehName)
			{
				if(subVehInfo[i].getNumPublishers() > 0 && vehInfoList[j].inFailure)
				{
					vehInfoList[j].inFailure = false;
				}
				// Ako "subVehInfo[i]" subscriber nema publisher-a
				// više se ne dobivaju informacije od i-tog vozila
				// što se smatra komunikacijskim kvarom na i-tom vozilu 
				else if(subVehInfo[i].getNumPublishers() == 0 && !vehInfoList[j].inFailure)
				{
					vehInfoList[j].inFailure = true;
					*otherCommFailure = true;
					ROS_INFO("%s: Loss of communication with %s!\n", myName.c_str(), vehName.c_str());
					fprintf(stderr, "%s: Loss of communication with %s!\n", myName.c_str(), vehName.c_str());
				}
			}
		}
	}
	
	// Ako niti jedno vozilo nije pretplaćeno na "vehInfo_pub" publisher
	// smatra se da se dogodio kvar u komunikacijskom sustavu ovog vozila
	if((vehInfo_pub.getNumSubscribers() == 0) && vehInfoList.size() > 0) 
		*myCommFailure = true;
}

// Loading coordinates of the cells occupied by the
// stopped vehicle for all vehicle orientations
bool loadStopCells(string fileName)
{
	mat_t *matfp = Mat_Open(fileName.c_str(), MAT_ACC_RDONLY);

	if ( NULL == matfp )
	{
		ROS_ERROR("Error opening MAT file \"%s\"!", fileName.c_str());
		return false;
	}

	char var1Name[] = "stop";

	matvar_t *stop = Mat_VarRead(matfp, var1Name);

	if ( NULL == stop )
	{
		ROS_FATAL("Variable \"%s\" not found or error reading MAT file", var1Name);
		return false;
	}

	matvar_t* matvar;

	for (int i = 0; i < 16; i++)
   	{
		matvar = Mat_VarGetCell(stop, i);
		double* carstop = (double*)matvar->data;

		int m = matvar->dims[0];

		vector<int> tmpx;
	        vector<int> tmpy;

		for (int j = 0; j < m; j++)
		{
			tmpx.push_back((int)carstop[j]);
			tmpy.push_back((int)carstop[j + m]);
		}

		lx.push_back(tmpx);
		ly.push_back(tmpy);
	}

	char var2Name[] = "dist";
	matvar_t *dist = Mat_VarRead(matfp, var2Name);

	if ( dist != NULL )
	{
		const double* data = static_cast<const double*>(dist->data);
		node_dist = (int)data[0];
	}

	Mat_Close(matfp);

	return true;
}

// ********************** TRANSFORMATIONS **************************
int c2d_yaw( double r )
{
	r /= M_PI/8;
    int fi = (r > 0.0) ? (r + 0.5) : (r - 0.5);

	while (fi < 0) 	fi += 16;
	while (fi > 15) fi -= 16;

	return fi;
}
// ******************************************************************

void getStopCells()
{
	const tf::TransformListener& tfList = *((tf::TransformListener*) tfListPointer);

	tf::StampedTransform transf;
	try
	{
		tfList.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transf);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}

	double xPos = transf.getOrigin().x();
	double yPos = transf.getOrigin().y();
	double yaw = tf::getYaw(transf.getRotation());

	int xPos_d = xPos / mapResolution;
	int yPos_d = yPos / mapResolution;
	int yaw_d =  c2d_yaw(yaw);

	nav_msgs::GridCells gCells;
	gCells.header.frame_id = global_frame_id;
	gCells.cell_width = mapResolution;
	gCells.cell_height = mapResolution;

	myInfo.xStop.clear();
	myInfo.yStop.clear();

	for (unsigned int i = 0; i < lx[yaw_d].size(); i++)
	{
		geometry_msgs::Point a;

		myInfo.xStop.push_back(lx[yaw_d][i] + xPos_d);
		myInfo.yStop.push_back(ly[yaw_d][i] + yPos_d);

		a.x = (lx[yaw_d][i] + xPos_d) * mapResolution;
		a.y = (ly[yaw_d][i] + yPos_d) * mapResolution;
		a.z = 0.0;
		gCells.cells.push_back(a);
	}

	publishGridCells(gCells);
}

void clearStopCells()
{
	nav_msgs::GridCells gCells;

	gCells.header.frame_id = global_frame_id;
	gCells.cell_width = mapResolution;
	gCells.cell_height = mapResolution;

	publishGridCells(gCells);
}

// Getting a new path from the pathPlanner node via ROS service
void getPath(agv_control::GetMyPlan::Request &req)
{
	agv_control::GetMyPlan srv;
	srv.request = req;

	agv_control::planData emptyPlan;
	myPlan = emptyPlan;
	myPlan.header = req.start.header;

	ros::Time s_ros, e_ros;
	s_ros = ros::Time::now();

	if (planClient.call(srv))
	{
		myPlan = srv.response.plan;

		if(myPlan.xp.size() > 0)
		{
			e_ros = ros::Time::now();
			double dur_ros = (e_ros - s_ros).toSec(); //.toNSec() * 1e-6;
			fprintf(stderr, "%s obtained a new path for mission %d. (Total segments: %ld, Path length: %.1f m, Calculation time: %.2f s).\n", \
					myName.c_str(), myPlan.header.seq, myPlan.grid.xGrid.size(), myPlan.sp[myPlan.sp.size()-1] * mapResolution, dur_ros);

			/*int size = 0;
			for(unsigned int k = 0; k < myPlan.grid.xGrid.size(); k++)
			{
				size = size + myPlan.grid.xGrid[k].data.size() * sizeof(myPlan.grid.xGrid[k].data[0]);
			}

			size = size * 4;
			ROS_INFO("%s - Total grid data size (to be published on vehInfo topic): %d bytes\n", myName.c_str(), size);*/
		}
		else
			ROS_ERROR("%s: No feasible paths for mission %d.", myName.c_str(), myPlan.header.seq);
	}

	myInfo.header = myPlan.header;
	myInfo.grid = myPlan.grid;
	myInfo.currPathSeq = myPlan.header.seq;
	myInfo.currPathSeg = 0;

	myPath.header = myPlan.header;
	myPath.poses.clear();

	if(myPlan.xp.size() > 0)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.frame_id = myPlan.header.frame_id;

		for(unsigned int i = 0; i < myPlan.xp.size(); i++)
		{
			pose.pose.position.x = mapResolution * myPlan.xp[i];
			pose.pose.position.y = mapResolution * myPlan.yp[i];
			pose.pose.position.z = 0;
			pose.pose.orientation = tf::createQuaternionMsgFromYaw( myPlan.fip[i] );

			myPath.poses.push_back(pose);
			pose.header.seq++;
		}
	}
}

void* executeMission(void* arg)
{
	ros::Time start_time, end_time;

	const tf::TransformListener& tfList = *((tf::TransformListener*) tfListPointer);
	ros::Rate loop(2); // 2Hz -> 500ms;
	
	unsigned int numOtherVehFailures = 0;
	bool myCommFailure = false;
	bool otherCommFailure = false;

	while(ros::ok())
	{
		checkCommFailures(&myCommFailure, &otherCommFailure);
		
		if((myMissions.size() > 0 || removeFlag) && !myCommFailure)
		{
			if(mapReceived)
			{
				idle = false;
				
				// ************************* PATH PLANNING **************************
				pthread_mutex_lock (&vehInfo_mutex);
				myInfo.planning = true;
				myInfo.removing = removeFlag;
				publishMyInfo();
				pthread_mutex_unlock (&vehInfo_mutex);

				geometry_msgs::PoseStamped start;
			  	geometry_msgs::PoseStamped goal;
			  	agv_control::GetMyPlan srv;

			  	start.header.frame_id = global_frame_id;

				tf::StampedTransform transf;
				try
				{
					tfList.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transf);
				}
				catch (tf::TransformException ex)
				{
					ROS_FATAL("%s",ex.what());
					return false;
				}

			  	tf::pointTFToMsg(transf.getOrigin(), start.pose.position);
				tf::quaternionTFToMsg(transf.getRotation(), start.pose.orientation);

				if(!removeFlag)
				{
					pthread_mutex_lock (&mission_mutex);
					geometry_msgs::PoseStamped mission = myMissions.front();
					pthread_mutex_unlock (&mission_mutex);

					start.header = mission.header;
			  		goal.pose = mission.pose;

			  		srv.request.goal = goal;
				}
				else
				{
					srv.request.carobst = myCarobst;
			  		GenerateWrongXY(vehBlockedByMe, vehInfoList, mapWidth, mapHeight, node_dist, srv.request.wrongX, srv.request.wrongY);
			  		removeFlag = false;
				}
				
				// ako je neko vozilo ostalo u kvaru, sve čelije sadržane u segmentu putanje
				// na kojem se dotično vozilo nalazilo u trenutku nastanka kvara 
				// promatraju se kao zabranjene čelije u procesu planiranja putanje  
				for(unsigned int i = 0; i < vehInfoList.size(); i++)
				{
					if(vehInfoList[i].inFailure)
					{	
						if(vehInfoList[i].grid.xGrid.size() > 0)
						{
							srv.request.forbiddenCells.xGrid.push_back(vehInfoList[i].grid.xGrid[0]);
							srv.request.forbiddenCells.yGrid.push_back(vehInfoList[i].grid.yGrid[0]);
						}
						else if(vehInfoList[i].xStop.size() > 0)
						{
							agv_control::arrayData gDataX, gDataY;
							
							for(unsigned int j = 0; j < vehInfoList[i].xStop.size(); j++)
							{
								gDataX.data.push_back( vehInfoList[i].xStop[j] );
								gDataY.data.push_back( vehInfoList[i].yStop[j] );
							}
							
							srv.request.forbiddenCells.xGrid.push_back(gDataX);
							srv.request.forbiddenCells.yGrid.push_back(gDataY);
						}
					}
				}

				start.header.stamp = ros::Time::now();
				goal.header = start.header;

				srv.request.start = start;
				srv.request.removing = myInfo.removing;

				//getPath(start, goal, carobst);
				getPath(srv.request); 

				myInfo.planning = false;
				getStopCells();
				publishMyInfo();
				path_pub.publish(myPath);

				// Podatke o zauzetosti čelija šaljemo samo jednom u vehInfo poruci neposredno nakon što se isplanira trajektorija.
				// Nakon toga ih brišemo iz vehInfo poruke da se smanji količina komunikacijskog prometa.
				agv_control::gridData emptyGrid;
				myInfo.grid = emptyGrid;
				// ******************************************************************

				if(myPlan.xp.size() > 0)
				{
					geometry_msgs::Twist cmd_vel;

					nav_msgs::Path global_plan;
					global_plan.header = myPath.header;
					nav_msgs::Path transformed_plan;

					clearStopCells();
					
					if(!myInfo.removing && (myMissions.front().header.seq != lastMissionSeq))
					{
						lastMissionSeq = myMissions.front().header.seq;
						shortestPath = myPlan.sp[myPlan.sp.size() - 1] * mapResolution;
						estTimeRequired = shortestPath / linVelocity;	// m/s
						start_time = ros::Time::now();
					}
					
					sInfo.moving = 0;
					state_pub.publish(sInfo);
					
					int numOfSegments = myPath.poses.size() / POINTS_PER_SEGMENT;

					for(int i = 0; i <= numOfSegments; i++)
					{
					  	// Deleting the path segment that has just been executed
					  	if(i > 0)
					  	{	
							myPath.poses.erase(myPath.poses.begin(), myPath.poses.begin() + POINTS_PER_SEGMENT);
							path_pub.publish(myPath);
							
							traveledPath =  traveledPath + (myPlan.sp[POINTS_PER_SEGMENT - 1] - myPlan.sp[0]) * mapResolution;
							
							myPlan.xp.erase(myPlan.xp.begin(), myPlan.xp.begin() + POINTS_PER_SEGMENT);
							myPlan.yp.erase(myPlan.yp.begin(), myPlan.yp.begin() + POINTS_PER_SEGMENT);
							myPlan.fip.erase(myPlan.fip.begin(), myPlan.fip.begin() + POINTS_PER_SEGMENT);
							myPlan.sp.erase(myPlan.sp.begin(), myPlan.sp.begin() + POINTS_PER_SEGMENT);

							myPlan.grid.xGrid.erase(myPlan.grid.xGrid.begin());
							myPlan.grid.yGrid.erase(myPlan.grid.yGrid.begin());
							myPlan.grid.timeInGrid.erase(myPlan.grid.timeInGrid.begin());
							myPlan.grid.timeOutGrid.erase(myPlan.grid.timeOutGrid.begin());

							myInfo.currPathSeg++;

							// mission completed!
							if(i == numOfSegments)
							{
								vehBlockedByMe.clear();
								break;
							}
						}
						
						// **************** CHECK FOR COMMUNICATION FAILURES ****************
						checkCommFailures(&myCommFailure, &otherCommFailure);
						// ******************************************************************
						
						if(!myCommFailure && !otherCommFailure)
						{
						  	// ********************** COLLISION DETECTION ***********************
						  	pthread_mutex_lock (&vehInfo_mutex);
						  	if(vehInfoList.size() > 0)
						  	{
						  		agv_control::vehInfo myFullInfo;
						  		myFullInfo = myInfo;
						  		myFullInfo.grid = myPlan.grid;

						  		while(colCheck(myFullInfo, vehInfoList, mapWidth, mapHeight, mapResolution))
						  		{
						  			// **************** CHECK FOR COMMUNICATION FAILURES ****************
									checkCommFailures(&myCommFailure, &otherCommFailure);
									// ******************************************************************
								
									if(myCommFailure || otherCommFailure)
										break;

						  			ROS_INFO("%s detected collision! Transition to WAITING state..", myName.c_str());
						  			pthread_mutex_unlock (&vehInfo_mutex);

						  			if(myInfo.moving)
									{
										numOfStops++;	// number of stops in order to avoid collisions with other vehicles
						  				myInfo.moving = false;
						  				// when the "moving" flag is NOT set, "xStop" and "yStop" data are published in vehInfo message
						  				getStopCells();
						  				publishMyInfo();
						  			}

						  			waiting = true;
						  			sInfo.moving = 0;
						  			state_pub.publish(sInfo);

						  			// **************************** WAITING *****************************
						  			ros::Duration(0.1).sleep();
						  			// ******************************************************************

						  			waiting = false;

						  			pthread_mutex_lock (&vehInfo_mutex);

						  			// Ako je prihvaćen zahtjev za removingom..
						  			if(removeFlag)	break;
						  		}
						  	}
						  	pthread_mutex_unlock (&vehInfo_mutex);
					  	}

					  	if(removeFlag)
					  	{
					  		ROS_INFO("%s - Current mission temporarily suspended due to removal action..", myName.c_str()); 
					  		break;
					  	}
					  	else if(myCommFailure)
					  	{
					  		ROS_INFO("%s - Current mission temporarily suspended for safety reasons!\n", myName.c_str());
					  		break;
					  	}
					  	else if(otherCommFailure)
					  	{
					  		ROS_INFO("%s - Replanning due to communication loss..\n", myName.c_str()); 
					  		break;
					  	}
					  	// ******************************************************************

						// Loading the next path segment..
						for(int j = 0; j < POINTS_PER_SEGMENT; j++)
						{
							global_plan.poses.push_back(myPath.poses[j]);
						}

						myInfo.moving = true;
						// when the "moving" flag is set, "xStop" and "yStop" data are NOT published in vehInfo message
						myInfo.xStop.clear();
						myInfo.yStop.clear();
						clearStopCells();
						publishMyInfo();

						// ****************** NEXT PATH SEGMENT EXECUTION *******************
						ROS_INFO("%s started execution of the next path segment.. (%ld segments remaining, Distance to goal: %.1f [m])", \
							myName.c_str(), myPlan.grid.xGrid.size() - 1, (myPlan.sp[myPlan.sp.size() - 1] - myPlan.sp[0]) * mapResolution);
						
						if(i == 0 && myInfo.removing)
							numOfRemovals++;
							
						initializePurePursuit(lookAheadDistance, maxAngVelocity, linVelocity, posTolerance);

					  	//ros::Time s_ros, e_ros;
					  	ros::Rate r(20); // 20Hz -> 50ms;
					  	while(ros::ok() && !isGoalReached())
					  	{
					  		//s_ros = ros::Time::now();
					  		transformed_plan.poses.clear();

							// Transforming plan from global to vehicle co-ord system:
							for(unsigned int k = 0; k < global_plan.poses.size(); k++)
							{
								geometry_msgs::PoseStamped transformed_pose;

								try
								{
									tfList.transformPose(base_frame_id, global_plan.poses[k], transformed_pose);
								}
								catch (tf::TransformException ex)
								{
								  	ROS_FATAL("Received an exception trying to transform plan from \"%s\" to \"%s\": %s", \
								  			  global_plan.poses[k].header.frame_id.c_str(), base_frame_id.c_str(), ex.what());

								  	cmd_vel.linear.x = 0;
								  	cmd_vel.angular.z = 0;
								  	cmd_vel_pub.publish(cmd_vel);
								  	
								  	sInfo.moving = 0;
					  				state_pub.publish(sInfo);
								  	
								  	return false;
								}

								transformed_plan.poses.push_back(transformed_pose);
							}

							tf::StampedTransform transf;
							tfList.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transf);

					  		if(computeVelocityCommands(cmd_vel, global_plan, transformed_plan))
					  		{
					  			publishLine(lookAhead_pub, transf.getOrigin().x(), transf.getOrigin().y(), \
					  						global_plan.poses[0].pose.position.x, global_plan.poses[0].pose.position.y);

					  			cmd_vel_pub.publish(cmd_vel);
					  			sInfo.moving = 1;
					  			state_pub.publish(sInfo);
					  		}
					  		else
					  		{
					  			cmd_vel.linear.x = 0;
							  	cmd_vel.angular.z = 0;
							  	cmd_vel_pub.publish(cmd_vel);
							  	
							  	sInfo.moving = 0;
					  			state_pub.publish(sInfo);
					  			
							  	return false;
					  		}

					  		r.sleep();
					  		/*e_ros = ros::Time::now();
					  		double dur_ros = (e_ros - s_ros).toNSec() * 1e-6;
					  		fprintf(stderr, "%s - Pure Pursuit sample time [ms]: %.1f\n", myName.c_str(), dur_ros);*/
					  	}
					  	// ******************************************************************
					}
					
					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = 0;
					cmd_vel_pub.publish(cmd_vel);
				}

				if(!myInfo.removing && !removeFlag && !myCommFailure && !otherCommFailure)
				{
					pthread_mutex_lock (&mission_mutex);
					myMissions.erase(myMissions.begin());
					pthread_mutex_unlock (&mission_mutex);
					
					tf::StampedTransform transf;
					tfList.lookupTransform(global_frame_id, base_frame_id, ros::Time(0), transf);
					geometry_msgs::PoseStamped goal_pose;
					
					goal_pose.header.frame_id = global_frame_id;
					goal_pose.header.stamp = ros::Time::now();
					
					tf::pointTFToMsg(transf.getOrigin(), goal_pose.pose.position);
					tf::quaternionTFToMsg(transf.getRotation(), goal_pose.pose.orientation);
					
					goal_reached_pub.publish(goal_pose);
					
					end_time = ros::Time::now();
					double dur_ros = (end_time - start_time).toSec();
					
					// printing statistics:
					ROS_INFO("%s: Mission %d completed! (shortestPath: %.2f, traveledPath: %.2f, Stops: %d, Removals: %d, estTime: %.2f totalTime: %.2f)\n", \
					myName.c_str(), myMissions.front().header.seq, shortestPath, traveledPath, numOfStops, numOfRemovals, estTimeRequired, dur_ros);
					
					//fprintf(stderr, "%s:\t\t%d\t\t%.2f\t\t%.2f\t\t%d\t\t%d\t\t%.2f\t\t%.2f\n", \
					//myName.c_str(), myMissions.front().header.seq, shortestPath, traveledPath, numOfStops, numOfRemovals, estTimeRequired, dur_ros);
					
					traveledPath = 0;
					numOfRemovals = 0;
					numOfStops = 0;
				}	
				
				sInfo.moving = 0;
				state_pub.publish(sInfo);
			
				myInfo.moving = false;
				myInfo.removing = false;
				// when the "moving" flag is NOT set, "xStop" and "yStop" data are published in vehInfo message
				getStopCells();
				publishMyInfo();
	  		}
	  		else
	  			ROS_ERROR_ONCE("Could not execute a given mission due to unknown map!");
	  	}
	  	else idle = true;

  		loop.sleep();
  	}

  	return false;
}

void connectToOtherVehicles(ros::NodeHandle n)
{
	XmlRpc::XmlRpcValue vehicleNames;
  	XmlRpc::XmlRpcValue vehiclePriorities;

  	// Getting vehicle names from the ROS Parameter Server
  	n.getParam("/vehicleNames", vehicleNames);
  	ROS_ASSERT(vehicleNames.getType() == XmlRpc::XmlRpcValue::TypeArray);

  	// Getting vehicle priorities from the ROS Parameter Server
  	n.getParam("/vehiclePriorities", vehiclePriorities);
  	ROS_ASSERT(vehiclePriorities.getType() == XmlRpc::XmlRpcValue::TypeArray);
  	
  	for (int32_t i = 0; i < vehicleNames.size(); ++i)
  	{
    	ROS_ASSERT(vehicleNames[i].getType() == XmlRpc::XmlRpcValue::TypeString);
    	string vehicleName = static_cast<string>(vehicleNames[i]);

    	// Ako ime i-tog vozila nije sadržano u imenu ovoga node-a, pretplaćujemo se na topic "/vehInfo" i-tog vozila.
		// Također se generira klijent za slanje "removal requesta" putem "/remReqSrv" servisa tom vozilu
    	// (na ovaj se način svako vozilo pretplaćuje na "/vehInfo" topic i "/remReqSrv" servis svih ostalih vozila).
    	if (ros::this_node::getName().find(vehicleName) == string::npos)
    	{
			string topicName = "/" + vehicleName + "/vehInfo";
			subVehInfo.push_back(n.subscribe(topicName, 10, vehInfoReceivedCallback));
			
			topicName = "/" + vehicleName + "/remReqSrv";
			remReqClients.push_back(n.serviceClient<agv_control::Removal>(topicName));
    	}
    	else
    	{
    		myName = vehicleName;

    		ROS_ASSERT(vehiclePriorities[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
    		myPriority = static_cast<int>(vehiclePriorities[i]);
    	}
  	}
}

int main(int argc, char **argv)
{
  	/**
   	* The ros::init() function needs to see argc and argv so that it can perform
   	* any ROS arguments and name remapping that were provided at the command line. For programmatic
   	* remappings you can use a different version of init() which takes remappings
   	* directly, but for most command-line programs, passing argc and argv is the easiest
   	* way to do it.  The third argument to init() is the name of the node.
   	*
   	* You must call one of the versions of ros::init() before using any other
   	* part of the ROS system.
   	*/
  	ros::init(argc, argv, "AGV_ROS_Controller");

  	/**
   	* NodeHandle is the main access point to communications with the ROS system.
   	* The first NodeHandle constructed will fully initialize this node, and the last
   	* NodeHandle destructed will close down the node.
   	*/
  	ros::NodeHandle n;
  	ros::NodeHandle private_nh("~");
	
	// creating communication connections to other vehicles
	connectToOtherVehicles(n);

  	/**
   	* The subscribe() call is how you tell ROS that you want to receive messages
   	* on a given topic.  This invokes a call to the ROS
   	* master node, which keeps a registry of who is publishing and who
   	* is subscribing.  Messages are passed to a callback function, here
   	* called chatterCallback.  subscribe() returns a Subscriber object that you
   	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   	* object go out of scope, this callback will automatically be unsubscribed from
   	* this topic.
   	*
   	* The second parameter to the subscribe() function is the size of the message
   	* queue.  If messages are arriving faster than they are being processed, this
   	* is the number of messages that will be buffered up before beginning to throw
   	* away the oldest ones.
   	*/

  	// Subscriber za primanje misija:
  	ros::Subscriber subMission = n.subscribe("mission", 1, missionReceivedCallback);

  	// Subscriber za primanje misija iz RViz-a:
  	ros::Subscriber subGoal = n.subscribe("move_base_simple/goal", 1, missionReceivedCallback);

  	// Subscriber za primanje karte prostora:
  	ros::Subscriber subMap = n.subscribe("map", 1, mapReceivedCallback);

  	// klijent za planiranje putanje preko servisa
  	planClient = n.serviceClient<agv_control::GetMyPlan>("getPlanSrv");

  	/**
   	* The advertise() function is how you tell ROS that you want to
   	* publish on a given topic name. This invokes a call to the ROS
   	* master node, which keeps a registry of who is publishing and who
   	* is subscribing. After this advertise() call is made, the master
   	* node will notify anyone who is trying to subscribe to this topic name,
   	* and they will in turn negotiate a peer-to-peer connection with this
   	* node.  advertise() returns a Publisher object which allows you to
   	* publish messages on that topic through a call to publish().  Once
   	* all copies of the returned Publisher object are destroyed, the topic
   	* will be automatically unadvertised.
   	*
   	* The second parameter to advertise() is the size of the message queue
   	* used for publishing messages.  If messages are published more quickly
   	* than we can send them, the number here specifies how many messages to
   	* buffer up before throwing some away.
   	*/

	path_pub = n.advertise<nav_msgs::Path>("pathToGoal", 1);					// Path
	vehInfo_pub = n.advertise<agv_control::vehInfo>("vehInfo", 1);				// vehInfo
	polygon_pub = n.advertise<geometry_msgs::PolygonStamped>("polygon", 10);	// footprint vozila
	lookAhead_pub = n.advertise<geometry_msgs::PolygonStamped>("lookAhead", 1);	// prikaz look ahead distance-a u RViz-u
	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);				// linearna i kutna brzina vozila
  	gridCells_pub = n.advertise<nav_msgs::GridCells>("collCells", 1);
  	state_pub = n.advertise<agv_control::stateInfo>("state", 1);
  	goal_reached_pub = n.advertise<geometry_msgs::PoseStamped>("goal_reached/pose", 1);	// pozicija vozila na kraju misije
	
  	remServer = n.advertiseService("remReqSrv", remReqCallback);
  	ros::ServiceServer commServer = n.advertiseService("communication", commCallback);

  	static tf::TransformBroadcaster br;
  	tf::TransformListener tfListener;

  	tfListPointer = &tfListener;

  	n.param("/lookAheadDistance", lookAheadDistance, 0.2);
	n.param("/linVelocity", linVelocity, 0.15);
	n.param("/maxAngVelocity", maxAngVelocity, 0.3);
	n.param("/posTolerance", posTolerance, 0.03);

	private_nh.param("base_frame_id", base_frame_id, std::string("/" + myName + "/base_link"));
  	private_nh.param("global_frame_id", global_frame_id, std::string("/" + myName + "/map"));

	string MAT_fileName = "";

	if(argc >= 2)
		MAT_fileName = argv[1];
	else
	{
		ROS_FATAL("MAT file path not specified!");
		return 0;
	}

	if(!loadStopCells(MAT_fileName))
		return 0;

	myInfo.vehicleName = myName;
	myInfo.priority = myPriority;
	myInfo.moving = false;
	myInfo.planning = false;
	myInfo.removing = false;
	myInfo.currPathSeq = 0;
	myInfo.currPathSeg = 0;

	// A new thread for the mission execution task:
	pthread_t thread;
	pthread_create(&thread, NULL, executeMission, NULL);

	/**
   	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
   	* callbacks will be called from within this thread (the main one).  ros::spin()
   	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   	*/

  	ros::spin();

  	return 0;
}
