#ifndef MAT_BY_NAME // fix issue with different versions of matio library
#define MAT_BY_NAME BY_NAME
#endif

#include "ros/ros.h"
#include "matio.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <tf/transform_datatypes.h>
#include <agv_control/GetMyPlan.h>
#include <agv_control/planData.h>
#include <nav_msgs/GridCells.h>

using namespace std;

struct lattice_params {
	matvar_t * vars;
	matvar_t * cost;
	matvar_t * edges;
	matvar_t * stop;
	matvar_t * dist;
} latt_coarse, latt_fine;

double* myMap = NULL;
double* originalMap = NULL;
int mapWidth = 0;
int mapHeight = 0;
float mapResolution = 0;
int node_dist = 5;
ros::Publisher gridCells_pub;

void graphSearch(matvar_t * varVars, matvar_t * varCost, const vector<vector<double> >& map, int start[], int finish[], bool removing, vector<int> wrX, vector<int> wrY, double** M, int sizeM[], float mapRes, int nodeDist);
void generate_path_data(double* M, int numOfSeg, lattice_params &lattice, agv_control::planData &plan, int start[], double ts0);

// ********************** TRANSFORMATIONS **************************
// diskretizacija orijentacije vozila ([0, 2*PI] -> [0, 16]):
int c2d_yaw( double r )
{
	r /= M_PI/8;
    int fi = (r > 0.0) ? (r + 0.5) : (r - 0.5);

    while (fi < 0) 	fi += 16;
    while (fi > 15) fi -= 16;

    return fi;
}

int c2d_pose( double r, int node_dist )
{
    return (r > 0.0) ? (r + (float)node_dist/2) / node_dist : (r - (float)node_dist/2) / node_dist;
}
// ******************************************************************

// ************************* PUBLISHING *****************************
void publishVisitedNodes(nav_msgs::GridCells gCells)
{
	gridCells_pub.publish(gCells);
}
// ******************************************************************

// ************************** CALLBACKS *****************************
/** Callback invoked on map received event */
void mapReceivedCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapWidth =  msg->info.width;
	mapHeight = msg->info.height;

  	mapResolution = msg->info.resolution; // [m/cell]

  	originalMap = new double[mapWidth * mapHeight * sizeof(double)];

  	for (int i = 0; i < mapWidth * mapHeight; i++)
  	{
		if (msg->data[i] != 0) 		// (msg->data[i] == 100 || msg->data[i] == -1)  -> occupied or unknown cell in OccupancyGrid message
			originalMap[i] = -1; 	// occupied cell in pathPlanner's map representation
		else						// (msg->data[i] == 0) -> unoccupied cell in OccupancyGrid message
			originalMap[i] = 1;		// unoccupied cell in pathPlanner's map representation
			
  	}

  	ROS_INFO("pathPlanner received a %d X %d map @ %.3f m/pix", mapWidth, mapHeight, mapResolution);
}

/** Callback invoked on getPath request */
bool getPlanCallback(agv_control::GetMyPlan::Request &req, agv_control::GetMyPlan::Response &res)
{
	if(originalMap != NULL)
	{
		myMap = new double[mapWidth * mapHeight];
		memcpy(myMap, originalMap, mapWidth * mapHeight * sizeof(double));
		const double* data = static_cast<const double*>(latt_coarse.dist->data);
		node_dist = (int)data[0];
			
		int xStart = c2d_pose(req.start.pose.position.x / mapResolution, node_dist);
		int YStart = c2d_pose(req.start.pose.position.y / mapResolution, node_dist);

		tf::Quaternion q1;
		tf::quaternionMsgToTF(req.start.pose.orientation, q1);
		double startYaw = tf::getYaw(q1);

		int fiStart = c2d_yaw(startYaw);
		
		int start[] =  {xStart, YStart, fiStart};

		int finish[3];

		if(req.forbiddenCells.xGrid.size() > 0)
		{
			// zabranjene čelije ucrtavaju se u kartu prostora kao prepreke
			for (unsigned int i = 0; i < req.forbiddenCells.xGrid.size(); i++)
			{
				for (unsigned int j = 0; j < req.forbiddenCells.xGrid[i].data.size(); j++)
				{
					myMap[(int)req.forbiddenCells.xGrid[i].data[j] + ((int)req.forbiddenCells.yGrid[i].data[j]) * mapWidth] = -1;
				}
			}
		}
		// učitavanje koordinata i orijentacije vozila koje predstavlja prepreku
		if(req.removing)
		{
			// diskretizacija prostora
			int carx = c2d_pose(req.carobst.pose.position.x / mapResolution, node_dist);
			int cary = c2d_pose(req.carobst.pose.position.y / mapResolution, node_dist);

			tf::quaternionMsgToTF(req.carobst.pose.orientation, q1);
			double carobstFI = tf::getYaw(q1);

			int carfi = c2d_yaw(carobstFI);

			matvar_t* matvar = Mat_VarGetCell(latt_coarse.stop, carfi);
			double* carstop = (double*)matvar->data;

			int m = matvar->dims[0];

			// ucrtavanje prepreke u kartu prostora
			for (int i = 0; i < m; i++)
			{
				myMap[(carx + (int)carstop[i]) + (cary + (int)carstop[i + m]) * mapWidth] = -1;
			}
		}
		else
		{
			// diskretizacija prostora
			int xGoal = c2d_pose(req.goal.pose.position.x / mapResolution, node_dist);
			int yGoal = c2d_pose(req.goal.pose.position.y / mapResolution, node_dist);

			tf::quaternionMsgToTF(req.goal.pose.orientation, q1);
			double goalYaw = tf::getYaw(q1);

			int fiGoal = c2d_yaw(goalYaw);

			finish[0] = xGoal;
			finish[1] = yGoal;
			finish[2] = fiGoal;

			if(xGoal == xStart && yGoal == YStart && fiGoal == fiStart)
				ROS_WARN("Start and goal poses are identical!");
		}
		
		vector<vector<double> > vMap;	// vektorski zapis karte
		vector<double> dtm;
		for (int i = 0; i < mapWidth; i++)
		{
		    dtm.clear();
		    for (int j = 0; j < mapHeight; j++)
		    {
		        dtm.push_back(myMap[i + j * mapWidth]);
		    }  
		    vMap.push_back(dtm);
		}
		
		delete myMap;

		double* M = NULL; // matrica u koju graphSearch pohranjuje indekse elemenata "edges" matrice koji sadrže segmente izračunate putanje
		int sizeM[2];
		
		graphSearch(latt_coarse.vars, latt_coarse.cost, vMap, start, finish, req.removing, req.wrongX, req.wrongY, &M, sizeM, mapResolution, node_dist);
	
		// **********************************************************************************************************************
		// Ako putanja nije uspješno isplanirana (npr. zato što u originalnom startnom čvoru vozilo zahvaća neku blisku prepreku)
		// pokušava se isplanirati sa startnom pozicijom u jednom od 8 susjednih čvorova:
		// **********************************************************************************************************************
		if(!(start[0] == finish[0] && start[1] == finish[1] && start[2] == finish[2]))
		{
			vector<pair<int, int> > neighbors;	// parovi x-y koordinata svih susjednih čvorova
			
			if(start[0] + 1 < mapWidth / node_dist)
			{
				neighbors.push_back(make_pair(start[0] + 1, start[1]));
			}
			else if(start[0] - 1 > 0)
			{
				neighbors.push_back(make_pair(start[0] - 1, start[1]));
			}
			else if(start[1] + 1 < mapHeight / node_dist)
			{
				neighbors.push_back(make_pair(start[0], start[1] + 1));
			}
			else if(start[1] - 1 > 0)
			{
				neighbors.push_back(make_pair(start[0], start[1] - 1));
			}
			else if((start[0] + 1 < mapWidth / node_dist) && (start[1] + 1 < mapHeight / node_dist))
			{
				neighbors.push_back(make_pair(start[0] + 1, start[1] + 1));
			}
			else if((start[0] + 1 < mapWidth / node_dist) && (start[1] - 1 > 0))
			{
				neighbors.push_back(make_pair(start[0] + 1, start[1] - 1));
			}
			else if((start[0] - 1 > 0) && (start[1] + 1 < mapHeight / node_dist))
			{
				neighbors.push_back(make_pair(start[0] - 1, start[1] + 1));
			}
			else if((start[0] - 1 > 0) && (start[1] - 1 < 0))
			{
				neighbors.push_back(make_pair(start[0] - 1, start[1] - 1));
			}
			
			int index = 0;

			while(!(M != NULL && sizeM[1] > 0) && index < neighbors.size())
			{
				start[0] = neighbors[index].first;
				start[1] = neighbors[index].second;
				
				graphSearch(latt_coarse.vars, latt_coarse.cost, vMap, start, finish, req.removing, req.wrongX, req.wrongY, &M, sizeM, mapResolution, node_dist);
				
				index++;
			}
		}
		// **********************************************************************************************************************
		
		int numOfSegments = sizeM[1];
		
		if(M != NULL && numOfSegments > 0)
		{	
			double ts0 = 0;
			
			if(latt_fine.vars == NULL || req.removing)	// ako se ne koristi planiranje s dvije latice ILI se radi o removingu
			{
				generate_path_data(M, numOfSegments, latt_coarse, res.plan, start, ts0);
			}
			else
			{
				// *******************************************************************************************************
				// Replaniranje završnog dijela putanje pomoću fine latice kako bi se postigla veća točnost pozicioniranja
				// *******************************************************************************************************
				if(numOfSegments > 3)
				{
					generate_path_data(M, numOfSegments - 3, latt_coarse, res.plan, start, ts0);
					
					// učitavanje "node_dist" parametra iz fine latice
					const double* data = static_cast<const double*>(latt_fine.dist->data);
					node_dist = (int)data[0];
					
					start[0] = c2d_pose(res.plan.xp[res.plan.xp.size() - 1], node_dist);
					start[1] = c2d_pose(res.plan.yp[res.plan.yp.size() - 1], node_dist);
					start[2] = c2d_yaw(res.plan.fip[res.plan.fip.size() - 1]);
					ts0 = res.plan.sp[res.plan.sp.size() - 1];
				}
				else
				{
					start[0] = c2d_pose(req.start.pose.position.x / mapResolution, node_dist);
					start[1] = c2d_pose(req.start.pose.position.y / mapResolution, node_dist);
				}
	
				// određivanje ciljnog čvora
				finish[0] = c2d_pose(req.goal.pose.position.x / mapResolution, node_dist);
				finish[1] = c2d_pose(req.goal.pose.position.y / mapResolution, node_dist);
				
				// PLANIRANJE POMOĆU FINE LATICE
				graphSearch(latt_fine.vars, latt_fine.cost, vMap, start, finish, req.removing, req.wrongX, req.wrongY, &M, sizeM, mapResolution, node_dist);
				generate_path_data(M, sizeM[1], latt_fine, res.plan, start, ts0);
				// *******************************************************************************************************
			}
		}
		else
		{
			res.plan.xp.clear();
			res.plan.yp.clear();
			res.plan.fip.clear();
			res.plan.sp.clear();

			res.plan.grid.xGrid.clear();
			res.plan.grid.yGrid.clear();
			res.plan.grid.timeInGrid.clear();
			res.plan.grid.timeOutGrid.clear();
		}

		res.plan.header = req.start.header;
		res.plan.header.stamp = ros::Time::now();
		
		delete M;
	}
	else
	{
		ROS_ERROR("Unable to plan the path.. pathPlanner node has not received the map!");
		return false;
	}

	return true;
}
// ******************************************************************

// ******************************************************************
void generate_path_data(double* M, int Mn, lattice_params &lattice, agv_control::planData &plan, int start[], double ts0)
{
	matvar_t *dx, *dy, *x, *y, *fi, *sf, *cost;

	char var1Name[] = "dx";
	char var2Name[] = "dy";
	char var3Name[] = "x";
	char var4Name[] = "y";
	char var5Name[] = "fi";
	char var6Name[] = "sf";
	char var7Name[] = "cost";

	int k = 0;
	
	int tx = node_dist * start[0];
	int ty = node_dist * start[1];
	double ts = ts0;
			
	for(int i = 0; i < Mn; i++)
	{
		int struct_lin_index = M[k] + M[k+1]*(lattice.edges->dims[0]); // M[k] -> indeks retka u "edges", M[k+1] -> indeks stupca

		dx = Mat_VarGetStructField(lattice.edges, var1Name, MAT_BY_NAME, struct_lin_index);
		dy = Mat_VarGetStructField(lattice.edges, var2Name, MAT_BY_NAME, struct_lin_index);
		 x = Mat_VarGetStructField(lattice.edges, var3Name, MAT_BY_NAME, struct_lin_index);
		 y = Mat_VarGetStructField(lattice.edges, var4Name, MAT_BY_NAME, struct_lin_index);

		fi = Mat_VarGetStructField(lattice.edges, var5Name, MAT_BY_NAME, struct_lin_index);
		sf = Mat_VarGetStructField(lattice.edges, var6Name, MAT_BY_NAME, struct_lin_index);
		cost = Mat_VarGetStructField(lattice.edges, var7Name, MAT_BY_NAME, struct_lin_index);

		double* dxData = static_cast<double*>(dx->data);
		double* dyData = static_cast<double*>(dy->data);
		double*  xData = static_cast<double*>( x->data);
		double*  yData = static_cast<double*>( y->data);
		double* fiData = static_cast<double*>(fi->data);
		double* sfData = static_cast<double*>(sf->data);
		double*  CData = static_cast<double*>(cost->data);
				
		int xm = x->dims[1];
		for(int j = 0; j < xm; j++)
		{
			plan.xp.push_back( xData[j] + tx );
			plan.yp.push_back( yData[j] + ty );
			plan.fip.push_back( fiData[j] ); //* 180/M_PI );
		}

		for(int j = 0; j <= 99; j++)
		{
			plan.sp.push_back( sfData[0]/99 * j + ts );
		}

		int Cm = cost->dims[0];

		agv_control::arrayData gDataX, gDataY, gDataTI, gDataTO;

		for(int j = 0; j < Cm; j++)
		{
			gDataX.data.push_back( CData[j] + tx );
			gDataY.data.push_back( CData[Cm + j] + ty );
			gDataTI.data.push_back( CData[2*Cm + j] );
			gDataTO.data.push_back( CData[3*Cm + j] );
		}

		if(i == Mn - 1)
		{
			double N = gDataTO.data[0];

			for(unsigned int j = 0; j < gDataTO.data.size(); j++)
			{
				if(gDataTO.data[j] > N)
					N = gDataTO.data[j];
			}

			for(unsigned int j = 0; j < gDataTO.data.size(); j++)
			{
				if(abs(gDataTO.data[j] - N) < 0.001)
					gDataTO.data[j] = 10000;
			}
		}

		plan.grid.xGrid.push_back(gDataX);
        plan.grid.yGrid.push_back(gDataY);
        plan.grid.timeInGrid.push_back(gDataTI);
        plan.grid.timeOutGrid.push_back(gDataTO);

		tx += dxData[0];
		ty += dyData[0];
		ts += sfData[0];

		k += 2;
	}
}
// ******************************************************************

// ********************** LOADING ".mat" file ***********************
bool load_MAT_file(const char *file_name, lattice_params &lattice)
{
	mat_t *matfp;

	char var1Name[] = "vars";	// matrica čiji retci povezuju početnu orijentaciju, koord. susjednog čvora, konačnu orijentaciju i cijenu pripadajućeg segmenta
	char var2Name[] = "cost";	// polje cell-ova koji sadrže koordinate svih čelija kroz koje vozilo prolazi prilikom gibanja pojedinim segmentom putanje
	char var3Name[] = "edges";	// matrica cell-ova koji sadrže sljedeće informacije o svakom segmentu latice: dx, dy, fif, x, y, fi, sf, inTime, outTime, R, cost.
	char var4Name[] = "stop";	// polje cell-ova koji sadrže koordinate prostornih čelija koje vozilo okupira u mirovanju pri pojedinoj orijentaciji
	char var5Name[] = "dist";	// podatak o udaljenosti između čvorova u latici (u jediničnim duljinama)

	matfp = Mat_Open(file_name, MAT_ACC_RDONLY);

	if ( matfp == NULL )
	{
		ROS_FATAL("Error opening MAT file \"%s\"!", file_name);
		return false;
	}

	lattice.vars = Mat_VarRead(matfp, var1Name);

	if ( lattice.vars == NULL )
	{
		ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var1Name, file_name);
		return false;
	}

	//Mat_VarPrint( lattice.vars, 1);

	lattice.cost = Mat_VarRead(matfp, var2Name);

	if ( lattice.cost == NULL )
	{
		ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var2Name, file_name);
		return false;
	}

	//Mat_VarPrint( lattice.cost, 1);

	lattice.edges = Mat_VarRead(matfp, var3Name);

	if ( lattice.edges == NULL )
	{
		ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var3Name, file_name);
		return false;
	}

	//Mat_VarPrint( lattice.edges, 1);

	lattice.stop = Mat_VarRead(matfp, var4Name);

	if ( lattice.stop == NULL )
	{
		ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var4Name, file_name);
		return false;
	}

	//Mat_VarPrint( lattice.stop, 1);

	lattice.dist = Mat_VarRead(matfp, var5Name);

	if ( lattice.dist == NULL )
	{
		ROS_FATAL("Variable \"%s\" not found, or error reading MAT file \"%s\"!", var5Name, file_name);
		return false;
	}
	// ******************************************************************

	Mat_Close(matfp);
	
	return true;
}
	
int main(int argc, char **argv)
{
	ros::init(argc, argv, "pathPlanner");
	ros::NodeHandle n;

	if(load_MAT_file(argv[1], latt_coarse) == false)	// loading latt_coarse params
		return 0;

	if(argc > 2)
		if(load_MAT_file(argv[2], latt_fine) == false)	// loading latt_fine params
			return 0;

	// Subscriber za primanje karte prostora:
  	ros::Subscriber subMap = n.subscribe("map", 1, mapReceivedCallback);

  	ros::ServiceServer getPathService = n.advertiseService("getPlanSrv", getPlanCallback);
  	
  	gridCells_pub = n.advertise<nav_msgs::GridCells>("visitedNodes", 1);

	ros::spin();

	delete originalMap;
	
	Mat_VarFree(latt_coarse.vars);
	Mat_VarFree(latt_coarse.cost);
	Mat_VarFree(latt_coarse.edges);
	Mat_VarFree(latt_coarse.stop);

	return 0;
}
