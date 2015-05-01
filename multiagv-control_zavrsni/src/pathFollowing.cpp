#include <ros/console.h>
#include <sys/time.h>
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>

using namespace std;

double lookahead_dist = 0.2; 	// [0.05 - 0.1]
double linear_velocity = 0.15;	// m/s
double max_angular_vel = 0.3;	// rad/s
double pos_tolerance = 0.03;	// m

bool goal_reached = false;
bool pathPruned = false;

bool isGoalReached()
{
	return goal_reached;
}

void initializePurePursuit(double lookAheadDist, double maxAngVel, double linVel, double posTolerance)
{
	goal_reached = false;
	pathPruned = false;

	lookahead_dist = lookAheadDist;
	max_angular_vel = maxAngVel;
	linear_velocity= linVel;
	pos_tolerance = posTolerance;
}

// Find the nearest point on the path
void prunePath(nav_msgs::Path& global_plan, nav_msgs::Path& transformed_plan)
{
	double x_diff,y_diff;
	double point_dist_sq = 0;
	int index = 0;

	x_diff = transformed_plan.poses[0].pose.position.x;
	y_diff = transformed_plan.poses[0].pose.position.y;
	double prev_dist_sq = x_diff * x_diff + y_diff * y_diff;

	if(transformed_plan.poses.size() > 0)
	{
		for(unsigned int i = 0; i < transformed_plan.poses.size(); i++)
		{
			x_diff = transformed_plan.poses[i].pose.position.x;
			y_diff = transformed_plan.poses[i].pose.position.y;

			point_dist_sq = x_diff * x_diff + y_diff * y_diff;

			if (point_dist_sq < prev_dist_sq)
			{
				prev_dist_sq = point_dist_sq;
				index = i;
			}
		}

		x_diff = transformed_plan.poses[index].pose.position.x;
		y_diff = transformed_plan.poses[index].pose.position.y;

		/*double closest_point_dist = sqrt(x_diff * x_diff + y_diff * y_diff);

		if (closest_point_dist > lookahead_dist)
		{
			ROS_ERROR("The nearest path point is outside the lookAhead radius!");
			return false;
		}*/

		// Uklanjanje svih točaka od početka putanje do točke koja se nalazi najbliže vozilu
		for(int i = 0; i < index; i++)
		{
			transformed_plan.poses.erase(transformed_plan.poses.begin());
			global_plan.poses.erase(global_plan.poses.begin());
		}
	}

	pathPruned = true;
}

bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel, nav_msgs::Path& global_plan, nav_msgs::Path& transformed_plan)
{
	//Check if the vehicle has reached the end of path
	double x_dist,y_dist;

	x_dist = transformed_plan.poses[transformed_plan.poses.size()-1].pose.position.x;
	y_dist = transformed_plan.poses[transformed_plan.poses.size()-1].pose.position.y;

	if (sqrt(x_dist * x_dist + y_dist * y_dist) < pos_tolerance || goal_reached == true)
	{
		goal_reached = true;
		
		// Maknuto zbog loseg utjecaja na upravljanje 
		// (pojavljuju se nagle 0 za vrijeme gibanja
		//cmd_vel.linear.x = 0;
		//cmd_vel.angular.z = 0;

		//fprintf(stderr, "Vehicle has reached the goal point!\n");

		return true;
	}

	if(!pathPruned)
		prunePath(global_plan, transformed_plan);
	
	std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_plan.poses.begin();
	std::vector<geometry_msgs::PoseStamped>::iterator it_global = global_plan.poses.begin();

	x_dist = transformed_plan.poses[0].pose.position.x;
	y_dist = transformed_plan.poses[0].pose.position.y;
	
	 //Move along the path looking for a goal point (one lookahead distance from the current vehicle position)
	double prev_diff = abs(sqrt(x_dist * x_dist + y_dist * y_dist) - lookahead_dist);

	it++;

	while(it != transformed_plan.poses.end())
	{
		const geometry_msgs::PoseStamped& w = *it;

		x_dist = w.pose.position.x;
		y_dist = w.pose.position.y;

		double lookahead_diff = abs(sqrt(x_dist * x_dist + y_dist * y_dist) - lookahead_dist);

		if (lookahead_diff > prev_diff)
		{
			break;
		}
		else
		{
			prev_diff = lookahead_diff;
			it = transformed_plan.poses.erase(it);
			it_global = global_plan.poses.erase(it_global);
		}
	}
	
	double angular_vel;
	double curvature = (2* y_dist)/(x_dist * x_dist + y_dist * y_dist);
		
	if(x_dist > 0) 	// moving forward
	{
		cmd_vel.linear.x  = linear_velocity;
		angular_vel = linear_velocity * curvature;
	}
	else 			// moving backward
	{
		cmd_vel.linear.x  = -linear_velocity;
		angular_vel = -linear_velocity * curvature;
	}
	
	if(angular_vel > max_angular_vel)
		angular_vel = max_angular_vel;
	else if(angular_vel < -max_angular_vel)
		angular_vel = -max_angular_vel;

	cmd_vel.angular.z = angular_vel;

	return true;
}
