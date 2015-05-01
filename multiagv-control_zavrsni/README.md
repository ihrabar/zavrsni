# 1.0 Introduction #
 
"agv_control" is a ROS package that implements an algorithm for 
decentralized control of free ranging Automated Guided Vehicles 
(AGVs) operating in automated warehouse environments. The package
implements all the functionalities required for missions execution
which implies mission receiving, path planning and following, 
detection of potential collisions with other vehicles and confilict 
situation resolution. The architecture of this package makes it 
suitable for the use in real AGV systems comprised of ROS-based 
platforms, as well as for testing purposes on an arbitrary number of
virtual vehicles.

# 2.0 Platform #

agv_control package has been developed and tested using the following 
setup: ROS Hydro Medusa distribution on the Linux Ubuntu 12.04 platform.
	
# 3.0 Building #

This section describes agv_control's dependencies and gives
instructions for its building.
    
## 3.1 Dependencies ##
    
### matio (system dependency) ###

Matio is an open-source library for reading/writing MATLAB 
MAT files. This library is used for reading various predefined 
trajectory parameters that have been generated in Matlab and 
stored in the agv_control/param/params.mat file.

Matio library can be installed by:

      $ sudo apt-get install libmatio-dev

(Current version available in Ubuntu Precise is 1.3.4-3 check if it's ok! Latest version on SourceForge 1.5.2.)
    
### p2os_urdf (ROS dependency) ###

The p2os_urdf ROS package is needed for visualizing the demo scenario.


    $ sudo apt-get install ros-hydro-p2os-urdf
        
## 3.2 Building agv_control ##

Once the matio library has been successfully installed as described in 
section 3.1.1, we are ready to build and run the agv_control package. 
For that purpose, after unpacking the agv_control package, move it 
to directory on the ROS path (for example ~/catkin_ws/src) 
and rosmake it:
		
    $ tar zxf agv_control.tar.gz
    $ mv agv_control ~/catkin_ws/src
    $ rospack profile
    $ cd ~/catkin_ws
    $ catkin_make
	
Upon successful building, the agv_control package is ready for use. 
For instructions on how to use the package for simulation purposes 
please refer to section 5.1.

# 4.0 agv_control package architecture #

This section describes the architecture of the agv_control package 
by means of implemented nodes and main topics and messages.

## NODES: ##

### agvController: ###
This is the main node which implements decentralized control 
logic and provides the vehicles with capabilities for 
autonomous motion planning and decision making. This node 
communicates with agvController nodes on all other vehicles
in order to exchange path information and the current 
vehicle states. 
						
### pathPlanner: ###
This node implements a path planning algorithm. It accepts requests 
for new paths from agvController node(s), obtains the feasible
path for the given mission and returns the calculated path
information. The pathPlanner and agvController nodes
communicate via request/response architecture using ROS services. 
One pathPlanner node can serve more than one agvController 
nodes which is practical when running simulations.
						
### missionPublisher: ###
This node is used to publish missions for all vehicles. The node 
loads mission information at startup from the missions file	
passed as an argument and publishes missions sequentially at 
a constant rate. The mission publishing rate can be adjusted to 
desired value using "-rate" option. Also, the order in which the 
missions are published can be randomized by starting the node with
"-rand" option. For more information on the startup options run the 
node with "--help" option and see the comments in 
"agv_control/missions/missions.txt" file.
						
### poseEstimator: ###
This node estimates the motion of a vehicle by receiving velocity
commands from the agvController node(s) via "/cmd_vel" topic 
and by calculating new vehicle's position and orientation. The node 
has been developed only for the simulation purposes and should be
disabled in real applications since the real ROS-based AGV 
vehicles provide their own nodes for execution of velocity commands 
and estimation of the current vehicle pose.

## MAIN TOPICS AND MESSAGES: ##

### /mission ###
agvController node of each vehicle subscribes to this topic in 
the vehicle's namespace in order to receive a new mission 
information via "geometry_msgs::PoseStamped" message.
    
### /initialPose ###
agvController node of each vehicle subscribes to this topic in 
the vehicle's namespace in order to recive an initial pose 
information via "geometry_msgs::PoseWithCovarianceStamped" message.

### /cmd_vel ###
agvController node of each vehicle publishes velocity commands
to this topic in the vehicle's namespace via "geometry_msgs::Twist" 
messages.
	
### /vehInfo ###
This topic is used for communication between agvController
nodes of different vehicles using custom "agv_control::vehInfo" 
messages.
	 
In order to separate topics of the same type running on different 
vehicles, the topics are defined within different namespaces named
after the vehicle's name. Therefore, all of the above listed topics 
are prefixed by the specific vehicle's name. For example, vehicle 
named "Alfa" subscribes to "/Alfa/mission" topic and publishes to 
"/Alfa/cmd_vel" topic.
	
# 5.0 Simulation mode #
                
## 5.1 Starting simulation ##
The simulation of decentralized control algorithm can be started using
roslaunch tool with a configuration file which specifies the 
parameters to set on the ROS parameter server and nodes to launch. 

The configuration file included in this package, which specifies all 
the parameters for the simulation, is named "agv_control_sim.launch". 
Accordingly, the following command should be executed to start the
simulation:

`$ roslaunch agv_control agv_control_sim.launch`

The above command starts one agvController node per vehicle, 
the pathPlanner node, the map_server node, the poseEstimator node and 
the RViz tool for visualization of the map and 3D vehicle models. 

## Setting initial position: ##
Upon starting, all vehicles are located in the (0,0) point on the map. 
Therefore, before starting the mission execution it is necessary to 
move each vehicle in a desired initial pose. This can be done by 
publishing initial pose message to each vehicle's /initialPose topic.
In order to make the process of pose initialization easier, this package
includes a simple script intended for loading vehicle initial poses
from the files (located in agv_control/initPose directory) and publishing
each pose to corresponding vehicle's /initalPose topic. Accordingly, 
to set all the vehicles to predefined initial poses, the following 
commands should be executed in a new terminal:

    $ roscd agv_control
    $ cd initPose
    $ ./setInitialPoses.sh
    
Upon execution of the above commands, vehicle models should change their
locations and orientations on the map in RViz environment. At this 
moment we are ready to start the missionPublisher node. The information
about all vehicle missions is stored in the "missions.txt" file located 
in "agv_control/missions/" directory, thus the missionPublisher node can 
be started by the following command:
 
`$ rosrun agv_control missionPublisher -mf `rospack find agv_control`/missions/missions.txt`

Upon execution of this command, vehicles should start moving toward 
their goal positions which can be seen in RViz environment.
 
## 5.2 Customization of simulation parameters ##

The agv_control package is designed in a way that allows for easy
customization of various simulation parameters by editing appropriate
configuration files.

For example, by editing "agv_control_sim.launch" it is possible to:
-  add (remove) vehicles to (from) simulation, 
-  set vehicle names and priorities, 
-  specify the map file,
-  specify the missions file,
-  specify arbitrary ROS nodes to start with simulation, etc.

To customize vehicle missions it is neccessary to modify the 
"missions.txt" file or to create a new file that satisfies
the syntax implemented in the "missions.txt" file.

## 5.3 Failure scenario simulation ##
### Communication loss ###

In addition to performing simulations under the regular conditions, the agv_control package can also be used to simulate system behaviour in case of communication failures represented by the loss of communication on one or more vehicles (for example due to the vehicle's communication module breakdown). For the purpose of communication loss simulations, the agv_control package provides a way to completely disable communication between a desired vehicle and all the other vehicles in the system. Once disabled, the communication can be re-enabled at any time. The communication disabling and enabling is based on the use of ros services. Each agvController node advertises the "communication" service which can be used to disable/enable the communication module on a certain vehicle.

To simulate communication loss it is neccessary to start the regular simulation first (as described in chapter 5.1.). Once the simulation is started, it is possible to disable/enable communication on one or more vehicles by calling appropriate "communication" services with appropriate arguments. 

For example, to disable the communication module on the vehicle "Charlie", the following command should be executed:

	$ rosservice call Charlie/communication "disable"
		
Charlie's communication module can be enabled in a similar way, by executing the following command:

	$ rosservice call Charlie/communication "enable"
	
Disabling the communication module on a certain vehicle casues the vehicle to stop receiving trajectory information and removal requests from other vehicles as well as to stop publishing its own trajectory information and removal requests. Upon re-enabling its communication module, the vehicle automatically reestablishes all communication connections to other (functional) vehicles.
	

# 6.0 Using a custom simulator #

In order to use the agv_control package for simulations by using
a custom simulator, it is neccessary to disable the "poseEstimator" node
in the "agv_control_sim.launch" configuration file. Furthermore, the 
custom simulator should subscribe to each vehicle's 
"(vehicle name)/cmd_vel" topic to recive vehicle's velocity commands
and estimate vehicle's current pose by publishing appropriate 
transformations between the global frame and the vehicle's base frame. 
The default global frame for each vehicle is "/(vehicle name)/map",
while the default base frame is "/(vehicle name)/base_link". However, 
the frame names can be changed by editing the "global_frame_id" and 
the "base_frame_id" params within agvController node definitions 
in "agv_control_sim.launch" configuration file.
	

 
