#!/bin/bash

#rostopic pub -f Alfa_homePose.yaml /Alfa/initialpose geometry_msgs/PoseWithCovarianceStamped

#rostopic pub -f Bravo_homePose.yaml /Bravo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Charlie_initialPose.yaml /Charlie/initialpose geometry_msgs/PoseWithCovarianceStamped

#rostopic pub -f Delta_homePose.yaml /Delta/initialpose geometry_msgs/PoseWithCovarianceStamped

#rostopic pub -f Echo_homePose.yaml /Echo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Foxtrot_initialPose.yaml /Foxtrot/initialpose geometry_msgs/PoseWithCovarianceStamped
