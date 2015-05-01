#!/bin/bash

rostopic pub -r 3 -f Alfa_initialPose.yaml /Alfa/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Bravo_initialPose.yaml /Bravo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Charlie_initialPose.yaml /Charlie/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Delta_initialPose.yaml /Delta/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Echo_initialPose.yaml /Echo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -r 3 -f Foxtrot_initialPose.yaml /Foxtrot/initialpose geometry_msgs/PoseWithCovarianceStamped
