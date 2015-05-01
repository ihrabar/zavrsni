#!/bin/bash

rostopic pub -f Alfa_initialPose.yaml /Alfa/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Bravo_initialPose.yaml /Bravo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Charlie_initialPose.yaml /Charlie/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Delta_initialPose.yaml /Delta/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Echo_initialPose.yaml /Echo/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Foxtrot_initialPose.yaml /Foxtrot/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Golf_initialPose.yaml /Golf/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f India_initialPose.yaml /India/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Hotel_initialPose.yaml /Hotel/initialpose geometry_msgs/PoseWithCovarianceStamped

rostopic pub -f Juliett_initialPose.yaml /Juliett/initialpose geometry_msgs/PoseWithCovarianceStamped
