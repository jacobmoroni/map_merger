/*!
 * \file package_template_node.cpp
 * \author Daniel Koch
 *
 * \brief This file provides the entry point for the package_template node
 *
 * You will create one of these files for each node in your package. The file should be named <node name>_node.cpp. In
 * general you will want to keep this file as short as possible and do as much of the work as possible within a separate
 * class. A minimal <node name>_node.cpp file will generally have a main function that does the following:
 *   1. Initialize the node
 *   2. Create a NodeHandle for the node
 *   3. Instantiate an object of the main class for your node
 *   4. Spin
 */

#include <ros/ros.h>
#include "map_merger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merger_node");
  ros::NodeHandle nh;

  map_merge::MapMerger map_merge;
  
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
