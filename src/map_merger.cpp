/*
 * \file waypoint_manager.cpp
 * \author Jacob Olson
 *
 * \brief This is the implementation file for the WaypointManager class
 */

#include "map_merger.h"
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <map>
#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/NodeData.h>
#include <rtabmap_ros/MapsManager.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Rtabmap.h>

#include "rtabmap_ros/MsgConversion.h"
#include "rtabmap_ros/MapsManager.h"
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>

namespace map_merge
{
  MapMerger::MapMerger():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{

  // Topic Params
  //robot_listener_ = new tf::TransformListener();
  nh_private_.param<std::string>("map0_topic", map0_topic_, "none");
  nh_private_.param<std::string>("map1_topic", map1_topic_, "none");
  nh_private_.param<std::string>("map2_topic", map2_topic_, "none");

  // Setup ROS hooks
  map0_subscriber_ = nh_.subscribe(map0_topic_, 1, &MapMerger::mapCallback0, this);
  map1_subscriber_ = nh_.subscribe(map0_topic_, 1, &MapMerger::mapCallback1, this);
  map2_subscriber_ = nh_.subscribe(map0_topic_, 1, &MapMerger::mapCallback2, this);



  // initialize static members of outgoing command
}

MapMerger::~MapMerger()
{}

void MapMerger::mapCallback0(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap0.push_back(sig);
}

void MapMerger::mapCallback1(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap1.push_back(sig);
}

void MapMerger::mapCallback2(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap2.push_back(sig);
}

void MapMerger::mergeMaps()
{
  // Merging request (a service or done each 30 sec), process each map one after the other
  //
  std::string configPath;
  nh_private_.param("config_path", configPath, configPath);

  //parameters
  rtabmap::ParametersMap parameters;
  uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
  uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
  if(!configPath.empty())
  {
    if(UFile::exists(configPath.c_str()))
    {
      ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), configPath.c_str());
      rtabmap::ParametersMap allParameters;
      rtabmap::Parameters::readINI(configPath.c_str(), allParameters);
      // only update odometry parameters
      for(rtabmap::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
      {
        rtabmap::ParametersMap::iterator jter = allParameters.find(iter->first);
        if(jter!=allParameters.end())
        {
          iter->second = jter->second;
        }
      }
    }
    else
    {
      ROS_ERROR( "Config file \"%s\" not found!", configPath.c_str());
    }
  }
//  rtabmap::ParametersMap parameters = rtabmap::Parameters::getDefaultParameters();
  // See example from MapAssembler to get all parameters from rosparam or arguments

  // Keep everything in RAM
  parameters.at(rtabmap::Parameters::kDbSqlite3InMemory()) = "true";

//  rtabmap::Rtabmap rtabmap(parameters...);

//  rtabmap::Rtabmap rtabmap(parameters...);
  rtabmap::Rtabmap rtabmap;
  if (map0_topic_ != "none")
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap0.begin(); iter!=nodesMap0.end();++iter)
    {
       rtabmap::SensorData data = iter->sensorData();
       data.uncompressData();
       rtabmap.process(data, iter->getPose() , odomLinearVariance, odomAngularVariance);
    }
    rtabmap.triggerNewMap(); // start a new session for the second robot
  }

  if (map1_topic_ != "none")
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap1.begin(); iter!=nodesMap1.end();++iter)
    {
       rtabmap::SensorData data = iter->sensorData();
       data.uncompressData();
       rtabmap.process(data, iter->getPose(), odomLinearVariance, odomAngularVariance);
    }
    rtabmap.triggerNewMap(); // start a new session for the second robot
  }

  if (map2_topic_ != "none")
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap2.begin(); iter!=nodesMap2.end();++iter)
    {
       rtabmap::SensorData data = iter->sensorData();
       data.uncompressData();
       rtabmap.process(data, iter->getPose(), odomLinearVariance, odomAngularVariance);
    }
  }
  // Update and publish combined map
  mapsManager.clear();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> constraints;
  rtabmap. getGraph(poses, constraints, true, true);
  poses = mapsManager.updateMapCaches(poses, rtabmap.getMemory(), false, false);
  mapsManager.publishMaps(poses, ros::Time::now(), "map_combined");

}

} // namespace map_merger


