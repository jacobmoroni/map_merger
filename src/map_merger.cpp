/*
 * \file waypoint_manager.cpp
 * \author Jacob Olson
 *
 * \brief This is the implementation file for the WaypointManager class
 */

#include "map_merger.h"
#include <ros/ros.h>
#include <map>
#include <iostream>

#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/NodeData.h>
#include <rtabmap_ros/MapsManager.h>
#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/MapsManager.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>

namespace map_merge
{
  MapMerger::MapMerger():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{

  // Topic Params
  nh_private_.param<std::string>("map1_topic", map1_topic_, "none");
  nh_private_.param<std::string>("map2_topic", map2_topic_, "none");
  nh_private_.param<std::string>("map3_topic", map3_topic_, "none");

  // Other ROS params
  nh_private_.param<std::string>("base_frame", base_frame_, "world");
  nh_private_.param<std::string>("db_location", db_location_, db_location_);
  nh_private_.param<std::string>("config_path", config_path_, config_path_);
  nh_private_.param<int>("map_merge_frequency", merge_freq_, 5);
  nh_private_.param<float>("odom_linear_variance", odom_linear_variance_,0.0001f);
  nh_private_.param<float>("odom_angular_variance", odom_angular_variance_, 0.0005f);

  // Initialize rtabmap instance and maps manager
  setupRtabParams();
  maps_manager_.init(nh_,nh_private_,"map_merge_manager",true);

  // Setup ROS hooks
  map1_subscriber_ = nh_.subscribe(map1_topic_, 1, &MapMerger::mapCallback1, this);
  map2_subscriber_ = nh_.subscribe(map2_topic_, 1, &MapMerger::mapCallback2, this);
  map3_subscriber_ = nh_.subscribe(map3_topic_, 1, &MapMerger::mapCallback3, this);
  std::cout<<"MAP TOPICS: "<< map1_topic_<<", "<< map2_topic_<<", "<<map3_topic_<<std::endl;
}

MapMerger::~MapMerger()
{}

void MapMerger::mapCallback1(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  std::cout<<"MapData1 len: "<<msg.nodes.size()<<std::endl;
  std::cout<<"nodesMap1 len: "<<nodesMap1.size()<<std::endl;
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap1.push_back(sig);

  // merge maps every merge_freq_ messages that come in (messages come at 1hz by default)
  if (nodesMap1.size()%static_cast<unsigned long>(merge_freq_) == 0)
  {
    mergeMaps();
  }
}

void MapMerger::mapCallback2(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap2.push_back(sig);
}

void MapMerger::mapCallback3(const rtabmap_ros::MapData& msg)
{
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodesMap3.push_back(sig);
}

void MapMerger::setupRtabParams()
{
  //parameters
  rtabmap::ParametersMap parameters;
  uInsert(parameters, rtabmap::Parameters::getDefaultParameters("Grid"));
  uInsert(parameters, rtabmap::Parameters::getDefaultParameters("StereoBM"));
  if(!config_path_.empty())
  {
    if(UFile::exists(config_path_.c_str()))
    {
      ROS_INFO( "%s: Loading parameters from %s", ros::this_node::getName().c_str(), config_path_.c_str());
      rtabmap::ParametersMap allParameters;
      rtabmap::Parameters::readINI(config_path_.c_str(), allParameters);
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
      ROS_ERROR( "Config file \"%s\" not found!", config_path_.c_str());
    }
  }
  else
  {
    parameters = rtabmap::Parameters::getDefaultParameters();
  }

  // Keep everything in RAM
  parameters.at(rtabmap::Parameters::kDbSqlite3InMemory()) = "true";
  rtabmap_.init(parameters,db_location_);
}

void MapMerger::mergeMaps()
{
  bool map1{false};
  bool map2{false};
  bool map3{false};
  if (map1_topic_ != "none")
    map1 = true;
  if (map2_topic_ != "none")
    map2 = true;
  if (map3_topic_ != "none")
    map3 = true;

  // Merging request, process each map one after the other

  if (map1)
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap1.begin(); iter!=nodesMap1.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose() , odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 1: "<< nodesMap1.size()<<" nodes in map."<<std::endl;
    if (map2 | map3)
      std::cout<<"Triggering new map"<<std::endl;
      rtabmap_.triggerNewMap(); // start a new session for the second robot
  }

  if (map2)
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap2.begin(); iter!=nodesMap2.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 2: "<< nodesMap2.size()<<" nodes in map."<<std::endl;
    if (map3)
      std::cout<<"Triggering new map"<<std::endl;
      rtabmap_.triggerNewMap(); // start a new session for the third robot
  }

  if (map3)
  {
    for(std::list<rtabmap::Signature>::iterator iter=nodesMap3.begin(); iter!=nodesMap3.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 3: "<< nodesMap3.size()<<" nodes in map."<<std::endl;
  }

  // Update and publish combined map
  maps_manager_.clear();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> constraints;
  rtabmap_.getGraph(poses, constraints, true, true);
  poses = maps_manager_.updateMapCaches(poses, rtabmap_.getMemory(), false, false);

  // map is published as pointcloud2 /cloud_map connected to the base_frame_
  maps_manager_.publishMaps(poses, ros::Time::now(), base_frame_);
}

} // namespace map_merger


