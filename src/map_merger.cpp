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
#include <mutex>

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
  nh_private_.param<std::string>("combined_map_topic", combo_map_topic_, "comboMapData");

  // Other ROS params
  nh_private_.param<std::string>("base_frame", base_frame_, "world");
  nh_private_.param<std::string>("db_location", db_location_, db_location_);
  nh_private_.param<std::string>("config_path", config_path_, config_path_);
  nh_private_.param<double>("map_merge_frequency", merge_freq_, 5);
  nh_private_.param<float>("odom_linear_variance", odom_linear_variance_,0.0001f);
  nh_private_.param<float>("odom_angular_variance", odom_angular_variance_, 0.0005f);
  nh_private_.param<bool>("merge_map_optimized", merge_map_optimized_, true);
  nh_private_.param<bool>("merge_map_global", merge_map_global_, true);

  // Initialize rtabmap instance and maps manager
  setupRtabParams();
  maps_manager_.init(nh_,nh_private_,"map_merge_manager",true);
  map_to_odom_ = rtabmap::Transform::getIdentity();

  // Setup ROS hooks
  map1_subscriber_ = nh_.subscribe(map1_topic_, 1, &MapMerger::mapCallback1, this);
  map2_subscriber_ = nh_.subscribe(map2_topic_, 1, &MapMerger::mapCallback2, this);
  map3_subscriber_ = nh_.subscribe(map3_topic_, 1, &MapMerger::mapCallback3, this);
  update_map_timer_ = nh_.createTimer(ros::Duration(merge_freq_), &MapMerger::timerCallback, this);

  map_data_pub_ = nh_.advertise<rtabmap_ros::MapData>(combo_map_topic_, 1);
  std::cout<<"MAP TOPICS: "<< map1_topic_<<", "<< map2_topic_<<", "<<map3_topic_<<std::endl;
}

MapMerger::~MapMerger()
{}

void MapMerger::timerCallback(const ros::TimerEvent& msg)
{
  std::stringstream nodes_message;
  lock.lock();
  nodes_message <<"Merging Maps, Map1 Nodes: " << nodes_map1.size();
  temp_nodes_map1 = nodes_map1;
  if (map2_)
  {
    nodes_message << ", Map2 Nodes:" << nodes_map2.size();
    temp_nodes_map2 = nodes_map2;
  }
  if (map3_)
  {
    nodes_message << ", Map3 Nodes:" << nodes_map3.size();
    temp_nodes_map3 = nodes_map3;
  }
  nodes_message<<std::endl;
  std::cout << nodes_message.str();
  lock.unlock();
  mergeMaps();
}

void MapMerger::mapCallback1(const rtabmap_ros::MapData& msg)
{
  lock.lock();
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodes_map1.push_back(sig);
  lock.unlock();
}

void MapMerger::mapCallback2(const rtabmap_ros::MapData& msg)
{
  lock.lock();
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodes_map2.push_back(sig);
  lock.unlock();
}

void MapMerger::mapCallback3(const rtabmap_ros::MapData& msg)
{
  lock.lock();
  rtabmap_ros::NodeData node{msg.nodes.back()};
  rtabmap::Signature sig{rtabmap_ros::nodeDataFromROS(node)};
  nodes_map3.push_back(sig);
  lock.unlock();
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

  // Set up map booleans
  if (map1_topic_ != "none")
    map1_ = true;
  if (map2_topic_ != "none")
    map2_ = true;
  if (map3_topic_ != "none")
    map3_ = true;
}

void MapMerger::publishComboMapData()
{
  std::map<int, rtabmap::Transform> poses_md;
  std::multimap<int, rtabmap::Link> constraints_md;
  std::map<int, rtabmap::Signature> signatures_md;

  // Set _md variables from map date created in mergeMaps()
  rtabmap_.get3DMap(signatures_md,
                    poses_md,
                    constraints_md,
                    merge_map_optimized_,
                    merge_map_global_);

  // setup mapdata message
  rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
  ros::Time now = ros::Time::now();
  msg->header.stamp = now;
  msg->header.frame_id = base_frame_;

  // push mapdata to ros message
  rtabmap_ros::mapDataToROS(poses_md,
                            constraints_md,
                            signatures_md,
                            map_to_odom_,
                            *msg);

  // publish message
  map_data_pub_.publish(msg);
}

void MapMerger::mergeMaps()
{
  // Merging request, process each map one after the other

  if (map1_)
  {
    for(std::list<rtabmap::Signature>::iterator iter=temp_nodes_map1.begin(); iter!=temp_nodes_map1.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose() , odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 1: "<< temp_nodes_map1.size()<<" nodes in map."<<std::endl;
    if (map2_ | map3_)
      std::cout<<"Triggering new map"<<std::endl;
      rtabmap_.triggerNewMap(); // start a new session for the second robot
  }

  if (map2_)
  {
    for(std::list<rtabmap::Signature>::iterator iter=temp_nodes_map2.begin(); iter!=temp_nodes_map2.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 2: "<< temp_nodes_map2.size()<<" nodes in map."<<std::endl;
    if (map3_)
      std::cout<<"Triggering new map"<<std::endl;
      rtabmap_.triggerNewMap(); // start a new session for the third robot
  }

  if (map3_)
  {
    for(std::list<rtabmap::Signature>::iterator iter=temp_nodes_map3.begin(); iter!=temp_nodes_map3.end();++iter)
    {
      rtabmap::SensorData data = iter->sensorData();
      data.uncompressData();
      rtabmap_.process(data, iter->getPose(), odom_linear_variance_, odom_angular_variance_);
    }
    std::cout<<"Scanned Map 3: "<< temp_nodes_map3.size()<<" nodes in map."<<std::endl;
  }

  // Update and publish combined map
  maps_manager_.clear();
  std::map<int, rtabmap::Transform> poses;
  std::multimap<int, rtabmap::Link> constraints;
  rtabmap_.getGraph(poses, constraints, true, true);
  poses = maps_manager_.updateMapCaches(poses, rtabmap_.getMemory(), false, false);

  // map is published as pointcloud2 /cloud_map connected to the base_frame_
  maps_manager_.publishMaps(poses, ros::Time::now(), base_frame_);

  // Publish mas as MapData Topic which allows for setting max ceiling height
  // and floor height to improve map readability
  publishComboMapData();
}

} // namespace map_merger
