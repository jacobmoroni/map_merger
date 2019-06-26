#ifndef MAP_MERGER_H_
#define MAP_MERGER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <iostream>
#include <string>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <rtabmap_ros/MapData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap_ros/MapsManager.h>
#include <rtabmap/core/Rtabmap.h>


namespace map_merge
{

class MapMerger
{

public:
  MapMerger();
  ~MapMerger();

protected:

  // ros stuff for this node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber map0_subscriber_;
  ros::Subscriber map1_subscriber_;
  ros::Subscriber map2_subscriber_;

  ros::Publisher  combined_map_publisher_;

  // topic names
  std::string map0_topic_;
  std::string map1_topic_;
  std::string map2_topic_;

  // callbacks
  void mapCallback0(const rtabmap_ros::MapData& msg);
  void mapCallback1(const rtabmap_ros::MapData& msg);
  void mapCallback2(const rtabmap_ros::MapData& msg);

  // data storage
  std::list<rtabmap::Signature> nodesMap0;
  std::list<rtabmap::Signature> nodesMap1;
  std::list<rtabmap::Signature> nodesMap2;

  //other functions
  void mergeMaps();
  void setupRtabParams();

  MapsManager mapsManager;
  float odomLinearVariance{0.0001f};
  float odomAngularVariance{0.0005f};

  rtabmap::Rtabmap rtabmap_;

};

} //end namespace

#endif // MAP_MERGER_H_
