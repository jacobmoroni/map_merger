#ifndef MAP_MERGER_H_
#define MAP_MERGER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <vector>
#include <iostream>
#include <string>
#include <mutex>

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

  ros::Subscriber map1_subscriber_;
  ros::Subscriber map2_subscriber_;
  ros::Subscriber map3_subscriber_;
  ros::Subscriber map4_subscriber_;
  
  ros::Publisher map_data_pub_;

  ros::Timer update_map_timer_;

  std::mutex lock;

  // topic names
  std::string map1_topic_;
  std::string map2_topic_;
  std::string map3_topic_;
  std::string map4_topic_;
  std::string combo_map_topic_;

  // ros parameters
  double merge_freq_;
  float odom_linear_variance_{0.0001f};
  float odom_angular_variance_{0.0005f};
  std::string config_path_;
  std::string db_location_;
  std::string base_frame_;
  bool merge_map_optimized_;
  bool merge_map_global_;

  // callbacks
  void mapCallback1(const rtabmap_ros::MapData& msg);
  void mapCallback2(const rtabmap_ros::MapData& msg);
  void mapCallback3(const rtabmap_ros::MapData& msg);
  void mapCallback4(const rtabmap_ros::MapData& msg);
  void timerCallback(const ros::TimerEvent& event);

  // data storage
  std::list<rtabmap::Signature> nodes_map1;
  std::list<rtabmap::Signature> nodes_map2;
  std::list<rtabmap::Signature> nodes_map3;  
  std::list<rtabmap::Signature> nodes_map4;
  std::list<rtabmap::Signature> temp_nodes_map1;
  std::list<rtabmap::Signature> temp_nodes_map2;
  std::list<rtabmap::Signature> temp_nodes_map3;
  std::list<rtabmap::Signature> temp_nodes_map4;
  int map1_len_{0};
  int map2_len_{0};
  int map3_len_{0};
  int map4_len_{0};

  // other functions
  void mergeMaps();
  void setupRtabParams();
  void publishComboMapData();

  // member classes
  MapsManager maps_manager_;
  rtabmap::Rtabmap rtabmap_;
  rtabmap::Transform map_to_odom_;

  // Map Booleans
  bool map1_{false};
  bool map2_{false};
  bool map3_{false};
  bool map4_{false};
  bool update_now_{false};
};

} //end namespace

#endif // MAP_MERGER_H_
