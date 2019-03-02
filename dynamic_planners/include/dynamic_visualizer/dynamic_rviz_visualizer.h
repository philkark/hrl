#ifndef DYNAMIC_PLANNERS_DYNAMIC_RVIZ_VISUALIZER_H_
#define DYNAMIC_PLANNERS_DYNAMIC_RVIZ_VISUALIZER_H_

//ros
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tools_std/color_scale.h>

//local
#include "dynamic_height_map/dynamic_height_map.h"
#include "dynamic_footstep_planner/dynamic_footstep_planner.h"
#include "dynamic_astar_grid_planner/dynamic_astar_grid_planner.h"
#include "dynamic_planners/get_height_mapResponse.h"

namespace dynamic_planners
{

class DynamicRvizVisualizer
{
public:
  enum MapType
  {
    Height, Normals, SegmentTypes, Edges
  };

  DynamicRvizVisualizer();

  void updateDisplaySettings();

  void setDynamicHeightMap(DynamicHeightMap* dynamicHeightMap);

  void setNewGlobalMap(const GlobalMap* globalMap, const std::vector<dynamic_planners::gazebo_object> &objects);

  void setDynamicAStarGridPlanner(DynamicAStarGridPlanner* dynamicAStarGridPlanner);

  void setDynamicFootstepPlanner(DynamicFootstepPlanner* dynamicFootstepPlanner);

  void setFootstepMarkerSizes();

  void setMsgPoses(const geometry_msgs::Pose &pose);

  void updateMap();

  void updateMapGlobal();

  void updateAStarPathGlobal();

  void updateAStarPathLocal();

  void updateFootstepPlan();

private:
  DynamicHeightMap* heightMap;
  DynamicAStarGridPlanner* aStarGridPlanner;
  DynamicFootstepPlanner* footstepPlanner;

  MapType type;
  ColorScale colorScale;
  Real minZ, maxZ;

  ros::NodeHandle nh;
  ros::Publisher publisherMap;
  ros::Publisher publisherMapGlobal;
  ros::Publisher publisherAStarPathGlobal;
  ros::Publisher publisherAStarPathLocal;
  ros::Publisher publisherFootstepPlan;

  visualization_msgs::Marker msgMap;
  visualization_msgs::MarkerArray msgMapGlobal;
  visualization_msgs::MarkerArray msgMapGlobalObjects;
  visualization_msgs::Marker msgAStarPathLocal;
  visualization_msgs::Marker msgAStarPathGlobal;
  visualization_msgs::MarkerArray msgFootstepPlan;
  std_msgs::ColorRGBA colorRightFoot;
  std_msgs::ColorRGBA colorLeftFoot;


  UInt subscriberCountGlobalMap;

  void initialize();

  void initializeMessageHandling();

  void initializeMsgMap();

  void initializeMsgMapGlobalObjects(const GlobalMap &globalMap, const std::vector<dynamic_planners::gazebo_object> &objects);

  void addMarkerSphere(const dynamic_planners::gazebo_object &object);

  void addMarkerCylinder(const dynamic_planners::gazebo_object &object);

  void addMarkerCuboid(const dynamic_planners::gazebo_object &object);

  void initializeMsgAStarPathGlobal();

  void initializeMsgAStarPathLocal();

  void initializeMsgFootstepPlan();

  void publishHeightMap(const PointMap &pointMap);

  void publishNormalMap(const NormalMap &normalMap, const PointMap &pointMap);

  void publishSegmentTypeMap(const SegmentTypeMap &segmentTypeMap, const PointMap &pointMap);

  void publishEdgeMap(const EdgeMap &edgeMap, const PointMap &pointMap);

  void publishMapDelete();

  void publishMapGlobal();

  void publishMapGlobalDelete();

  void publishAStarPathGlobal();

  void publishAStarPathGlobalDelete();

  void publishAStarPathLocal();

  void publishAStarPathLocalDelete();

  void publishFootstepPlan();

  void publishFootstepPlanDelete();

  void reducePoseToXYYaw(const geometry_msgs::Pose &in, geometry_msgs::Pose &out);
};

} //namespace dynamic_planners

#endif // DYNAMIC_PLANNERS_DYNAMIC_RVIZ_VISUALIZER_H_
