//std
#include <vector>
#include <string>

//ros
#include <tools_std/ros_param_loader.h>

//local
#include "dynamic_visualizer/dynamic_rviz_visualizer.h"

namespace dynamic_planners
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

DynamicRvizVisualizer::DynamicRvizVisualizer() :
    colorScale(ColorScale::MColor), heightMap(nullptr), aStarGridPlanner(nullptr), subscriberCountGlobalMap(0), footstepPlanner(nullptr)
{
  initialize();
}

void DynamicRvizVisualizer::updateDisplaySettings()
{
  Real tmpReal;
  std::string tmpString;

  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "displayed_map_type", tmpString, "edge");
  if (tmpString == "height")
    type = DynamicRvizVisualizer::MapType::Height;
  else if (tmpString == "normal")
    type = DynamicRvizVisualizer::MapType::Normals;
  else if (tmpString == "segment")
    type = DynamicRvizVisualizer::MapType::SegmentTypes;
  else if (tmpString == "edge")
    type = DynamicRvizVisualizer::MapType::Edges;

  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_global_radius", tmpReal, 0.02);
  msgAStarPathGlobal.scale.x = msgAStarPathGlobal.scale.y = msgAStarPathGlobal.scale.z = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_global_r", tmpReal, 1.0);
  msgAStarPathGlobal.color.r = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_global_g", tmpReal, 1.0);
  msgAStarPathGlobal.color.g = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_global_b", tmpReal, 1.0);
  msgAStarPathGlobal.color.b = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_global_a", tmpReal, 1.0);
  msgAStarPathGlobal.color.a = tmpReal;

  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_local_radius", tmpReal, 0.02);
  msgAStarPathLocal.scale.x = msgAStarPathLocal.scale.y = msgAStarPathLocal.scale.z = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_local_r", tmpReal, 1.0);
  msgAStarPathLocal.color.r = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_local_g", tmpReal, 1.0);
  msgAStarPathLocal.color.g = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_local_b", tmpReal, 1.0);
  msgAStarPathLocal.color.b = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_local_a", tmpReal, 1.0);
  msgAStarPathLocal.color.a = tmpReal;

  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_r_r", tmpReal, 1.0);
  colorRightFoot.r = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_r_g", tmpReal, 1.0);
  colorRightFoot.g = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_r_b", tmpReal, 1.0);
  colorRightFoot.b = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_r_a", tmpReal, 1.0);
  colorRightFoot.a = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_l_r", tmpReal, 1.0);
  colorLeftFoot.r = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_l_g", tmpReal, 1.0);
  colorLeftFoot.g = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_l_b", tmpReal, 1.0);
  colorLeftFoot.b = tmpReal;
  RosParamLoader::getRosParam(nh, "DynamicRvizVisualizer::updateDisplaySettings", "display_foot_l_a", tmpReal, 1.0);
  colorLeftFoot.a = tmpReal;

}

void DynamicRvizVisualizer::setDynamicHeightMap(DynamicHeightMap* dynamicHeightMap)
{
  heightMap = dynamicHeightMap;
  initializeMsgMap();
}

void DynamicRvizVisualizer::setNewGlobalMap(const GlobalMap* globalMap, const std::vector<dynamic_planners::gazebo_object> &objects)
{
  initializeMsgMapGlobalObjects(*globalMap, objects);
  subscriberCountGlobalMap = 0;
}

void DynamicRvizVisualizer::setDynamicAStarGridPlanner(DynamicAStarGridPlanner* dynamicAStarGridPlanner)
{
  aStarGridPlanner = dynamicAStarGridPlanner;
}

void DynamicRvizVisualizer::setDynamicFootstepPlanner(DynamicFootstepPlanner* dynamicFootstepPlanner)
{
  footstepPlanner = dynamicFootstepPlanner;
}

void DynamicRvizVisualizer::setFootstepMarkerSizes()
{
  for (UInt i = 0; i < msgFootstepPlan.markers.size(); ++i)
  {
    msgFootstepPlan.markers[i].scale.x = DynamicFootstepPlannerConfig::sizeFront + DynamicFootstepPlannerConfig::sizeBack;
    msgFootstepPlan.markers[i].scale.y = DynamicFootstepPlannerConfig::sizeInner + DynamicFootstepPlannerConfig::sizeOuter;
  }
}

void DynamicRvizVisualizer::setMsgPoses(const geometry_msgs::Pose &pose)
{
  //msgMap.pose = pose;
//  for(UInt i = 0; i < msgFootstepPlan.markers.size(); ++i)
//    msgFootstepPlan.markers[i].pose = pose;

  reducePoseToXYYaw(pose, msgAStarPathLocal.pose);
}

void DynamicRvizVisualizer::updateMap()
{
  if (heightMap == nullptr)
    return;

  switch (type)
  {
    case MapType::Height:
      publishHeightMap(heightMap->getPointMapConst(0u));
      break;
    case MapType::Normals:
      publishNormalMap(heightMap->getNormalMapConst(), heightMap->getPointMapConst(0u));
      break;
    case MapType::SegmentTypes:
      publishSegmentTypeMap(heightMap->getSegmentTypeMapConst(0u), heightMap->getPointMapConst(0u));
      break;
    case MapType::Edges:
      publishEdgeMap(heightMap->getEdgeMapConst(0u), heightMap->getPointMapConst(0u));
      break;
  }
}

void DynamicRvizVisualizer::updateMapGlobal()
{
  publishMapGlobal();
}

void DynamicRvizVisualizer::updateAStarPathGlobal()
{
  const AStarPath2D &AStarPath = aStarGridPlanner->getAStarPathGlobal();
  if (aStarGridPlanner == nullptr || !AStarPath.valid)
    publishAStarPathGlobalDelete();
  else
    publishAStarPathGlobal();
}

void DynamicRvizVisualizer::updateAStarPathLocal()
{
  const AStarPath2D &AStarPath = aStarGridPlanner->getAStarPathLocal();
  if (aStarGridPlanner == nullptr || !AStarPath.valid)
    publishAStarPathLocalDelete();
  else
    publishAStarPathLocal();
}

void DynamicRvizVisualizer::updateFootstepPlan()
{
  if (footstepPlanner == nullptr || !footstepPlanner->isPlanValid())
    publishFootstepPlanDelete();
  else
    publishFootstepPlan();
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void DynamicRvizVisualizer::initialize()
{
  initializeMessageHandling();
  initializeMsgAStarPathGlobal();
  initializeMsgAStarPathLocal();
  initializeMsgFootstepPlan();

  minZ = -1.0;
  maxZ = 0.0;
}

void DynamicRvizVisualizer::initializeMessageHandling()
{
  publisherMap = nh.advertise<visualization_msgs::Marker>("dynamic_map", 0);
  publisherMapGlobal = nh.advertise<visualization_msgs::MarkerArray>("global_map", 0);
  publisherAStarPathGlobal = nh.advertise<visualization_msgs::Marker>("astar_grid_path_global", 0);
  publisherAStarPathLocal = nh.advertise<visualization_msgs::Marker>("astar_grid_path_local", 0);
  publisherFootstepPlan = nh.advertise<visualization_msgs::MarkerArray>("footstep_plan", 0);
}

void DynamicRvizVisualizer::initializeMsgMap()
{
  msgMap.header.frame_id = "base_link";
  msgMap.id = 97;
  msgMap.type = visualization_msgs::Marker::CUBE_LIST;
  msgMap.scale.x = DynamicHeightMapConfig::resolutionSpace;
  msgMap.scale.y = DynamicHeightMapConfig::resolutionSpace;
  msgMap.scale.z = 0.001;
  msgMap.pose.position.x = msgMap.pose.position.y = msgMap.pose.position.z = 0.0;
  msgMap.pose.orientation.x = msgMap.pose.orientation.y = msgMap.pose.orientation.z = 0.0;
  msgMap.pose.orientation.w = 1.0;

  msgMap.points.resize(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);
  msgMap.colors.resize(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);
}

void DynamicRvizVisualizer::initializeMsgMapGlobalObjects(const GlobalMap &globalMap, const std::vector<dynamic_planners::gazebo_object> &objects)
{
  msgMapGlobalObjects.markers.resize(1);
  msgMapGlobalObjects.markers[0].header.frame_id = "map";
  msgMapGlobalObjects.markers[0].id = 250;
  msgMapGlobalObjects.markers[0].type = visualization_msgs::Marker::CUBE;
  msgMapGlobalObjects.markers[0].action = visualization_msgs::Marker::ADD;
  msgMapGlobalObjects.markers[0].scale.x = globalMap.maxX - globalMap.minX;
  msgMapGlobalObjects.markers[0].scale.y = globalMap.maxY - globalMap.minY;
  msgMapGlobalObjects.markers[0].scale.z = 0.001;
  msgMapGlobalObjects.markers[0].pose.position.x = 0.5 * (globalMap.maxX + globalMap.minX);
  msgMapGlobalObjects.markers[0].pose.position.y = 0.5 * (globalMap.maxY + globalMap.minY);
  msgMapGlobalObjects.markers[0].pose.position.z = -0.0005;
  msgMapGlobalObjects.markers[0].pose.orientation.x = msgMap.pose.orientation.y = msgMap.pose.orientation.z = 0.0;
  msgMapGlobalObjects.markers[0].pose.orientation.w = 1.0;
  msgMapGlobalObjects.markers[0].color.a = 0.2;

  for (UInt i = 0; i < objects.size(); ++i)
  {
    if (objects[i].type == dynamic_planners::gazebo_object::sphere)
      addMarkerSphere(objects[i]);
    else if (objects[i].type == dynamic_planners::gazebo_object::cylinder)
      addMarkerCylinder(objects[i]);
    else if (objects[i].type == dynamic_planners::gazebo_object::cuboid)
      addMarkerCuboid(objects[i]);
  }
}

void DynamicRvizVisualizer::addMarkerSphere(const dynamic_planners::gazebo_object &object)
{
  msgMapGlobalObjects.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker &marker = msgMapGlobalObjects.markers.back();
  marker.id = msgMapGlobalObjects.markers[msgMapGlobalObjects.markers.size() - 2].id + 1;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = object.radius;
  marker.color.a = 0.5;
  marker.color.r = marker.color.g = 0.0;
  marker.color.b = 0.2;
  marker.pose.position.x = object.pos_x;
  marker.pose.position.y = object.pos_y;
  marker.pose.position.z = object.pos_z;
  marker.pose.orientation.x = object.rot_x;
  marker.pose.orientation.y = object.rot_y;
  marker.pose.orientation.z = object.rot_z;
  marker.pose.orientation.w = object.rot_w;
}

void DynamicRvizVisualizer::addMarkerCylinder(const dynamic_planners::gazebo_object &object)
{
  msgMapGlobalObjects.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker &marker = msgMapGlobalObjects.markers.back();
  marker.id = msgMapGlobalObjects.markers[msgMapGlobalObjects.markers.size() - 2].id + 1;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = marker.scale.y = object.radius * 2;
  marker.scale.z = object.height;
  marker.color.a = 0.5;
  marker.color.r = marker.color.g = 0.0;
  marker.color.b = 0.2;
  marker.pose.position.x = object.pos_x;
  marker.pose.position.y = object.pos_y;
  marker.pose.position.z = object.pos_z;
  marker.pose.orientation.x = object.rot_x;
  marker.pose.orientation.y = object.rot_y;
  marker.pose.orientation.z = object.rot_z;
  marker.pose.orientation.w = object.rot_w;
}

void DynamicRvizVisualizer::addMarkerCuboid(const dynamic_planners::gazebo_object &object)
{
  msgMapGlobalObjects.markers.push_back(visualization_msgs::Marker());
  visualization_msgs::Marker &marker = msgMapGlobalObjects.markers.back();
  marker.id = msgMapGlobalObjects.markers[msgMapGlobalObjects.markers.size() - 2].id + 1;
  marker.header.frame_id = "map";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = object.length;
  marker.scale.y = object.width;
  marker.scale.z = object.height;
  marker.color.a = 0.5;
  marker.color.r = marker.color.g = 0.0;
  marker.color.b = 0.2;
  marker.pose.position.x = object.pos_x;
  marker.pose.position.y = object.pos_y;
  marker.pose.position.z = object.pos_z;
  marker.pose.orientation.x = object.rot_x;
  marker.pose.orientation.y = object.rot_y;
  marker.pose.orientation.z = object.rot_z;
  marker.pose.orientation.w = object.rot_w;
}

void DynamicRvizVisualizer::initializeMsgAStarPathGlobal()
{
  msgAStarPathGlobal.header.frame_id = "map";
  msgAStarPathGlobal.id = 99;
  msgAStarPathGlobal.type = visualization_msgs::Marker::SPHERE_LIST;
  msgAStarPathGlobal.scale.x = 0.04;
  msgAStarPathGlobal.scale.y = 0.04;
  msgAStarPathGlobal.scale.z = 0.04;
  msgAStarPathGlobal.pose.orientation.w = 1.0;
}

void DynamicRvizVisualizer::initializeMsgAStarPathLocal()
{
  msgAStarPathLocal.header.frame_id = "map";
  msgAStarPathLocal.id = 98;
  msgAStarPathLocal.type = visualization_msgs::Marker::SPHERE_LIST;
  msgAStarPathLocal.scale.x = 0.04;
  msgAStarPathLocal.scale.y = 0.04;
  msgAStarPathLocal.scale.z = 0.04;
  msgAStarPathLocal.pose.orientation.w = 1.0;
}

void DynamicRvizVisualizer::initializeMsgFootstepPlan()
{
  visualization_msgs::Marker markerSteps;
  markerSteps.header.frame_id = "base_link";
  markerSteps.action = visualization_msgs::Marker::ADD;
  markerSteps.type = visualization_msgs::Marker::CUBE;
  markerSteps.ns = "steps";
  markerSteps.pose.orientation.w = 1.;
  markerSteps.scale.z = 0.04;

  for (UInt i = 100; i < 200; ++i)
  {
    markerSteps.id = i;
    markerSteps.color.a = 1.0;
    markerSteps.color.r = markerSteps.color.g = markerSteps.color.b = 0.5;
    msgFootstepPlan.markers.push_back(markerSteps);
  }
}

void DynamicRvizVisualizer::publishHeightMap(const PointMap &pointMap)
{
  if (publisherMap.getNumSubscribers() == 0)
    return;

  msgMap.header.stamp = ros::Time::now();
  msgMap.action = visualization_msgs::Marker::ADD;

  const Real colorHeightSlope = 0.3 / (maxZ - minZ);

  UInt pointIndex = 0;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      geometry_msgs::Point &point = msgMap.points[pointIndex];
      std_msgs::ColorRGBA &color = msgMap.colors[pointIndex];
      ++pointIndex;

      point.x = pointMap[xIndex][yIndex].C0;
      point.y = pointMap[xIndex][yIndex].C1;
      point.z = pointMap[xIndex][yIndex].C2;

      Real colorHeightValue = point.z;
      if (colorHeightValue < minZ)
        colorHeightValue = minZ;

      if (colorHeightValue > maxZ)
        colorHeightValue = maxZ;

      if (pointMap[xIndex][yIndex] == HEIGHT_UNKNOWN)
      {
        color.a = 1.0;
        point.z = 1000000.0;
      }
      else
      {
        color.a = 1.0;
        color.r = color.g = color.b = 0.4 + (colorHeightValue - minZ) * colorHeightSlope;
      }

    }

  publisherMap.publish(msgMap);
}

void DynamicRvizVisualizer::publishNormalMap(const NormalMap &normalMap, const PointMap &pointMap)
{
  if (publisherMap.getNumSubscribers() == 0)
    return;

  msgMap.header.stamp = ros::Time::now();
  msgMap.action = visualization_msgs::Marker::ADD;

  UInt pointIndex = 0;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      geometry_msgs::Point &point = msgMap.points[pointIndex];
      std_msgs::ColorRGBA &color = msgMap.colors[pointIndex];
      ++pointIndex;
      if (pointMap[xIndex][yIndex].C2 == HEIGHT_UNKNOWN)
      {
        color.a = 1.0;
        point.z = 1000000.0;
        continue;
      }

      point.x = pointMap[xIndex][yIndex].C0;
      point.y = pointMap[xIndex][yIndex].C1;
      point.z = pointMap[xIndex][yIndex].C2 - DynamicHeightMapConfig::resolutionSpace * 0.5;

      if (xIndex == 0 || yIndex == 0 || xIndex == DynamicHeightMapConfig::sizeX - 1 || yIndex == DynamicHeightMapConfig::sizeY - 1)
      {
        color.r = 1.0;
        color.g = 0;
        color.b = 0;
      }
      else
      {
        Real cosVal = 1;
        if (normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex - 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex - 1];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex + 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex - 1][yIndex + 1];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex][yIndex - 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex][yIndex - 1];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex][yIndex + 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex][yIndex + 1];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex - 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex - 1];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex];
        if (normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex + 1] < cosVal)
          cosVal = normalMap[xIndex][yIndex] * normalMap[xIndex + 1][yIndex + 1];

        const Color &colorTmp = colorScale.getColor(1.0 - acos(cosVal));
        color.r = colorTmp.r;
        color.g = colorTmp.g;
        color.b = colorTmp.b;
      }
    }

  publisherMap.publish(msgMap);
}

void DynamicRvizVisualizer::publishSegmentTypeMap(const SegmentTypeMap &segmentTypeMap, const PointMap &pointMap)
{
  if (publisherMap.getNumSubscribers() == 0)
    return;

  msgMap.header.stamp = ros::Time::now();
  msgMap.action = visualization_msgs::Marker::ADD;

  const Real colorHeightSlopeR = 0.15 / (maxZ - minZ);
  const Real colorHeightSlopeG = 0.3 / (maxZ - minZ);
  const Real colorHeightSlopeB = 0.15 / (maxZ - minZ);

  UInt pointIndex = 0;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      geometry_msgs::Point &point = msgMap.points[pointIndex];
      std_msgs::ColorRGBA &color = msgMap.colors[pointIndex];
      ++pointIndex;

      if (pointMap[xIndex][yIndex].C2 == HEIGHT_UNKNOWN)
      {
        color.a = 1.0;
        point.z = 1000000.0;
        continue;
      }

      point.x = pointMap[xIndex][yIndex].C0;
      point.y = pointMap[xIndex][yIndex].C1;
      point.z = pointMap[xIndex][yIndex].C2 - DynamicHeightMapConfig::resolutionSpace * 0.5;

      Real colorHeightValue = point.z;
      if (colorHeightValue < minZ)
        colorHeightValue = minZ;

      if (colorHeightValue > maxZ)
        colorHeightValue = maxZ;

      if (segmentTypeMap[xIndex][yIndex] == SegmentType::Planar)
      {
        color.a = 1.0;
        color.r = 0.1 + (colorHeightValue - minZ) * colorHeightSlopeR;
        color.g = 0.6 + (colorHeightValue - minZ) * colorHeightSlopeG;
        color.b = 0.5 + (colorHeightValue - minZ) * colorHeightSlopeB;
      }
      else if (segmentTypeMap[xIndex][yIndex] == SegmentType::Nonplanar)
      {
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.6;
        color.b = 0.0;
      }
      else if (segmentTypeMap[xIndex][yIndex] == SegmentType::None)
      {
        color.a = 1.0;
        color.r = 0.3;
        color.g = 0.3;
        color.b = 0.3;
      }

    }

  publisherMap.publish(msgMap);
}

void DynamicRvizVisualizer::publishEdgeMap(const EdgeMap &edgeMap, const PointMap &pointMap)
{
  if (publisherMap.getNumSubscribers() == 0)
    return;

  msgMap.header.stamp = ros::Time::now();
  msgMap.action = visualization_msgs::Marker::ADD;

  UInt pointIndex = 0;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      geometry_msgs::Point &point = msgMap.points[pointIndex];
      std_msgs::ColorRGBA &color = msgMap.colors[pointIndex];
      ++pointIndex;

      if (pointMap[xIndex][yIndex].C2 == HEIGHT_UNKNOWN)
      {
        color.a = 1.0;
        point.z = 1000000.0;
        continue;
      }

      point.x = pointMap[xIndex][yIndex].C0;
      point.y = pointMap[xIndex][yIndex].C1;
      point.z = pointMap[xIndex][yIndex].C2 - DynamicHeightMapConfig::resolutionSpace * 0.5;

      if (edgeMap[xIndex][yIndex] == -1)
      {
        color.a = 1.0;
        color.r = 0.6;
        color.g = 0.6;
        color.b = 0.6;

      }
      else if (edgeMap[xIndex][yIndex] == 0)
      {
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
      }
      else
      {
        color.a = 1.0;
        color.r = 1.0;
        color.g = 0.6;
        color.b = 0.6;
      }
    }

  publisherMap.publish(msgMap);
}

void DynamicRvizVisualizer::publishMapDelete()
{
  msgMap.action = visualization_msgs::Marker::DELETE;
  publisherMap.publish(msgMap);
}

void DynamicRvizVisualizer::publishMapGlobal()
{
  bool publish = false;
  UInt newSubscriberCount = publisherMapGlobal.getNumSubscribers();

  if (newSubscriberCount > subscriberCountGlobalMap)
    publish = true;

  subscriberCountGlobalMap = newSubscriberCount;

  if (!publish)
    return;

  const ros::Time timeNow = ros::Time::now();

  for(UInt i = 0; i< msgMapGlobalObjects.markers.size(); ++i)
  {
    msgMapGlobalObjects.markers[i].header.stamp = timeNow;
    msgMapGlobalObjects.markers[i].action = visualization_msgs::Marker::ADD;
  }

  publisherMapGlobal.publish(msgMapGlobalObjects);
}

void DynamicRvizVisualizer::publishMapGlobalDelete()
{
  for(UInt i = 0; i< msgMapGlobalObjects.markers.size(); ++i)
    msgMapGlobalObjects.markers[i].action = visualization_msgs::Marker::DELETE;

  publisherMapGlobal.publish(msgMapGlobalObjects);
}

void DynamicRvizVisualizer::publishAStarPathGlobal()
{
  if (publisherAStarPathGlobal.getNumSubscribers() == 0)
    return;

  const AStarPath2D &AStarPath = aStarGridPlanner->getAStarPathGlobal();
  msgAStarPathGlobal.header.stamp = ros::Time::now();
  msgAStarPathGlobal.action = visualization_msgs::Marker::ADD;
  msgAStarPathGlobal.points.clear();

  for (UInt i = 0; i < AStarPath.pointsSmoothed.size(); ++i)
  {
    msgAStarPathGlobal.points.push_back(geometry_msgs::Point());
    msgAStarPathGlobal.points.back().x = AStarPath.pointsSmoothed[i].C0;
    msgAStarPathGlobal.points.back().y = AStarPath.pointsSmoothed[i].C1;
    msgAStarPathGlobal.points.back().z = DynamicHeightMapConfig::resolutionSpace * 0.5;
  }

  publisherAStarPathGlobal.publish(msgAStarPathGlobal);
}

void DynamicRvizVisualizer::publishAStarPathGlobalDelete()
{
  msgAStarPathGlobal.action = visualization_msgs::Marker::DELETE;
  publisherAStarPathGlobal.publish(msgAStarPathGlobal);
}

void DynamicRvizVisualizer::publishAStarPathLocal()
{
  const AStarPath2D &AStarPath = aStarGridPlanner->getAStarPathLocal();
  if (publisherAStarPathLocal.getNumSubscribers() == 0)
    return;

  msgAStarPathLocal.header.stamp = ros::Time::now();
  msgAStarPathLocal.action = visualization_msgs::Marker::ADD;
  msgAStarPathLocal.points.clear();

  for (UInt i = 0; i < AStarPath.pointsSmoothed.size(); ++i)
  {
    msgAStarPathLocal.points.push_back(geometry_msgs::Point());
    msgAStarPathLocal.points.back().x = AStarPath.pointsSmoothed[i].C0;
    msgAStarPathLocal.points.back().y = AStarPath.pointsSmoothed[i].C1;
    msgAStarPathLocal.points.back().z = DynamicHeightMapConfig::resolutionSpace * 0.5;
  }

  publisherAStarPathLocal.publish(msgAStarPathLocal);
}

void DynamicRvizVisualizer::publishAStarPathLocalDelete()
{
  msgAStarPathLocal.action = visualization_msgs::Marker::DELETE;
  publisherAStarPathLocal.publish(msgAStarPathLocal);
}

void DynamicRvizVisualizer::publishFootstepPlan()
{
  if (publisherFootstepPlan.getNumSubscribers() == 0)
    return;

  const ros::Time time = ros::Time::now();
  const std::deque<Footstep> &footstepPlan = footstepPlanner->getFootstepPlan();

  UInt foostepIndex = 1;
  UInt markerIndex = 0;
  const Real correctionX = 0.5 * (DynamicFootstepPlannerConfig::sizeFront - DynamicFootstepPlannerConfig::sizeBack);
  const Real correctionYRight = 0.5 * (DynamicFootstepPlannerConfig::sizeInner - DynamicFootstepPlannerConfig::sizeOuter);
  const Real correctionYLeft = 0.5 * (DynamicFootstepPlannerConfig::sizeOuter - DynamicFootstepPlannerConfig::sizeInner);

  for (; foostepIndex < footstepPlan.size() && foostepIndex < msgFootstepPlan.markers.size(); ++foostepIndex, ++markerIndex)
  {
    const Footstep &footstep = footstepPlan[foostepIndex];
    visualization_msgs::Marker& markerSteps = msgFootstepPlan.markers[markerIndex];

    Real angleCos, angleSin;
    sincos(footstep.pose.C3, &angleSin, &angleCos);
    PV2D correctionStepX(angleCos, angleSin);
    PV2D correctionStepY(-angleSin, angleCos);

    correctionStepX *= correctionX;
    correctionStepY *= footstep.foot == Foot::right ? correctionYRight : correctionYLeft;

    markerSteps.header.stamp = time;
    markerSteps.action = visualization_msgs::Marker::ADD;
    markerSteps.color = footstep.foot == Foot::right ? colorRightFoot : colorLeftFoot;
    markerSteps.pose.position.x = footstep.pose.C0 + correctionStepX.C0 + correctionStepY.C0;
    markerSteps.pose.position.y = footstep.pose.C1 + correctionStepX.C1 + correctionStepY.C1;
    markerSteps.pose.position.z = footstep.pose.C2 + 0.02;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(footstep.pose.C3), markerSteps.pose.orientation);
  }

  // Publish deletion message for the remaining unused markers
  for (; markerIndex < msgFootstepPlan.markers.size(); ++markerIndex)
  {
    visualization_msgs::Marker& markerSteps = msgFootstepPlan.markers[markerIndex];
    markerSteps.header.stamp = time;
    markerSteps.action = visualization_msgs::Marker::DELETE;
  }

  // Publish the markers
  publisherFootstepPlan.publish(msgFootstepPlan);
}

void DynamicRvizVisualizer::publishFootstepPlanDelete()
{
  for (UInt i = 0; i < msgFootstepPlan.markers.size(); ++i)
    msgFootstepPlan.markers[i].action = visualization_msgs::Marker::DELETE;
  publisherFootstepPlan.publish(msgFootstepPlan);
}

void DynamicRvizVisualizer::reducePoseToXYYaw(const geometry_msgs::Pose &in, geometry_msgs::Pose &out)
{
  out.position.x = in.position.x;
  out.position.y = in.position.y;

  Real tmp, yaw;
  tf::Quaternion quat(in.orientation.x, in.orientation.y, in.orientation.z, in.orientation.w);
  tf::Matrix3x3 mat;
  mat.setRotation(quat);
  mat.getRPY(tmp, tmp, yaw);
  quat.setRPY(0.0, 0.0, yaw);

  out.orientation.x = quat[0];
  out.orientation.y = quat[1];
  out.orientation.z = quat[2];
  out.orientation.w = quat[3];
}

} //namespace dynamic_planners
