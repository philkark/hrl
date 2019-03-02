#ifndef DYNAMIC_HEIGHT_MAP_DYNAMIC_HEIGHT_MAP_H_
#define DYNAMIC_HEIGHT_MAP_DYNAMIC_HEIGHT_MAP_H_

//std
#include <vector>
#include <deque>
#include <string>
#include <iostream>
#include <fstream>

//personal
#include <math_std/math_std.h>
#include <tools_std/ros_param_loader.h>
#include <tools_std/timer.h>
#include <tools_std/ros_utils.h>
#include <tools_std/toggle.h>

//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>

//local
#include "dynamic_height_map/types.h"
#include "dynamic_height_map/cells.h"
#include "dynamic_height_map/dynamic_height_map_config.h"
#include "dynamic_height_map/objects.h"
#include "dynamic_height_map/segment.h"
#include "dynamic_height_map/simulation.h"
#include "dynamic_height_map/global_map.h"
#include "dynamic_planners/get_height_map.h"


class DynamicHeightMap
{
public:
  DynamicHeightMap();

  void updateMap(const Matrix34 &transform, const Pose6D &robotPose);

  void setGlobalMap(const GlobalMap* globalMap);

  void findRobotCellList(const Real radius);

  // ******************** GETTERS ********************

  const std::vector<PointMap> &getPointMapsConst() const;

  const std::vector<SegmentTypeMap> &getSegmentTypeMapsConst() const;

  const std::vector<EdgeMap> &getEdgeMapsConst() const;

  const std::vector<Object> &getObjectsConst() const;

  const PointMap &getPointMapConst(const Real time) const;

  const NormalMap &getNormalMapConst() const;

  const SegmentMap &getSegmentMapConst(const Real time) const;

  const SegmentTypeMap &getSegmentTypeMapConst(const Real time) const;

  const EdgeMap &getEdgeMapConst(const Real time) const;

  const PointMap &getPointMapConst(const UInt timeIndex) const;

  const SegmentMap &getSegmentMapConst(const UInt timeIndex) const;

  const SegmentTypeMap &getSegmentTypeMapConst(const UInt timeIndex) const;

  const EdgeMap &getEdgeMapConst(const UInt timeIndex) const;

private:
  //ros
  ros::NodeHandle nh;
  Int progressMessageIndex;
  ros::Subscriber subscriberDepthImage;
  ros::Publisher publisherObjectPredictions;

  //settings
  bool firstFrame;
  ros::Time timeStart;
  ros::Time timeStampCurrent;
  UInt objectIDNext;
  Pose6D robotPose;
  Cell2DList cellListRobot;

  //real camera
  std::vector<std::vector<PV3D>> unitImage;
  std::vector<std::vector<PV3D>> depthImagePointBuffer;

  //maps
  const GlobalMap* globalMap;
  HeightMap pointsInitial;
  Toggle<HeightMap> heightsFiltering;

  PointMap pointMapAnalysis;
  NormalMap normalMapAnalysis;
  SegmentMap segmentMapAnalysis;
  SegmentTypeMap segmentTypeMapAnalysis;
  EdgeMap edgeMapAnalysis;

  std::vector<PointMap> pointMapsPlanning;
  std::vector<SegmentMap> segmentMapsPlanning;
  std::vector<SegmentTypeMap> segmentTypeMapsPlanning;
  std::vector<EdgeMap> edgeMapsPlanning;

  //analysis
  std::vector<Segment> segments;
  std::vector<Cell2DList> segmentCellLists;
  std::deque<std::vector<Object> > objects;

  // ******************** INITIALIZATION ********************

  bool initialize();

  void initializeMaps();

  void initializeUnitImage();

  // ******************** MAP ANALYSIS ********************

  void filterSingleHoles();

  void filterMedian();

  void computeNormals();

  void findSegments();

  void removeRobotFromHeightMap();

  void insertGlobalMap();

  void findNewSegmentPlanar(const Cell2D &cellStart);

  void continuePlanarSegmentAsNonPlanar(const UInt segmentID);

  void findNewSegmentNonPlanar(const Cell2D &cellStart);

  void findAndReplaceShadow(const Cell2D &cellStart);

  UInt findAdjacentPlanarSegmentID(const UInt segmentID) const;

  UInt findAdjacentNonplanarSegmentID(const UInt segmentID) const;

  void findSegmentsAdjacentToEdge();

  void findSegmentTypeMap();

  void findHeightVariances();

  Real getSegmentDistanceDeviation(const Segment &segment);

  void createNewObjects();

  void findObjectMappings();

  void findMaximizingPermutation(Int index, std::vector<Int> &permutation, std::vector<Int> &permutationMax, Real &scoreMax,
                                 const std::vector<std::vector<Real> > &mappingScores, const std::vector<std::vector<UInt> > &mappingPossibilities);

  void updateObjectTrajectories();

  void updateObjectIDs();

  void extractDynamicObjects();

  void createPredictionMaps();

  void findEdgeMap();

  void inflateEdgeMap(EdgeMap &edgeMap, std::vector<Cell2D> &edgeQueue);

  bool trackObject();

  void saveTrackedTrajectory();

  // ******************** CAMERA ********************

  void getFrameFromDepthImageBuffer(const Matrix34 &transform);

  void subscriberDepthImageHandler(const sensor_msgs::Image &msg);

  // ******************** INLINES ********************

  void resetMaps();

  Real getElapsedTime();

  void copyFilteredToPointMap();

  void copyInitialToRaw();

  Real getMedian(std::vector<Real> &vec) const;

  void getIndexMax(const std::vector<UInt> &vec, UInt &index) const;

  // ******************** PUBLISHING ********************

  void publishObjectPredictions();
};

#endif // DYNAMIC_HEIGHT_MAP_DYNAMIC_HEIGHT_MAP_H
