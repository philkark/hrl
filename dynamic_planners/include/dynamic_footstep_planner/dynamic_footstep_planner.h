#ifndef DYNAMIC_FOOTSTEP_PLANNER_H_
#define DYNAMIC_FOOTSTEP_PLANNER_H_

//std
#include <utility>
#include <list>

//personal
#include <math_std/math_std.h>
#include <tools_std/ros_param_loader.h>

//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//local
#include "dynamic_footstep_planner/dynamic_footstep_planner_config.h"
#include "dynamic_footstep_planner/dynamic_footstep_planner_structs.h"
#include "dynamic_footstep_planner/template_interval_vector.h"
#include "dynamic_footstep_planner/expansion_map.h"
#include "dynamic_height_map/dynamic_height_map.h"
#include "dynamic_astar_grid_planner/dynamic_astar_grid_planner_structs.h"

#define CONST_FOOTSTEP_DURATION 2.0

class DynamicFootstepPlanner
{
public:
  DynamicFootstepPlanner();

  bool initializeSettings();

  void setDynamicHeightMap(const DynamicHeightMap* dynamicHeightMap);

  void setGlobalPath(const AStarPath2D &globalPath);

  void invalidateGlobalPath();

  void setProgressMessageIndex(const Int index);

  void setRobotPose(const PV3D &pose);

  void setInitialFeet(const PV4D &leftFoot, const PV4D &rightFoot, const Foot support);

  void findPlan(const Real goalDistance, const Real maxTime);

  bool isPlanValid();

  void invalidatePlan();

  bool hasPlanCollision();

  bool isGoalReached();

  const Foot getSupportFoot();

  const std::deque<Footstep> &getFootstepPlan() const;

  const std::vector<std::vector<UInt> > &getNodeMap() const;

  PV4D getFootDisplacement(const UInt index) const;

private:
  ros::NodeHandle nh;
  Int progressMessageIndex;

  const DynamicHeightMap* dynamicHeightMap;
  ExpansionMap expansionMap;
  bool initialized;

  AStarPath2D globalPath;
  PV3D robotPose;
  Real sinYaw, cosYaw;

  PV2D goal;
  Real goalDistance;
  bool goalReached;
  std::deque<Footstep> footstepPlan;
  std::vector<UInt> addedNodes;
  Foot supportFoot;
  Real heuristicFactor;
  bool planValid;

  UInt nodeIndexNext;
  std::vector<AStarNodeFootsteps> nodes;
  std::vector<UInt> openList;
  Real distanceMin;
  UInt distanceMinIndex;

  std::vector<std::vector<UInt> > nodeMap;
  std::vector<std::vector<Cell2DSigned> > footCellsRight;
  std::vector<std::vector<Cell2DSigned> > footCellsLeft;

  bool generateExpansionMap();

  void findFeetCells(const Real resolution);

  bool findGoalPoint();

  void openListInsert(const AStarNodeFootsteps &node);

  void openListRemoveFront();

  void expandNode(const AStarNodeFootsteps &node);

  void expandNodeLoop(const AStarNodeFootsteps &node, const std::vector<ExpansionNode> &expansionNodes);

  bool isNodeGood(const AStarNodeFootsteps &node);

  bool isNodeGoodQuick(const AStarNodeFootsteps &node);

  bool isNodeTrajectoryGood(const AStarNodeFootsteps &node);

  bool isFootAboveHeight(const Foot foot, const PV4D &pose, const Real time);

  void constructPath(UInt nodeIndex);

  bool isStepFree(const Footstep &step);

  //////////////////// HELPER INLINES //////////////////////////////

  UInt getCellX(const Real &position);

  UInt getCellY(const Real &position);

  UInt getNodeCellX(const Real &position);

  UInt getNodeCellY(const Real &position);

  const std::vector<Cell2DSigned> &getFootCells(const Real angle, const Foot foot);

  const PV3D &getPointMapCell(const AStarNodeFootsteps &node);

  const UInt &getSegmentCell(const AStarNodeFootsteps &node);

  const SegmentType &getSegmentTypeMapCell(const AStarNodeFootsteps &node);

  const Int &getEdgeMapCell(const AStarNodeFootsteps &node);

  void resetNodes();

  void setNodeCell(AStarNodeFootsteps &node);

  void adjustAngle(AStarNodeFootsteps &node);

  void setNodeTimeFromParent(AStarNodeFootsteps &node);

  void finalizeNode(AStarNodeFootsteps &node);

  Real getHeightDifference(const AStarNodeFootsteps &node, const AStarNodeFootsteps &nodePrev);

  Real getDistance(const AStarNodeFootsteps &node, const AStarNodeFootsteps &nodePrev);

  Real getGoalDistance(const AStarNodeFootsteps &node);
};

#endif //DYNAMIC_FOOTSTEP_PLANNER_H_
