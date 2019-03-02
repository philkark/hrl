#ifndef DYNAMIC_FOOTSTEP_PLANNER_CONFIG_DYNAMIC_FOOTSTEP_PLANNER_H_
#define DYNAMIC_FOOTSTEP_PLANNER_CONFIG_DYNAMIC_FOOTSTEP_PLANNER_H_

//personal
#include <math_std/math_std.h>
#include <tools_std/ros_param_loader.h>

//ros
#include <ros/package.h>

//local
#include "dynamic_height_map/cells.h"
#include "dynamic_footstep_planner/dynamic_footstep_planner_structs.h"


struct DynamicFootstepPlannerConfig
{
  //feet sizes
  static Real sizeFront;
  static Real sizeBack;
  static Real sizeOuter;
  static Real sizeInner;
  static Real footSeparation;
  static Real footSeparationHalf;
  static Real minEdgeDistance;     ///< minimum distance (in mm) to edge for node to be added to priority queue

  static Real expansionForward;
  static Real expansionSideways;
  static Real expansionBackward;
  static Real expansionUp;
  static Real expansionDown;
  static Real expansionNodeDistance;
  static Real expansionMaxFootRotation;
  static Real expansionBaseHeight;

  static Real angularSkipDistance;
  static Real angularSkipIncrease;

  //corner points in foot frame
  static std::vector<PV2D> cornersRight;
  static std::vector<PV2D> cornersLeft;

  static Real nodeMapResolution;
  static Real nodeMapResolutionRecip;
  static UInt nodeMapSizeX;
  static UInt nodeMapSizeY;
  static Int maxNodesPerCell;

  static Real robotSpeed;
  static Real robotSpeedRecip;
  static Real stepNormalHeight;
  static Real stepOverHeight;
  static Real trajectoryPoseDistances;

public:
  static void initialize(const ros::NodeHandle &nh);

private:
  static void findMinEdgeDistance();
};

#endif //DYNAMIC_FOOTSTEP_PLANNER_CONFIG_DYNAMIC_FOOTSTEP_PLANNER_H_
