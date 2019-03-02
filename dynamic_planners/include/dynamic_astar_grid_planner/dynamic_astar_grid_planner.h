#ifndef DYNAMIC_ASTAR_GRID_PLANNER_DYNAMIC_ASTAR_GRID_PLANNER_H_
#define DYNAMIC_ASTAR_GRID_PLANNER_DYNAMIC_ASTAR_GRID_PLANNER_H_

#include "dynamic_astar_grid_planner/dynamic_astar_grid_planner_structs.h"
#include "dynamic_height_map/dynamic_height_map.h"
#include "dynamic_height_map/global_map.h"

#include <math_std/math_std.h>
#include <tools_std/ros_param_loader.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <utility>

class DynamicAStarGridPlanner
{
public:
  enum class Status
  {
    SUCCESS, NO_PATH, OCCUPIED_START_OR_GOAL, EQUAL_START_AND_GOAL, REQUIRED_MAP_NOT_SET, MISSING_GLOBAL_PATH
  };

  Status status;

  DynamicAStarGridPlanner();

  void setDynamicHeightMap(const DynamicHeightMap* dynamicHeightMap);

  void invalidateLocalPath();

  void setGlobalMap(const GlobalMap* globalMap);

  void setPlanningParameters();

  void setRobotPose(const PV3D &pose);

  void invalidateGlobalPath();

  bool findAStarPathLocal(const UInt attempts);

  bool findAStarPathGlobal(const PV2D startPoint, const PV2D goalPoint);

  const AStarPath2D &getAStarPathLocal() const;

  const AStarPath2D &getAStarPathGlobal() const;

private:
  const DynamicHeightMap* dynamicHeightMap;     ///< Reference to dynamic height map used during planning for collision checks.
  const GlobalMap* globalMap;
  PV3D robotPose;
  Real sinYaw, cosYaw;

  Real movementSpeed;
  Real movementSpeedRecip;
  AStarPath2D AStarPathGlobal;
  AStarPath2D AStarPath;     ///< Full A* grid path that is found during the 2D planning part.
  Cell2D AStarCellStart;     ///< Start cell for the current 2D path search.
  Cell2D AStarCellGoal;     ///< Goal cell for the current 2D path search.
  std::vector<std::vector<AStarNodeGrid> > AStarNodes;     ///< Structured map of A* nodes that is used during the path search.
  std::vector<std::vector<AStarNodeGrid> > AStarNodesGlobal;
  std::vector<Cell2D> openListNodes;     ///< Priority queue of grid cells on AStarNodes for the A* path search.
  Real localSmoothingFactor;     ///< Factor that indicates the smoothing behavior of the A* path; good value lies between 1.5 and 2.5.
  Real localSmoothingDistance;     ///< Maximum distance from intermediate corners that is used during the A* path smoothing.
  Real localSmoothedPointDistance;     ///< The approximate final distance between points of the smoothed A* path.
  Real globalSmoothingFactor;     ///< Factor that indicates the smoothing behavior of the A* path; good value lies between 1.5 and 2.5.
  Real globalSmoothingDistance;     ///< Maximum distance from intermediate corners that is used during the A* path smoothing.
  Real globalSmoothedPointDistance;     ///< The approximate final distance between points of the smoothed A* path.

  //////////////////// LOCAL PLANNING //////////////////////////////

  void createNodesMapLocal();

  Int setLocalGoalCell(Int index, const UInt segment);

  bool findAStarPathLocalCont();

  void expandNodeLocal(const AStarNodeGrid &node);

  void updateFCostLocal(AStarNodeGrid &node) const;

  void constructAStarPathLocal();

  void getSmoothTrajFromAStarPathLocal();

  bool isConnectionLineFreeLocal(const PV2D &pointStart, const Real timeStart, const PV2D &pointEnd, const Real timeEnd);

  void openListInsertNodeLocal(AStarNodeGrid &node);

  void openListUpdateNodeLocal(AStarNodeGrid &node);

  void openListRemoveFrontNodeLocal();

  //////////////////// GLOBAL PLANNING /////////////////////////////

  void createNodesMapGlobal();

  void expandNodeGlobal(const AStarNodeGrid &node);

  void updateFCostGlobal(AStarNodeGrid &node) const;

  void constructAStarPathGlobal();

  void getSmoothTrajFromAStarPathGlobal();

  bool isConnectionLineFreeGlobal(const PV2D &pointStart, const PV2D &pointEnd);

  void openListInsertNodeGlobal(AStarNodeGrid &node);

  void openListUpdateNodeGlobal(AStarNodeGrid &node);

  void openListRemoveFrontNodeGlobal();

  //////////////////// COMMON //////////////////////////////////////



  Real distanceCells(const Cell2D &cell1, const Cell2D &cell2);
};

#endif // DYNAMIC_ASTAR_GRID_PLANNER_DYNAMIC_ASTAR_GRID_PLANNER_H_
