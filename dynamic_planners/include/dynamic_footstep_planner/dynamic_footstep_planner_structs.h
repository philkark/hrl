#ifndef DYNAMIC_FOOTSTEP_PLANNER_DYNAMIC_FOOTSTEP_PLANNER_STRUCTS_H_
#define DYNAMIC_FOOTSTEP_PLANNER_DYNAMIC_FOOTSTEP_PLANNER_STRUCTS_H_

//std
#include <vector>

//personal
#include <math_std/math_std.h>

//local
#include "dynamic_height_map/cells.h"

enum class Foot
{
  left,
  right
};

struct Footstep;

struct AStarNodeFootsteps
{
  //astar costs, index locations, and positions
  Real f;
  UInt index, indexParent;
  UInt expansionNodeIndex;
  UInt* nodeCellCount;

  //location and settings of the node
  Foot foot;
  PV4D pose;
  PV2D direction;

  Real time;
  Real angleSin, angleCos;
  bool isStepOver;

  //equates direction, position, angle, foot, and height of the given footstep
  void operator=(const Footstep &step);
};

struct Footstep
{
  //footstep location and settings
  Foot foot;
  PV4D pose;
  PV2D direction;
  Real time;
  bool isStepOver;

  UInt expansionMapIndex;

  //list of cells occupied by the foot
  std::vector<Cell2D> cells;

  //equates direction, position, angle, foot, and height of the given node
  void operator=(const AStarNodeFootsteps &node);
};

#endif // DYNAMIC_FOOTSTEP_PLANNER_DYNAMIC_FOOTSTEP_PLANNER_STRUCTS_H_
