#include "dynamic_footstep_planner/dynamic_footstep_planner_structs.h"

//////////////////////////////////////////////////////////////////

//////////////////// FOOTSTEP ////////////////////////////////////

//////////////////////////////////////////////////////////////////

void Footstep::operator=(const AStarNodeFootsteps &node)
{
  pose = node.pose;
  direction = node.direction;
  foot = node.foot;
  time = node.time;
  isStepOver = node.isStepOver;
  expansionMapIndex = node.expansionNodeIndex;
}

//////////////////////////////////////////////////////////////////

//////////////////// ASTARNODEFOOTSTEPS //////////////////////////

//////////////////////////////////////////////////////////////////


void AStarNodeFootsteps::operator=(const Footstep &step)
{
  pose = step.pose;
  direction = step.direction;
  sincos(pose.C3, &angleSin, &angleCos);
  foot = step.foot;
  time = step.time;
  isStepOver = step.isStepOver;
}
