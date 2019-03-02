#include "dynamic_footstep_planner/dynamic_footstep_planner_config.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

Real DynamicFootstepPlannerConfig::sizeFront = 0.0;
Real DynamicFootstepPlannerConfig::sizeBack = 0.0;
Real DynamicFootstepPlannerConfig::sizeInner = 0.0;
Real DynamicFootstepPlannerConfig::sizeOuter = 0.0;
Real DynamicFootstepPlannerConfig::footSeparation = 0.0;
Real DynamicFootstepPlannerConfig::footSeparationHalf = 0.0;

Real DynamicFootstepPlannerConfig::expansionForward = 0.0;
Real DynamicFootstepPlannerConfig::expansionSideways = 0.0;
Real DynamicFootstepPlannerConfig::expansionBackward = 0.0;
Real DynamicFootstepPlannerConfig::expansionUp = 0.0;
Real DynamicFootstepPlannerConfig::expansionDown = 0.0;
Real DynamicFootstepPlannerConfig::expansionNodeDistance = 0.0;
Real DynamicFootstepPlannerConfig::expansionMaxFootRotation = 0.0;
Real DynamicFootstepPlannerConfig::expansionBaseHeight = 0.0;

Real DynamicFootstepPlannerConfig::angularSkipDistance = 0.0;
Real DynamicFootstepPlannerConfig::angularSkipIncrease = 0.0;

Real DynamicFootstepPlannerConfig::minEdgeDistance = 0.0;
std::vector<PV2D> DynamicFootstepPlannerConfig::cornersRight = std::vector<PV2D>();
std::vector<PV2D> DynamicFootstepPlannerConfig::cornersLeft = std::vector<PV2D>();
Real DynamicFootstepPlannerConfig::nodeMapResolution = 0.0;
Real DynamicFootstepPlannerConfig::nodeMapResolutionRecip = 0.0;
UInt DynamicFootstepPlannerConfig::nodeMapSizeX = 0.0;
UInt DynamicFootstepPlannerConfig::nodeMapSizeY = 0.0;
Int DynamicFootstepPlannerConfig::maxNodesPerCell = 0.0;
Real DynamicFootstepPlannerConfig::robotSpeed = 0.0;
Real DynamicFootstepPlannerConfig::robotSpeedRecip = 0.0;
Real DynamicFootstepPlannerConfig::stepNormalHeight = 0.0;
Real DynamicFootstepPlannerConfig::stepOverHeight = 0.0;
Real DynamicFootstepPlannerConfig::trajectoryPoseDistances = 0;

void DynamicFootstepPlannerConfig::initialize(const ros::NodeHandle &nh)
{
  cornersRight.resize(4);
  cornersLeft.resize(4);

  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "foot_size_front", sizeFront);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "foot_size_back", sizeBack);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "foot_size_inner", sizeInner);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "foot_size_outer", sizeOuter);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "foot_separation", footSeparation);

  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_forward", expansionForward);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_sideways", expansionSideways);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_backward", expansionBackward);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_up", expansionUp);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_down", expansionDown);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_node_distance", expansionNodeDistance);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_map_max_foot_rotation", expansionMaxFootRotation);
  expansionMaxFootRotation *= TORAD;
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "expansion_base_height", expansionBaseHeight);

  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "search_skip_distance", angularSkipDistance);
  angularSkipDistance *= TORAD;
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "search_skip_increase_factor", angularSkipIncrease);
  angularSkipIncrease *= TORAD;

  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "robot_movement_speed", robotSpeed);
  robotSpeedRecip = 1.0 / robotSpeed;
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "trajectory_normal_height", stepNormalHeight);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "trajectory_over_height", stepOverHeight);
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "trajectory_pose_distances", trajectoryPoseDistances);

  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "node_map_resolution", nodeMapResolution);
  nodeMapResolutionRecip = 1.0 / nodeMapResolution;
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlannerConfig::DynamicFootstepPlannerConfig()", "max_nodes_per_node_cell", maxNodesPerCell);

  footSeparationHalf = footSeparation * 0.5;
  cornersRight[0] = PV2D(sizeFront, sizeInner);
  cornersRight[1] = PV2D(sizeFront, -sizeOuter);
  cornersRight[2] = PV2D(-sizeBack, -sizeOuter);
  cornersRight[3] = PV2D(-sizeBack, sizeInner);
  cornersLeft[0] = PV2D(sizeFront, sizeOuter);
  cornersLeft[1] = PV2D(sizeFront, -sizeInner);
  cornersLeft[2] = PV2D(-sizeBack, -sizeInner);
  cornersLeft[3] = PV2D(-sizeBack, sizeOuter);
  findMinEdgeDistance();

}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void DynamicFootstepPlannerConfig::findMinEdgeDistance()
{
  minEdgeDistance = sizeFront;
  if (sizeBack < minEdgeDistance)
    minEdgeDistance = sizeBack;
  if (sizeInner < minEdgeDistance)
    minEdgeDistance = sizeInner;
  if (sizeOuter < minEdgeDistance)
    minEdgeDistance = sizeOuter;
  minEdgeDistance *= 1000;
}
