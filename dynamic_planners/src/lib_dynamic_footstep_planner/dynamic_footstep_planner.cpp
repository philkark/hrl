#include <std_msgs/String.h>

#include "dynamic_footstep_planner/dynamic_footstep_planner.h"
#include "dynamic_footstep_planner/dynamic_footstep_planner_config.h"
#include "common/progress_publisher.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

DynamicFootstepPlanner::DynamicFootstepPlanner() :
    dynamicHeightMap(nullptr), planValid(false), initialized(false), goalReached(false)
{
}

bool DynamicFootstepPlanner::initializeSettings()
{
  if (dynamicHeightMap == nullptr)
  {
    initialized = false;
    ProgressPublisher::showMessage(progressMessageIndex, "Could not initialize planner. Dynamic height map not set.", 3.0);
    return false;
  }

  DynamicFootstepPlannerConfig::initialize(nh);

  DynamicFootstepPlannerConfig::nodeMapSizeX = (Int)((DynamicHeightMapConfig::maxX - DynamicHeightMapConfig::minX)
      * DynamicFootstepPlannerConfig::nodeMapResolutionRecip);
  DynamicFootstepPlannerConfig::nodeMapSizeY = (Int)((DynamicHeightMapConfig::maxY - DynamicHeightMapConfig::minY)
      * DynamicFootstepPlannerConfig::nodeMapResolutionRecip);

  if ((Int)((DynamicHeightMapConfig::maxX - DynamicHeightMapConfig::minX) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)
      < (DynamicHeightMapConfig::maxX - DynamicHeightMapConfig::minX) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)
    DynamicFootstepPlannerConfig::nodeMapSizeX += 1;
  if ((Int)((DynamicHeightMapConfig::maxY - DynamicHeightMapConfig::minY) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)
      < (DynamicHeightMapConfig::maxY - DynamicHeightMapConfig::minY) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)
    DynamicFootstepPlannerConfig::nodeMapSizeY += 1;

  nodeMap.resize(DynamicFootstepPlannerConfig::nodeMapSizeX, std::vector<UInt>(DynamicFootstepPlannerConfig::nodeMapSizeY, 0));
  nodes.resize((UInt)(DynamicFootstepPlannerConfig::nodeMapSizeX * DynamicFootstepPlannerConfig::nodeMapSizeY * DynamicFootstepPlannerConfig::maxNodesPerCell));

  findFeetCells(DynamicHeightMapConfig::resolutionSpace);

  if (!generateExpansionMap())
  {
    initialized = false;
    return false;
  }
  else
  {
    initialized = true;
    return true;
  }
}

void DynamicFootstepPlanner::setDynamicHeightMap(const DynamicHeightMap* dynamicHeightMap)
{
  this->dynamicHeightMap = dynamicHeightMap;
  initialized = false;
}

void DynamicFootstepPlanner::setProgressMessageIndex(const Int index)
{
  progressMessageIndex = index;
}

void DynamicFootstepPlanner::setGlobalPath(const AStarPath2D &globalPath)
{
  this->globalPath = globalPath;
  goalReached = false;
}

void DynamicFootstepPlanner::invalidateGlobalPath()
{
  globalPath.valid = false;
  planValid = false;
  goalReached = false;
}

void DynamicFootstepPlanner::setRobotPose(const PV3D &pose)
{
  robotPose = pose;
  sincos(pose.C2, &sinYaw, &cosYaw);
}

void DynamicFootstepPlanner::setInitialFeet(const PV4D &leftFoot, const PV4D &rightFoot, const Foot support)
{
  supportFoot = support;
  footstepPlan.resize(2);

  Footstep* stepLeft;
  Footstep* stepRight;
  if (supportFoot == Foot::right)
  {
    stepLeft = &footstepPlan[0];
    stepRight = &footstepPlan[1];
  }
  else
  {
    stepRight = &footstepPlan[0];
    stepLeft = &footstepPlan[1];
  }

  stepLeft->foot = Foot::left;
  stepRight->foot = Foot::right;
  stepLeft->pose = leftFoot;
  stepRight->pose = rightFoot;
  stepLeft->direction = PV2D(cos(leftFoot.C3), sin(leftFoot.C3));
  stepRight->direction = PV2D(cos(rightFoot.C3), sin(rightFoot.C3));
  stepLeft->isStepOver = false;
  stepRight->isStepOver = false;

  footstepPlan[0].time = -1.0;
  footstepPlan[1].time = 0.0;
}

void DynamicFootstepPlanner::findPlan(const Real goalDistance, const Real maxTime)
{
  planValid = false;
  if (!initialized)
    return;

  const ros::WallTime startTime = ros::WallTime::now();

  //set new goals
  if (!findGoalPoint())
    return;

  this->goalDistance = goalDistance;
  distanceMin = goal.norm();
  distanceMinIndex = 1;

  resetNodes();

  if (getGoalDistance(nodes[1]) < goalDistance)
  {
    goalReached = true;
    planValid = false;
    return;
  }

  //main astar loop
  while (openList.size() > 1 && ros::ok())
  {
    const Real secondsPast = (ros::WallTime::now() - startTime).toSec();
    if (secondsPast > maxTime)
    {
      constructPath(distanceMinIndex);
      planValid = true;
      return;
    }

    UInt nodeIndexCurrent = openList[1];
    if (nodeIndexCurrent > 1 && getGoalDistance(nodes[nodeIndexCurrent]) < goalDistance && isNodeGood(nodes[nodeIndexCurrent])
        && isNodeTrajectoryGood(nodes[nodeIndexCurrent]))
    {
      constructPath(nodeIndexCurrent);
      planValid = true;
      return;
    }

    openListRemoveFront();
    expandNode(nodes[nodeIndexCurrent]);
  }

  //remove nodes beyond support foot, if no plan is found
  footstepPlan.resize(2);
  planValid = false;
  return;
}

bool DynamicFootstepPlanner::isPlanValid()
{
  return planValid;
}

void DynamicFootstepPlanner::invalidatePlan()
{
  planValid = false;
}

bool DynamicFootstepPlanner::hasPlanCollision()
{
  for (Int i = 0; i < footstepPlan.size(); ++i)
    if (!isStepFree(footstepPlan[i]))
      return true;

  return false;
}

bool DynamicFootstepPlanner::isGoalReached()
{
  return goalReached;
}

const Foot DynamicFootstepPlanner::getSupportFoot()
{
  return supportFoot;
}

const std::deque<Footstep> &DynamicFootstepPlanner::getFootstepPlan() const
{
  return footstepPlan;
}

const std::vector<std::vector<UInt> > &DynamicFootstepPlanner::getNodeMap() const
{
  return nodeMap;
}

PV4D DynamicFootstepPlanner::getFootDisplacement(const UInt index) const
{
  if (index >= footstepPlan.size())
    return PV4D();

  const Footstep &footstep = footstepPlan[index];
  const ExpansionNode &node =
      footstep.foot == Foot::left ? expansionMap.getLeftExpansionNodes()[footstep.expansionMapIndex] :
          expansionMap.getRightExpansionNodes()[footstep.expansionMapIndex];

  PV4D pose;
  pose.C0 = node.position.C0;
  pose.C1 = node.position.C1 + (footstep.foot == Foot::left ? DynamicFootstepPlannerConfig::footSeparation : -DynamicFootstepPlannerConfig::footSeparation);
  pose.C2 = footstep.pose.C2 - footstepPlan[index - 1].pose.C2;
  pose.C3 = node.footAngle;

  return pose;
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

bool DynamicFootstepPlanner::generateExpansionMap()
{
  heuristicFactor = 1.0 / DynamicFootstepPlannerConfig::expansionForward
      * (CONST_FOOTSTEP_DURATION
          + sqrt(pow(DynamicFootstepPlannerConfig::expansionForward, 2) + pow(DynamicFootstepPlannerConfig::footSeparation, 2))
              * DynamicFootstepPlannerConfig::robotSpeedRecip);

  bool validate;
  RosParamLoader::getRosParam(nh, "DynamicFootstepPlanner::generateExpansionMap", "perform_reachability_checks", validate);
  expansionMap.initializeExpansionMap(validate, progressMessageIndex);
  return expansionMap.isValid();
}

void DynamicFootstepPlanner::findFeetCells(const Real resolution)
{
  footCellsRight.clear();
  footCellsLeft.clear();

  Int maxIndex = (Int)(3
      + std::sqrt(
          std::pow(std::max(fabs(DynamicFootstepPlannerConfig::sizeFront), fabs(DynamicFootstepPlannerConfig::sizeBack)), 2)
              + std::pow(std::max(fabs(DynamicFootstepPlannerConfig::sizeInner), fabs(DynamicFootstepPlannerConfig::sizeOuter)), 2)) / resolution);
  Real cosA, sinA;
  for (Int angle = 180; angle >= -180; --angle)
  {
    footCellsRight.push_back(std::vector<Cell2DSigned>());
    footCellsLeft.push_back(std::vector<Cell2DSigned>());

    sincos(angle * TORAD, &sinA, &cosA);
    for (Int i = -maxIndex; i <= maxIndex; ++i)
      for (Int j = -maxIndex; j <= maxIndex; ++j)
      {
        const Real x = resolution * (i * cosA + j * sinA);
        const Real y = resolution * (-i * sinA + j * cosA);

        if (x <= DynamicFootstepPlannerConfig::sizeFront && x >= -DynamicFootstepPlannerConfig::sizeBack && y <= DynamicFootstepPlannerConfig::sizeInner
            && y >= -DynamicFootstepPlannerConfig::sizeOuter)
          footCellsRight.back().push_back(Cell2DSigned(i, j));

        if (x <= DynamicFootstepPlannerConfig::sizeFront && x >= -DynamicFootstepPlannerConfig::sizeBack && y <= DynamicFootstepPlannerConfig::sizeOuter
            && y >= -DynamicFootstepPlannerConfig::sizeInner)
          footCellsLeft.back().push_back(Cell2DSigned(i, j));
      }
  }
}

bool DynamicFootstepPlanner::findGoalPoint()
{
  if (!globalPath.valid)
    return false;

  const SegmentTypeMap &segmentTypeMap = dynamicHeightMap->getSegmentTypeMapConst(0u);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(0u);

  const std::vector<PV2D> &points = globalPath.pointsSmoothed;

  for (Int index = points.size() - 1; index > 0; --index)
  {
    const PV2D &currentPoint = points[index];
    const Real dx = currentPoint.C0 - robotPose.C0;
    const Real dy = currentPoint.C1 - robotPose.C1;
    const PV2D pointTransformed(dx * cosYaw + dy * sinYaw, -dx * sinYaw + dy * cosYaw);

    const Cell2D cell = DynamicHeightMapConfig::getCell(pointTransformed);

    if (cell.x >= DynamicHeightMapConfig::sizeX || cell.y >= DynamicHeightMapConfig::sizeY || segmentTypeMap[cell.x][cell.y] != SegmentType::Planar
        || edgeMap[cell.x][cell.y] >= 0)
      continue;

    goal = pointTransformed;
    return true;
  }

  return false;
}

void DynamicFootstepPlanner::openListInsert(const AStarNodeFootsteps &node)
{
  openList.push_back(node.index);
  UInt index = openList.size() - 1;
  UInt indexHalf = index / 2;

  while (indexHalf > 0 && nodes[openList[index]].f < nodes[openList[indexHalf]].f)
  {
    UInt nodeBuffer = openList[index];
    openList[index] = openList[indexHalf];
    openList[indexHalf] = nodeBuffer;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void DynamicFootstepPlanner::openListRemoveFront()
{
  openList[1] = openList.back();
  openList.pop_back();

  UInt index = 1, indexDouble = 2, indexDoubleOne = 3;
  while (indexDouble < openList.size())
  {
    if (indexDoubleOne == openList.size())
    {
      if (nodes[openList[index]].f > nodes[openList[indexDouble]].f)
      {
        UInt nodeBuffer = openList[index];
        openList[index] = openList[indexDouble];
        openList[indexDouble] = nodeBuffer;
        index = indexDouble;
      }
      else
        break;
    }
    else if (nodes[openList[index]].f > nodes[openList[indexDouble]].f || nodes[openList[index]].f > nodes[openList[indexDoubleOne]].f)
    {
      if (nodes[openList[indexDouble]].f < nodes[openList[indexDoubleOne]].f)
      {
        UInt nodeBuffer = openList[index];
        openList[index] = openList[indexDouble];
        openList[indexDouble] = nodeBuffer;
        index = indexDouble;
      }
      else
      {
        UInt nodeBuffer = openList[index];
        openList[index] = openList[indexDoubleOne];
        openList[indexDoubleOne] = nodeBuffer;
        index = indexDoubleOne;
      }
    }
    else
      break;
    indexDouble = index * 2;
    indexDoubleOne = indexDouble + 1;
  }
}

void DynamicFootstepPlanner::expandNode(const AStarNodeFootsteps &node)
{
  if (node.index > 1 && !isNodeGood(node))
    return;

  bool nodeTrajectoryGood = true;
  if (node.index > 1)
    nodeTrajectoryGood = isNodeTrajectoryGood(node);

  if (nodeTrajectoryGood)
  {
    Real distanceNew = getGoalDistance(node);
    if (distanceNew < distanceMin)
    {
      distanceMin = distanceNew;
      distanceMinIndex = node.index;
    }

    if (node.foot == Foot::right)
      expandNodeLoop(node, expansionMap.getLeftExpansionNodes());
    else
      expandNodeLoop(node, expansionMap.getRightExpansionNodes());
  }
  else if (!node.isStepOver)
  {
    const AStarNodeFootsteps &nodeParent = nodes[node.indexParent];

    AStarNodeFootsteps &nodeNew = nodes[nodeIndexNext];
    nodeNew = node;
    nodeNew.index = nodeIndexNext;
    nodeNew.isStepOver = true;
    finalizeNode(nodeNew);
    openListInsert(nodeNew);
    ++nodeIndexNext;
  }
}

void DynamicFootstepPlanner::expandNodeLoop(const AStarNodeFootsteps &node, const std::vector<ExpansionNode> &expansionNodes)
{
  const PointMap &pointMap = dynamicHeightMap->getPointMapConst(node.time);
  PV2D zeroPosition(node.pose.C0, node.pose.C1);
  if (node.foot == Foot::right)
    zeroPosition += PV2D(-node.angleSin, node.angleCos) * DynamicFootstepPlannerConfig::footSeparation;
  else
    zeroPosition += PV2D(node.angleSin, -node.angleCos) * DynamicFootstepPlannerConfig::footSeparation;

  for (UInt i = 0; i < expansionNodes.size();)
  {
    const ExpansionNode &expansionNode = expansionNodes[i];
    AStarNodeFootsteps &nodeNew = nodes[nodeIndexNext];
    nodeNew.expansionNodeIndex = i;
    nodeNew.index = nodeIndexNext;
    nodeNew.indexParent = node.index;
    nodeNew.foot = node.foot == Foot::right ? Foot::left : Foot::right;
    nodeNew.pose.C3 = node.pose.C3 + expansionNode.footAngle;

    nodeNew.pose.C0 = zeroPosition.C0 + expansionNode.position.C0 * node.angleCos - expansionNode.position.C1 * node.angleSin;
    nodeNew.pose.C1 = zeroPosition.C1 + expansionNode.position.C0 * node.angleSin + expansionNode.position.C1 * node.angleCos;

    if (nodeNew.pose.C0 < DynamicHeightMapConfig::boundaryMinX || nodeNew.pose.C0 > DynamicHeightMapConfig::boundaryMaxX
        || nodeNew.pose.C1 < DynamicHeightMapConfig::boundaryMinY || nodeNew.pose.C1 > DynamicHeightMapConfig::boundaryMaxY)
    {
      ++i;
      continue;
    }

    setNodeCell(nodeNew);
    if (*(nodeNew.nodeCellCount) >= DynamicFootstepPlannerConfig::maxNodesPerCell)
    {
      ++i;
      continue;
    }

    nodeNew.pose.C2 = getPointMapCell(nodeNew).C2;
    const Real heightDifference = nodeNew.pose.C2 - node.pose.C2;
    if (heightDifference > expansionNode.heightUp || heightDifference < expansionNode.heightDown)
    {
      ++i;
      continue;
    }

    adjustAngle(nodeNew);
    setNodeTimeFromParent(nodeNew);
    nodeNew.isStepOver = false;

    if (isNodeGoodQuick(nodeNew))
    {
      finalizeNode(nodeNew);
      openListInsert(nodeNew);
      ++(*nodeNew.nodeCellCount);
      ++nodeIndexNext;
      i = expansionNode.nextExpansionNodeIndex;
    }
    else
      ++i;
  }
}

bool DynamicFootstepPlanner::isNodeGoodQuick(const AStarNodeFootsteps &node)
{
  const PointMap &pointMap = dynamicHeightMap->getPointMapConst(node.time);
  const SegmentTypeMap &segmentTypeMap = dynamicHeightMap->getSegmentTypeMapConst(node.time);
  const SegmentMap &segmentMap = dynamicHeightMap->getSegmentMapConst(node.time);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(node.time);
//check if node lies in bounds
  if (node.pose.C0 < DynamicHeightMapConfig::boundaryMinX || node.pose.C0 > DynamicHeightMapConfig::boundaryMaxX
      || node.pose.C1 < DynamicHeightMapConfig::boundaryMinY || node.pose.C1 > DynamicHeightMapConfig::boundaryMaxY)
    return false;

//check if region and its normal are ok
  const UInt cellX = getCellX(node.pose.C0);
  const UInt cellY = getCellY(node.pose.C1);
  if (segmentTypeMap[cellX][cellY] != SegmentType::Planar)
    return false;

  const UInt segmentNumber = segmentMap[cellX][cellY];

//check if position lies inside safety margin, if true, check corners
  PV2D cornerPosition;
  UInt indexXCorner, indexYCorner;

  if (edgeMap[cellX][cellY] == -1)
    return true;
  else if ((Real)edgeMap[cellX][cellY] < DynamicFootstepPlannerConfig::minEdgeDistance)
    return false;

  else
  {
    if (node.foot == Foot::right)
      for (UInt i = 0; i < 4; ++i)
      {
        PV2D cornerPosition = MathStd::transform(DynamicFootstepPlannerConfig::cornersRight[i], PV2D(node.pose.C0, node.pose.C1), node.angleSin, node.angleCos);
        indexXCorner = getCellX(cornerPosition.C0);
        indexYCorner = getCellY(cornerPosition.C1);
        if (segmentTypeMap[indexXCorner][indexYCorner] != SegmentType::Planar || segmentMap[indexXCorner][indexYCorner] != segmentNumber)
          return false;
      }
    else
      for (UInt i = 0; i < 4; ++i)
      {
        PV2D cornerPosition = MathStd::transform(DynamicFootstepPlannerConfig::cornersLeft[i], PV2D(node.pose.C0, node.pose.C1), node.angleSin, node.angleCos);
        indexXCorner = getCellX(cornerPosition.C0);
        indexYCorner = getCellY(cornerPosition.C1);
        if (segmentTypeMap[indexXCorner][indexYCorner] != SegmentType::Planar || segmentMap[indexXCorner][indexYCorner] != segmentNumber)
          return false;
      }
  }

//all checks were successful
  return true;
}

bool DynamicFootstepPlanner::isNodeGood(const AStarNodeFootsteps &node)
{
  const SegmentTypeMap &segmentTypeMap = dynamicHeightMap->getSegmentTypeMapConst(node.time);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(node.time);

  const UInt cellCentreX = getCellX(node.pose.C0);
  const UInt cellCentreY = getCellY(node.pose.C1);

  if (edgeMap[cellCentreY][cellCentreY] < 0)
    return true;

  const std::vector<Cell2DSigned> &cells = getFootCells(node.pose.C3, node.foot);

  for (std::vector<Cell2DSigned>::const_iterator it = cells.begin(); it != cells.end(); ++it)
  {
    const UInt cellX = (UInt)(cellCentreX + it->x);
    const UInt cellY = (UInt)(cellCentreY + it->y);
    if (cellX >= DynamicHeightMapConfig::sizeX || cellY >= DynamicHeightMapConfig::sizeY)
      continue;

    if (segmentTypeMap[cellX][cellY] != SegmentType::Planar || edgeMap[cellX][cellY] == 0)
      return false;
  }
  return true;
}

bool DynamicFootstepPlanner::isNodeTrajectoryGood(const AStarNodeFootsteps &node)
{
  const AStarNodeFootsteps &nodeParent = nodes[node.indexParent];
  const AStarNodeFootsteps &nodePrev = nodes[nodeParent.indexParent];
  const Real distance = getDistance(node, nodePrev);
  //TODO get corrected angle
  const Real intermediateAngle = 0.5 * (node.pose.C3 + nodePrev.pose.C3);
  const Real intermediateTime = 0.5 * (node.time + nodeParent.time);
  PV2D direction(node.pose.C0 - nodePrev.pose.C0, node.pose.C1 - nodePrev.pose.C1);
  direction /= distance;

  const ExpansionConnection &connection = expansionMap.getExpansionConnection(distance, getHeightDifference(node, nodePrev));

  const std::vector<std::pair<Real, Real> >* intermediatePositions;
  if (node.isStepOver)
    intermediatePositions = &connection.positionsOver;
  else
    intermediatePositions = &connection.positionsNormal;

  for (UInt i = 0; i < intermediatePositions->size(); ++i)
  {
    const std::pair<Real, Real> &intermediatePosition = (*intermediatePositions)[i];
    PV4D poseIntermed(nodePrev.pose.C0 + direction.C0 * intermediatePosition.first, nodePrev.pose.C1 + direction.C1 * intermediatePosition.first,
                      nodePrev.pose.C2 + intermediatePosition.second, intermediateAngle);

    if (!isFootAboveHeight(node.foot, poseIntermed, intermediateTime))
      return false;

  }

  return true;
}

bool DynamicFootstepPlanner::isFootAboveHeight(const Foot foot, const PV4D &pose, const Real time)
{
  const PointMap &pointMap = dynamicHeightMap->getPointMapConst(time);

  const std::vector<Cell2DSigned> &cells = getFootCells(pose.C3, foot);
  const Int cellCentreX = getCellX(pose.C0);
  const Int cellCentreY = getCellY(pose.C1);

  for (std::vector<Cell2DSigned>::const_iterator it = cells.begin(); it != cells.end(); ++it)
  {
    const UInt cellX = (UInt)(cellCentreX + it->x);
    const UInt cellY = (UInt)(cellCentreY + it->y);
    if (cellX >= DynamicHeightMapConfig::sizeX || cellY >= DynamicHeightMapConfig::sizeY)
      continue;

    if (pointMap[cellX][cellY].C2 > pose.C2)
      return false;
  }
  return true;
}

void DynamicFootstepPlanner::constructPath(UInt nodeIndex)
{
  footstepPlan.clear();

  Footstep step;
  std::deque<UInt> stepList;
  while (true)
  {
    stepList.push_front(nodeIndex);
    if (nodes[nodeIndex].indexParent == -1)
      break;
    nodeIndex = nodes[nodeIndex].indexParent;
  }
  for (std::deque<UInt>::const_iterator it = stepList.begin(); it != stepList.end(); ++it)
  {
    step = nodes[*it];
    footstepPlan.push_back(step);
  }
}

bool DynamicFootstepPlanner::isStepFree(const Footstep &step)
{
  const Int cellCenterX = getCellX(step.pose.C0);
  const Int cellCenterY = getCellY(step.pose.C1);
  const std::vector<Cell2DSigned> &cells = getFootCells(step.pose.C3, step.foot);
  const SegmentTypeMap &segmentTypeMap = dynamicHeightMap->getSegmentTypeMapConst(step.time);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(step.time);

  for (std::vector<Cell2DSigned>::const_iterator it = cells.begin(); it != cells.end(); ++it)
  {
    const UInt cellX = (UInt)(cellCenterX + it->x);
    const UInt cellY = (UInt)(cellCenterY + it->y);
    if (cellX >= DynamicHeightMapConfig::sizeX || cellY >= DynamicHeightMapConfig::sizeY)
      continue;

    if (segmentTypeMap[cellX][cellY] == SegmentType::Nonplanar)
      return false;
  }

  return true;
}

//////////////////// HELPER INLINES //////////////////////////////

inline UInt DynamicFootstepPlanner::getCellX(const Real &position)
{
  return (UInt)((position - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip);
}

inline UInt DynamicFootstepPlanner::getCellY(const Real &position)
{
  return (UInt)((position - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip);
}

inline UInt DynamicFootstepPlanner::getNodeCellX(const Real &position)
{
  return (UInt)((position - DynamicHeightMapConfig::minX) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip);
}

inline UInt DynamicFootstepPlanner::getNodeCellY(const Real &position)
{
  return (UInt)((position - DynamicHeightMapConfig::minY) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip);
}

inline const std::vector<Cell2DSigned> &DynamicFootstepPlanner::getFootCells(const Real angle, const Foot foot)
{
  if (foot == Foot::right)
    return footCellsRight[(UInt)(180.5 - angle * TODEG)];
  else
    return footCellsLeft[(UInt)(180.5 - angle * TODEG)];
}

inline const PV3D &DynamicFootstepPlanner::getPointMapCell(const AStarNodeFootsteps &node)
{
  const PointMap &pointMap = dynamicHeightMap->getPointMapConst(node.time);
  return pointMap[(UInt)((node.pose.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip)][(UInt)((node.pose.C1
      - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip)];
}

inline const UInt &DynamicFootstepPlanner::getSegmentCell(const AStarNodeFootsteps &node)
{
  const SegmentMap &segmentMap = dynamicHeightMap->getSegmentMapConst(node.time);
  return segmentMap[(UInt)((node.pose.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip)][(UInt)((node.pose.C1
      - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip)];
}

inline const SegmentType &DynamicFootstepPlanner::getSegmentTypeMapCell(const AStarNodeFootsteps &node)
{
  const SegmentTypeMap &segmentTypeMap = dynamicHeightMap->getSegmentTypeMapConst(node.time);
  return segmentTypeMap[(UInt)((node.pose.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip)][(UInt)((node.pose.C1
      - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip)];
}

inline const Int &DynamicFootstepPlanner::getEdgeMapCell(const AStarNodeFootsteps &node)
{
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(node.time);
  return edgeMap[(UInt)((node.pose.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip)][(UInt)((node.pose.C1
      - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip)];
}

inline void DynamicFootstepPlanner::resetNodes()
{
  openList.clear();
  openList.push_back(-1);
  openList.push_back(1);
  nodeIndexNext = 2;

  for (Int i = 0; i < DynamicFootstepPlannerConfig::nodeMapSizeX; ++i)
    for (Int j = 0; j < DynamicFootstepPlannerConfig::nodeMapSizeY; ++j)
      nodeMap[i][j] = 0;

  //initialize starting nodes
  nodes[0] = footstepPlan[0];
  setNodeCell(nodes[0]);
  nodes[0].indexParent = -1;
  nodes[0].index = 0;
  nodes[0].expansionNodeIndex = expansionMap.size() - 2;
  nodes[0].f = heuristicFactor * goal.norm();

  nodes[1] = footstepPlan[1];
  setNodeCell(nodes[1]);
  nodes[1].indexParent = 0;
  nodes[1].index = 1;
  nodes[1].expansionNodeIndex = expansionMap.size() - 2;
  nodes[1].f = heuristicFactor * goal.norm();
}

inline void DynamicFootstepPlanner::setNodeCell(AStarNodeFootsteps &node)
{
  node.nodeCellCount =
      &(nodeMap[(UInt)((node.pose.C0 - DynamicHeightMapConfig::minX) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)][(UInt)((node.pose.C1
          - DynamicHeightMapConfig::minY) * DynamicFootstepPlannerConfig::nodeMapResolutionRecip)]);
}

inline void DynamicFootstepPlanner::adjustAngle(AStarNodeFootsteps &node)
{
  if (node.pose.C3 > RAD180)
    node.pose.C3 -= RAD360;
  else if (node.pose.C3 < -RAD180)
    node.pose.C3 += RAD360;
  sincos(node.pose.C3, &node.angleSin, &node.angleCos);
}

inline void DynamicFootstepPlanner::setNodeTimeFromParent(AStarNodeFootsteps &node)
{
  const AStarNodeFootsteps &nodeParent = nodes[node.indexParent];
  node.time = nodeParent.time + CONST_FOOTSTEP_DURATION
      + PV2D(node.pose.C0 - nodeParent.pose.C0, node.pose.C1 - nodeParent.pose.C1).norm() * DynamicFootstepPlannerConfig::robotSpeedRecip;

}

inline void DynamicFootstepPlanner::finalizeNode(AStarNodeFootsteps &node)
{
  node.direction.C0 = node.angleCos;
  node.direction.C1 = node.angleSin;
  node.pose.C2 = getPointMapCell(node).C2;
  node.f = node.time + heuristicFactor * getGoalDistance(node);
}

inline Real DynamicFootstepPlanner::getHeightDifference(const AStarNodeFootsteps &node, const AStarNodeFootsteps &nodePrev)
{
  return node.pose.C2 - nodePrev.pose.C2;
}

inline Real DynamicFootstepPlanner::getDistance(const AStarNodeFootsteps &node, const AStarNodeFootsteps &nodePrev)
{
  return sqrt(pow(node.pose.C0 - nodePrev.pose.C0, 2) + pow(node.pose.C1 - nodePrev.pose.C1, 2));
}

inline Real DynamicFootstepPlanner::getGoalDistance(const AStarNodeFootsteps &node)
{
  return PV2D(node.pose.C0 - goal.C0, node.pose.C1 - goal.C1).norm();
}
