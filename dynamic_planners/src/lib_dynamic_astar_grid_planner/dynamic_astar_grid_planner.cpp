#include <dynamic_astar_grid_planner/dynamic_astar_grid_planner.h>

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

DynamicAStarGridPlanner::DynamicAStarGridPlanner() :
    dynamicHeightMap(nullptr), globalMap(nullptr)
{
}

void DynamicAStarGridPlanner::setDynamicHeightMap(const DynamicHeightMap* dynamicHeightMap)
{
  this->dynamicHeightMap = dynamicHeightMap;
  createNodesMapLocal();
  AStarPath.valid = false;
}

void DynamicAStarGridPlanner::invalidateLocalPath()
{
  AStarPath.valid = false;
}

void DynamicAStarGridPlanner::setGlobalMap(const GlobalMap* globalMap)
{
  this->globalMap = globalMap;
  createNodesMapGlobal();
}

void DynamicAStarGridPlanner::invalidateGlobalPath()
{
  AStarPathGlobal.valid = false;
}

void DynamicAStarGridPlanner::setPlanningParameters()
{
  ros::NodeHandle nh;
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "robot_movement_speed", movementSpeed, 0.4);
  movementSpeedRecip = 1.0 / movementSpeed;
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_local_smoothing_factor", localSmoothingFactor, 2.0);
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_local_smoothing_distance", localSmoothingDistance, 0.1);
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_local_smoothed_point_distance", localSmoothedPointDistance, 0.05);
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_global_smoothing_factor", globalSmoothingFactor, 2.0);
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_global_smoothing_distance", globalSmoothingDistance, 0.1);
  RosParamLoader::getRosParam(nh, "DynamicAStarGridPlanner::initialize", "astar_global_smoothed_point_distance", globalSmoothedPointDistance, 0.05);
}

void DynamicAStarGridPlanner::setRobotPose(const PV3D &pose)
{
  robotPose = pose;
  sincos(pose.C2, &sinYaw, &cosYaw);
}

bool DynamicAStarGridPlanner::findAStarPathLocal(const UInt attempts)
{
  if (dynamicHeightMap == nullptr)
  {
    status = Status::REQUIRED_MAP_NOT_SET;
    return false;
  }
  if(!AStarPathGlobal.valid || AStarPathGlobal.pointsSmoothed.size() <= 1)
  {
    status = Status::MISSING_GLOBAL_PATH;
    return false;
  }

  const SegmentMap &segmentMap = dynamicHeightMap->getSegmentMapConst(0u);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(0u);

  AStarPath.valid = false;
  AStarPath.points.clear();
  AStarPath.pointsSmoothed.clear();
  AStarPath.anglesSmoothed.clear();
  AStarPath.times.clear();

  AStarCellStart = DynamicHeightMapConfig::getCell(PV2D());
  if (AStarCellStart.x >= DynamicHeightMapConfig::sizeX || AStarCellStart.y >= DynamicHeightMapConfig::sizeY
      || edgeMap[AStarCellStart.x][AStarCellStart.y] >= 0)
  {
    AStarPath.points.push_back(DynamicHeightMapConfig::getPoint(AStarCellStart));
    AStarPath.times.push_back(0.0);
    AStarPath.pointsSmoothed.push_back(AStarPath.points.front());
    AStarPath.anglesSmoothed.push_back(0.0);
    AStarPath.valid = false;
    status = Status::OCCUPIED_START_OR_GOAL;
    return false;
  }

  const UInt segment = segmentMap[AStarCellStart.x][AStarCellStart.y];
  Int currentGlobalGoalIndex = AStarPathGlobal.pointsSmoothed.size() - 1;

  UInt attemptCount = 1;
  while(currentGlobalGoalIndex > 0 && attemptCount <= attempts)
  {
    ++attemptCount;
    currentGlobalGoalIndex = setLocalGoalCell(currentGlobalGoalIndex, segment);

    if (AStarCellStart == AStarCellGoal)
    {
      AStarPath.points.push_back(DynamicHeightMapConfig::getPoint(AStarCellGoal));
      AStarPath.times.push_back(0.0);
      AStarPath.pointsSmoothed.push_back(DynamicHeightMapConfig::getPoint(AStarCellGoal));
      AStarPath.anglesSmoothed.push_back(0.0);
      AStarPath.valid = true;
      status = Status::EQUAL_START_AND_GOAL;
      return true;
    }

    if(findAStarPathLocalCont())
    {
      AStarPath.valid = true;
      status = Status::SUCCESS;
      return true;
    }
  }

  status = Status::NO_PATH;
  return false;
}

bool DynamicAStarGridPlanner::findAStarPathGlobal(const PV2D startPoint, const PV2D goalPoint)
{
  if (globalMap == nullptr)
  {
    status = Status::REQUIRED_MAP_NOT_SET;
    return false;
  }

  AStarPathGlobal.valid = false;
  AStarPathGlobal.points.clear();
  AStarPathGlobal.pointsSmoothed.clear();
  AStarPathGlobal.anglesSmoothed.clear();
  AStarPathGlobal.times.clear();

  for (UInt xIndex = 0; xIndex < globalMap->sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < globalMap->sizeY; ++yIndex)
      AStarNodesGlobal[xIndex][yIndex].open = AStarNodesGlobal[xIndex][yIndex].closed = false;

  AStarCellStart = globalMap->getCell(startPoint);
  AStarCellGoal = globalMap->getCell(goalPoint);

  const OccupancyMap &occupancyMap = globalMap->occupancyMap;
  if (occupancyMap[AStarCellStart.x][AStarCellStart.y] || occupancyMap[AStarCellGoal.x][AStarCellGoal.y])
  {
    AStarPathGlobal.points.push_back(globalMap->getPoint(AStarCellStart));
    AStarPathGlobal.times.push_back(0.0);
    AStarPathGlobal.pointsSmoothed.push_back(AStarPathGlobal.points.front());
    AStarPathGlobal.anglesSmoothed.push_back(0.0);
    AStarPathGlobal.valid = false;
    status = Status::OCCUPIED_START_OR_GOAL;
    return false;
  }

  if (AStarCellStart == AStarCellGoal)
  {
    AStarPathGlobal.points.push_back(globalMap->getPoint(AStarCellStart));
    AStarPathGlobal.times.push_back(0.0);
    AStarPathGlobal.pointsSmoothed.push_back(AStarPathGlobal.points.front());
    AStarPathGlobal.anglesSmoothed.push_back(0.0);
    AStarPathGlobal.valid = true;
    status = Status::EQUAL_START_AND_GOAL;
    return true;
  }

  openListNodes.clear();
  openListNodes.push_back(Cell2D(-1, -1));
  openListNodes.push_back(AStarCellStart);

  //initialize starting nodes
  AStarNodesGlobal[AStarCellStart.x][AStarCellStart.y].cellParent = Cell2D(-1, -1);
  AStarNodesGlobal[AStarCellStart.x][AStarCellStart.y].cost = 0;
  AStarNodesGlobal[AStarCellStart.x][AStarCellStart.y].indexOpenList = 1;
  updateFCostGlobal(AStarNodesGlobal[AStarCellStart.x][AStarCellStart.y]);

  while (openListNodes.size() > 1)
  {
    AStarNodeGrid &nodeCurrent = AStarNodesGlobal[openListNodes[1].x][openListNodes[1].y];
    if (nodeCurrent.cell == AStarCellGoal)
    {
      constructAStarPathGlobal();
      getSmoothTrajFromAStarPathGlobal();
      AStarPathGlobal.valid = true;
      status = Status::SUCCESS;
      return true;
    }

    openListRemoveFrontNodeGlobal();
    nodeCurrent.closed = true;
    expandNodeGlobal(nodeCurrent);
  }

  status = Status::NO_PATH;
  return false;

}

const AStarPath2D &DynamicAStarGridPlanner::getAStarPathLocal() const
{
  return AStarPath;
}

const AStarPath2D &DynamicAStarGridPlanner::getAStarPathGlobal() const
{
  return AStarPathGlobal;
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// LOCAL PLANNING //////////////////////////////

void DynamicAStarGridPlanner::createNodesMapLocal()
{
  AStarNodes.clear();
  AStarNodes.resize(DynamicHeightMapConfig::sizeX, std::vector<AStarNodeGrid>(DynamicHeightMapConfig::sizeY));

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      AStarNodes[xIndex][yIndex].cell = Cell2D(xIndex, yIndex);

      if (xIndex + 1 < DynamicHeightMapConfig::sizeX && yIndex + 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex + 1][yIndex + 1], DynamicHeightMapConfig::resolutionSpace * M_SQRT2));
      if (xIndex + 1 < DynamicHeightMapConfig::sizeX)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex + 1][yIndex], DynamicHeightMapConfig::resolutionSpace));
      if (xIndex + 1 < DynamicHeightMapConfig::sizeX && yIndex - 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex + 1][yIndex - 1], DynamicHeightMapConfig::resolutionSpace * M_SQRT2));
      if (yIndex + 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex][yIndex + 1], DynamicHeightMapConfig::resolutionSpace));
      if (yIndex - 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex][yIndex - 1], DynamicHeightMapConfig::resolutionSpace));
      if (xIndex - 1 < DynamicHeightMapConfig::sizeX && yIndex + 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex - 1][yIndex + 1], DynamicHeightMapConfig::resolutionSpace * M_SQRT2));
      if (xIndex - 1 < DynamicHeightMapConfig::sizeX)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex - 1][yIndex], DynamicHeightMapConfig::resolutionSpace));
      if (xIndex - 1 < DynamicHeightMapConfig::sizeX && yIndex - 1 < DynamicHeightMapConfig::sizeY)
        AStarNodes[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodes[xIndex - 1][yIndex - 1], DynamicHeightMapConfig::resolutionSpace * M_SQRT2));
    }

  openListNodes.reserve(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);
}

Int DynamicAStarGridPlanner::setLocalGoalCell(Int index, const UInt segment)
{
  const SegmentMap &segmentMap = dynamicHeightMap->getSegmentMapConst(0u);
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(0u);

  for (; index > 0; --index)
  {
    const PV2D &currentPoint = AStarPathGlobal.pointsSmoothed[index];
    const Real dx = currentPoint.C0 - robotPose.C0;
    const Real dy = currentPoint.C1 - robotPose.C1;
    const PV2D pointTransformed(dx * cosYaw + dy * sinYaw, -dx * sinYaw + dy * cosYaw);

    const Cell2D cell = DynamicHeightMapConfig::getCell(pointTransformed);

    if (cell.x >= DynamicHeightMapConfig::sizeX || cell.y >= DynamicHeightMapConfig::sizeY || segmentMap[cell.x][cell.y] != segment
        || edgeMap[cell.x][cell.y] >= 0)
      continue;

    AStarCellGoal = cell;
    return index;
  }

  return index;
}

bool DynamicAStarGridPlanner::findAStarPathLocalCont()
{
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      AStarNodes[xIndex][yIndex].open = AStarNodes[xIndex][yIndex].closed = false;

  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(0u);

  openListNodes.clear();
  openListNodes.push_back(Cell2D(-1, -1));
  openListNodes.push_back(AStarCellStart);

  //initialize starting nodes
  AStarNodes[AStarCellStart.x][AStarCellStart.y].cellParent = Cell2D(-1, -1);
  AStarNodes[AStarCellStart.x][AStarCellStart.y].cost = 0;
  AStarNodes[AStarCellStart.x][AStarCellStart.y].indexOpenList = 1;
  updateFCostLocal(AStarNodes[AStarCellStart.x][AStarCellStart.y]);

  while (openListNodes.size() > 1)
  {
    AStarNodeGrid &nodeCurrent = AStarNodes[openListNodes[1].x][openListNodes[1].y];
    if (nodeCurrent.cell == AStarCellGoal)
    {
      constructAStarPathLocal();
      getSmoothTrajFromAStarPathLocal();
      return true;
    }

    openListRemoveFrontNodeLocal();
    nodeCurrent.closed = true;
    expandNodeLocal(nodeCurrent);
  }

  return false;
}

void DynamicAStarGridPlanner::expandNodeLocal(const AStarNodeGrid &node)
{
  const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(node.cost);

  for (UInt i = 0; i < node.neighbors.size(); ++i)
  {
    AStarNodeGrid &nodeNew = *node.neighbors[i].first;
    if (edgeMap[nodeNew.cell.x][nodeNew.cell.y] >= 0)
      continue;

    if (nodeNew.closed)
      continue;

    const Real timeNew = node.cost + node.neighbors[i].second * movementSpeedRecip;

    if (!nodeNew.open)
    {
      nodeNew.open = true;
      nodeNew.cellParent = node.cell;
      nodeNew.cost = timeNew;
      updateFCostLocal(nodeNew);
      openListInsertNodeLocal(nodeNew);
    }
    else if (nodeNew.cost > timeNew)
    {
      nodeNew.cellParent = node.cell;
      nodeNew.cost = timeNew;
      updateFCostLocal(nodeNew);
      openListUpdateNodeLocal(nodeNew);
    }
  }
}

inline void DynamicAStarGridPlanner::updateFCostLocal(AStarNodeGrid &node) const
{
  Real dx = fabs((Real)node.cell.x - (Real)AStarCellGoal.x) * DynamicHeightMapConfig::resolutionSpace;
  Real dy = fabs((Real)node.cell.y - (Real)AStarCellGoal.y) * DynamicHeightMapConfig::resolutionSpace;
  node.f = node.cost + (std::min(dx, dy) * M_SQRT2 + fabs(dx - dy)) * movementSpeedRecip;
}

void DynamicAStarGridPlanner::constructAStarPathLocal()
{
  AStarNodeGrid* nodeTmp = &AStarNodes[AStarCellGoal.x][AStarCellGoal.y];
//find path length

  UInt length = 0;
  while (true)
  {
    ++length;
    if (nodeTmp->cell == AStarCellStart)
      break;
    nodeTmp = &AStarNodes[nodeTmp->cellParent.x][nodeTmp->cellParent.y];
  }

  AStarPath.points.resize(length);
  AStarPath.times.resize(length);

  UInt position = length - 1;
  nodeTmp = &AStarNodes[AStarCellGoal.x][AStarCellGoal.y];
  while (true)
  {
    AStarPath.points[position] = DynamicHeightMapConfig::getPoint(nodeTmp->cell);
    AStarPath.times[position] = nodeTmp->cost;
    if (nodeTmp->cell == AStarCellStart)
      break;
    nodeTmp = &AStarNodes[nodeTmp->cellParent.x][nodeTmp->cellParent.y];
    --position;
  }
}

void DynamicAStarGridPlanner::getSmoothTrajFromAStarPathLocal()
{
  std::vector<LineSeg2D> pathSegments;
  std::vector<ParametricFunctionCubic2D> connections;
  UInt startIndexCurrent = 0;
  PV2D startPointCurrent = AStarPath.points[0];

//find maximal free connecting way points along astar path
  for (UInt i = 1; i < AStarPath.points.size(); ++i)
  {
    if (isConnectionLineFreeLocal(startPointCurrent, AStarPath.times[startIndexCurrent], AStarPath.points[i], AStarPath.times[i]))
      continue;

    if (i - startIndexCurrent > 1)
    {
      pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPath.points[i - 1]));
      startPointCurrent = AStarPath.points[i - 1];
      startIndexCurrent = i - 1;
      --i;
    }
    else
    {
      pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPath.points[i]));
      startPointCurrent = AStarPath.points[i];
      startIndexCurrent = i;
    }
  }
  pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPath.points.back()));

  if (pathSegments.size() > 1)
  {
    //shorten path segments to lengths which are kept
    std::vector<PV2D> intersectionPoints;
    intersectionPoints.push_back(pathSegments[0].end);
    pathSegments[0].clipEnd(localSmoothingDistance);
    for (UInt i = 1; i < pathSegments.size() - 1; ++i)
    {
      intersectionPoints.push_back(pathSegments[i].end);
      pathSegments[i].clipBoth(localSmoothingDistance);
    }
    pathSegments.back().clipStart(localSmoothingDistance);

    //find smooth parametric cubic function for each intermediate way point
    for (UInt i = 0; i < pathSegments.size() - 1; ++i)
      connections.push_back(ParametricFunctionCubic2D(pathSegments[i].end, pathSegments[i + 1].start, intersectionPoints[i], localSmoothingFactor, 8));
  }

//generate points on full smoothed path
  Real sinTmp, cosTmp;
  AStarPath.pointsSmoothed.clear();
  AStarPath.anglesSmoothed.clear();
  AStarPath.directionsSmoothed.clear();
  Real distanceOnCurrent = 0.0, distanceTotal = 0.0;
  AStarPath.pointsSmoothed.push_back(pathSegments[0].start);
  AStarPath.anglesSmoothed.push_back(pathSegments[0].angle);
  AStarPath.directionsSmoothed.push_back(pathSegments[0].direction);
  for (UInt i = 0; i < pathSegments.size() - 1; ++i)
  {
    while (true)
    {
      distanceOnCurrent += localSmoothedPointDistance;
      if (distanceOnCurrent > pathSegments[i].length)
      {
        distanceOnCurrent = pathSegments[i].length - distanceOnCurrent;
        break;
      }
      AStarPath.pointsSmoothed.push_back(pathSegments[i].getPointAbsolute(distanceOnCurrent));
      AStarPath.anglesSmoothed.push_back(pathSegments[i].angle);
      AStarPath.directionsSmoothed.push_back(pathSegments[i].direction);
    }
    while (true)
    {
      distanceOnCurrent += localSmoothedPointDistance;
      if (distanceOnCurrent > connections[i].length)
      {
        distanceOnCurrent = connections[i].length - distanceOnCurrent;
        break;
      }
      AStarPath.pointsSmoothed.push_back(connections[i].getPointAbsolute(distanceOnCurrent));
      AStarPath.anglesSmoothed.push_back(connections[i].getAngleAbsolute(distanceOnCurrent));
      sincos(AStarPath.anglesSmoothed.back(), &sinTmp, &cosTmp);
      AStarPath.directionsSmoothed.push_back(PV2D(cosTmp, sinTmp));
    }
  }
  while (true)
  {
    distanceOnCurrent += localSmoothedPointDistance;
    if (distanceOnCurrent > pathSegments.back().length)
      break;
    AStarPath.pointsSmoothed.push_back(pathSegments.back().getPointAbsolute(distanceOnCurrent));
    AStarPath.anglesSmoothed.push_back(pathSegments.back().angle);
    AStarPath.directionsSmoothed.push_back(pathSegments.back().direction);
  }
}

bool DynamicAStarGridPlanner::isConnectionLineFreeLocal(const PV2D &pointStart, const Real timeStart, const PV2D &pointEnd, const Real timeEnd)
{
  PV2D direction = pointEnd - pointStart;
  const Real distanceMax = direction.norm();
  const Real distanceFactor = DynamicHeightMapConfig::resolutionSpace * 0.5;
  direction *= distanceFactor / distanceMax;
  const Real timeFactor = (timeEnd - timeStart) * distanceFactor / distanceMax;

  PV2D pointCurrent = pointStart;
  Real distanceCurrent = 0;
  Real timeCurrent = timeStart;

  while (distanceCurrent < distanceMax)
  {
    const EdgeMap &edgeMap = dynamicHeightMap->getEdgeMapConst(timeCurrent);
    distanceCurrent += distanceFactor;
    pointCurrent += direction;
    timeCurrent += timeFactor;
    const Cell2D cell = DynamicHeightMapConfig::getCell(pointCurrent);
    if (edgeMap[cell.x][cell.y] != -1)
      return false;
  }

  return true;
}

void DynamicAStarGridPlanner::openListInsertNodeLocal(AStarNodeGrid &node)
{
  openListNodes.push_back(node.cell);
  UInt index = openListNodes.size() - 1;
  UInt indexHalf = index / 2;
  node.indexOpenList = index;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void DynamicAStarGridPlanner::openListUpdateNodeLocal(AStarNodeGrid &node)
{
  UInt index = node.indexOpenList;
  UInt indexHalf = index / 2;

  while (indexHalf > 0 && AStarNodes[openListNodes[index].x][openListNodes[index].y].f < AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodes[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void DynamicAStarGridPlanner::openListRemoveFrontNodeLocal()
{
  openListNodes[1] = openListNodes.back();
  openListNodes.pop_back();
  if (openListNodes.size() > 1)
    AStarNodes[openListNodes[1].x][openListNodes[1].y].indexOpenList = 1;

  UInt index = 1, indexDouble = 2, indexDoubleOne = 3;
  while (indexDouble < openListNodes.size())
  {
    if (indexDoubleOne == openListNodes.size())
    {
      if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
        break;
    }
    else if (AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
        || AStarNodes[openListNodes[index].x][openListNodes[index].y].f > AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
    {
      if (AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
          < AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodes[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDoubleOne];
        openListNodes[indexDoubleOne] = nodeBuffer;
        AStarNodes[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDoubleOne;
        AStarNodes[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].indexOpenList = index;
        index = indexDoubleOne;
      }
    }
    else
      break;
    indexDouble = index * 2;
    indexDoubleOne = indexDouble + 1;
  }
}

//////////////////// GLOBAL PLANNING /////////////////////////////

void DynamicAStarGridPlanner::createNodesMapGlobal()
{
  AStarNodesGlobal.clear();
  AStarNodesGlobal.resize(globalMap->sizeX, std::vector<AStarNodeGrid>(globalMap->sizeY));

  for (UInt xIndex = 0; xIndex < globalMap->sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < globalMap->sizeY; ++yIndex)
    {
      AStarNodesGlobal[xIndex][yIndex].cell = Cell2D(xIndex, yIndex);

      if (xIndex + 1 < globalMap->sizeX && yIndex + 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex + 1][yIndex + 1], globalMap->resolution * M_SQRT2));
      if (xIndex + 1 < globalMap->sizeX)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex + 1][yIndex], globalMap->resolution));
      if (xIndex + 1 < globalMap->sizeX && yIndex - 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex + 1][yIndex - 1], globalMap->resolution * M_SQRT2));
      if (yIndex + 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex][yIndex + 1], globalMap->resolution));
      if (yIndex - 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex][yIndex - 1], globalMap->resolution));
      if (xIndex - 1 < globalMap->sizeX && yIndex + 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex - 1][yIndex + 1], globalMap->resolution * M_SQRT2));
      if (xIndex - 1 < globalMap->sizeX)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex - 1][yIndex], globalMap->resolution));
      if (xIndex - 1 < globalMap->sizeX && yIndex - 1 < globalMap->sizeY)
        AStarNodesGlobal[xIndex][yIndex].neighbors.push_back(std::make_pair(&AStarNodesGlobal[xIndex - 1][yIndex - 1], globalMap->resolution * M_SQRT2));
    }

  openListNodes.reserve(globalMap->sizeX * globalMap->sizeY);
}

void DynamicAStarGridPlanner::expandNodeGlobal(const AStarNodeGrid &node)
{
  const OccupancyMap &occupancyMap = globalMap->occupancyMap;

  for (UInt i = 0; i < node.neighbors.size(); ++i)
  {
    AStarNodeGrid &nodeNew = *node.neighbors[i].first;
    if (occupancyMap[nodeNew.cell.x][nodeNew.cell.y])
      continue;

    if (nodeNew.closed)
      continue;

    const Real timeNew = node.cost + node.neighbors[i].second * movementSpeedRecip;

    if (!nodeNew.open)
    {
      nodeNew.open = true;
      nodeNew.cellParent = node.cell;
      nodeNew.cost = timeNew;
      updateFCostGlobal(nodeNew);
      openListInsertNodeGlobal(nodeNew);
    }
    else if (nodeNew.cost > timeNew)
    {
      nodeNew.cellParent = node.cell;
      nodeNew.cost = timeNew;
      updateFCostGlobal(nodeNew);
      openListUpdateNodeGlobal(nodeNew);
    }
  }
}

inline void DynamicAStarGridPlanner::updateFCostGlobal(AStarNodeGrid &node) const
{
  Real dx = fabs((Real)node.cell.x - (Real)AStarCellGoal.x) * globalMap->resolution;
  Real dy = fabs((Real)node.cell.y - (Real)AStarCellGoal.y) * globalMap->resolution;
  node.f = node.cost + (std::min(dx, dy) * M_SQRT2 + fabs(dx - dy)) * movementSpeedRecip;
}

void DynamicAStarGridPlanner::constructAStarPathGlobal()
{
  AStarNodeGrid* nodeTmp = &AStarNodesGlobal[AStarCellGoal.x][AStarCellGoal.y];

  UInt length = 0;
  while (true)
  {
    ++length;
    if (nodeTmp->cell == AStarCellStart)
      break;
    nodeTmp = &AStarNodesGlobal[nodeTmp->cellParent.x][nodeTmp->cellParent.y];
  }

  AStarPathGlobal.points.resize(length);
  AStarPathGlobal.times.resize(length);

  UInt position = length - 1;
  nodeTmp = &AStarNodesGlobal[AStarCellGoal.x][AStarCellGoal.y];
  while (true)
  {
    AStarPathGlobal.points[position] = globalMap->getPoint(nodeTmp->cell);
    AStarPathGlobal.times[position] = nodeTmp->cost;
    if (nodeTmp->cell == AStarCellStart)
      break;
    nodeTmp = &AStarNodesGlobal[nodeTmp->cellParent.x][nodeTmp->cellParent.y];
    --position;
  }
}

void DynamicAStarGridPlanner::getSmoothTrajFromAStarPathGlobal()
{
  std::vector<LineSeg2D> pathSegments;
  std::vector<ParametricFunctionCubic2D> connections;
  UInt startIndexCurrent = 0;
  PV2D startPointCurrent = AStarPathGlobal.points[0];

//find maximal free connecting way points along astar path
  for (UInt i = 1; i < AStarPathGlobal.points.size(); ++i)
  {
    if (isConnectionLineFreeGlobal(startPointCurrent, AStarPathGlobal.points[i]))
      continue;

    if (i - startIndexCurrent > 1)
    {
      pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPathGlobal.points[i - 1]));
      startPointCurrent = AStarPathGlobal.points[i - 1];
      startIndexCurrent = i - 1;
      --i;
    }
    else
    {
      pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPathGlobal.points[i]));
      startPointCurrent = AStarPathGlobal.points[i];
      startIndexCurrent = i;
    }
  }
  pathSegments.push_back(LineSeg2D(startPointCurrent, AStarPathGlobal.points.back()));

  if (pathSegments.size() > 1)
  {
    //shorten path segments to lengths which are kept
    std::vector<PV2D> intersectionPoints;
    intersectionPoints.push_back(pathSegments[0].end);
    pathSegments[0].clipEnd(globalSmoothingDistance);
    for (UInt i = 1; i < pathSegments.size() - 1; ++i)
    {
      intersectionPoints.push_back(pathSegments[i].end);
      pathSegments[i].clipBoth(globalSmoothingDistance);
    }
    pathSegments.back().clipStart(globalSmoothingDistance);

    //find smooth parametric cubic function for each intermediate way point
    for (UInt i = 0; i < pathSegments.size() - 1; ++i)
      connections.push_back(ParametricFunctionCubic2D(pathSegments[i].end, pathSegments[i + 1].start, intersectionPoints[i], globalSmoothingFactor, 8));
  }

//generate points on full smoothed path
  Real sinTmp, cosTmp;
  AStarPathGlobal.pointsSmoothed.clear();
  AStarPathGlobal.anglesSmoothed.clear();
  AStarPathGlobal.directionsSmoothed.clear();
  Real distanceOnCurrent = 0.0, distanceTotal = 0.0;
  AStarPathGlobal.pointsSmoothed.push_back(pathSegments[0].start);
  AStarPathGlobal.anglesSmoothed.push_back(pathSegments[0].angle);
  AStarPathGlobal.directionsSmoothed.push_back(pathSegments[0].direction);
  for (UInt i = 0; i < pathSegments.size() - 1; ++i)
  {
    while (true)
    {
      distanceOnCurrent += globalSmoothedPointDistance;
      if (distanceOnCurrent > pathSegments[i].length)
      {
        distanceOnCurrent = pathSegments[i].length - distanceOnCurrent;
        break;
      }
      AStarPathGlobal.pointsSmoothed.push_back(pathSegments[i].getPointAbsolute(distanceOnCurrent));
      AStarPathGlobal.anglesSmoothed.push_back(pathSegments[i].angle);
      AStarPathGlobal.directionsSmoothed.push_back(pathSegments[i].direction);
    }
    while (true)
    {
      distanceOnCurrent += globalSmoothedPointDistance;
      if (distanceOnCurrent > connections[i].length)
      {
        distanceOnCurrent = connections[i].length - distanceOnCurrent;
        break;
      }
      AStarPathGlobal.pointsSmoothed.push_back(connections[i].getPointAbsolute(distanceOnCurrent));
      AStarPathGlobal.anglesSmoothed.push_back(connections[i].getAngleAbsolute(distanceOnCurrent));
      sincos(AStarPathGlobal.anglesSmoothed.back(), &sinTmp, &cosTmp);
      AStarPathGlobal.directionsSmoothed.push_back(PV2D(cosTmp, sinTmp));
    }
  }
  while (true)
  {
    distanceOnCurrent += globalSmoothedPointDistance;
    if (distanceOnCurrent > pathSegments.back().length)
      break;
    AStarPathGlobal.pointsSmoothed.push_back(pathSegments.back().getPointAbsolute(distanceOnCurrent));
    AStarPathGlobal.anglesSmoothed.push_back(pathSegments.back().angle);
    AStarPathGlobal.directionsSmoothed.push_back(pathSegments.back().direction);
  }
}

bool DynamicAStarGridPlanner::isConnectionLineFreeGlobal(const PV2D &pointStart, const PV2D &pointEnd)
{
  PV2D direction = pointEnd - pointStart;
  const Real distanceMax = direction.norm();
  const Real distanceFactor = globalMap->resolution * 0.5;
  direction *= distanceFactor / distanceMax;

  PV2D pointCurrent = pointStart;
  Real distanceCurrent = 0;

  const OccupancyMap &occupancyMap = globalMap->occupancyMap;
  while (distanceCurrent < distanceMax)
  {
    distanceCurrent += distanceFactor;
    pointCurrent += direction;
    const Cell2D cell = globalMap->getCell(pointCurrent);
    if (occupancyMap[cell.x][cell.y])
      return false;
  }

  return true;
}

void DynamicAStarGridPlanner::openListInsertNodeGlobal(AStarNodeGrid &node)
{
  openListNodes.push_back(node.cell);
  UInt index = openListNodes.size() - 1;
  UInt indexHalf = index / 2;
  node.indexOpenList = index;

  while (indexHalf > 0
      && AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].f < AStarNodesGlobal[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodesGlobal[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void DynamicAStarGridPlanner::openListUpdateNodeGlobal(AStarNodeGrid &node)
{
  UInt index = node.indexOpenList;
  UInt indexHalf = index / 2;

  while (indexHalf > 0
      && AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].f < AStarNodesGlobal[openListNodes[indexHalf].x][openListNodes[indexHalf].y].f)
  {
    Cell2D nodeBuffer = openListNodes[index];
    openListNodes[index] = openListNodes[indexHalf];
    openListNodes[indexHalf] = nodeBuffer;
    AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexHalf;
    AStarNodesGlobal[openListNodes[indexHalf].x][openListNodes[indexHalf].y].indexOpenList = index;
    index = indexHalf;
    indexHalf = index / 2;
  }
}

void DynamicAStarGridPlanner::openListRemoveFrontNodeGlobal()
{
  openListNodes[1] = openListNodes.back();
  openListNodes.pop_back();
  if (openListNodes.size() > 1)
    AStarNodesGlobal[openListNodes[1].x][openListNodes[1].y].indexOpenList = 1;

  UInt index = 1, indexDouble = 2, indexDoubleOne = 3;
  while (indexDouble < openListNodes.size())
  {
    if (indexDoubleOne == openListNodes.size())
    {
      if (AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].f > AStarNodesGlobal[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodesGlobal[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
        break;
    }
    else if (AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].f > AStarNodesGlobal[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
        || AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].f
            > AStarNodesGlobal[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
    {
      if (AStarNodesGlobal[openListNodes[indexDouble].x][openListNodes[indexDouble].y].f
          < AStarNodesGlobal[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].f)
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDouble];
        openListNodes[indexDouble] = nodeBuffer;
        AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDouble;
        AStarNodesGlobal[openListNodes[indexDouble].x][openListNodes[indexDouble].y].indexOpenList = index;
        index = indexDouble;
      }
      else
      {
        Cell2D nodeBuffer = openListNodes[index];
        openListNodes[index] = openListNodes[indexDoubleOne];
        openListNodes[indexDoubleOne] = nodeBuffer;
        AStarNodesGlobal[openListNodes[index].x][openListNodes[index].y].indexOpenList = indexDoubleOne;
        AStarNodesGlobal[openListNodes[indexDoubleOne].x][openListNodes[indexDoubleOne].y].indexOpenList = index;
        index = indexDoubleOne;
      }
    }
    else
      break;
    indexDouble = index * 2;
    indexDoubleOne = indexDouble + 1;
  }
}

//////////////////// COMMON //////////////////////////////////////

inline Real DynamicAStarGridPlanner::distanceCells(const Cell2D &cell1, const Cell2D &cell2)
{
  return sqrt(pow((Real)(cell1.x - cell2.x), 2) + pow((Real)(cell1.y - cell2.y), 2));
}
