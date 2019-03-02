//local
#include "dynamic_height_map/dynamic_height_map.h"
#include "common/progress_publisher.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

DynamicHeightMap::DynamicHeightMap() :
    globalMap(nullptr)
{
  progressMessageIndex = ProgressPublisher::addPublisher("info_height_map");
  if (!initialize())
    ros::shutdown();
}

void DynamicHeightMap::updateMap(const Matrix34 &transform, const Pose6D &robotPose)
{
  bool onlyGlobalMap;
  RosParamLoader::getRosParam(nh, "DynamicHeightMap::updateMapGazebo", "only_global_map", onlyGlobalMap, false);

  this->robotPose = robotPose;
  //update time
  if (firstFrame)
  {
    firstFrame = false;
    timeStart = ros::Time::now();
  }
  timeStampCurrent = ros::Time::now();

  resetMaps();

  if (!onlyGlobalMap)
  {
    getFrameFromDepthImageBuffer(transform);
    removeRobotFromHeightMap();
  }
  //filter height map
  for (UInt i = 0; i < 3; ++i)
    filterSingleHoles();
  for (UInt i = 0; i < 2; ++i)
    filterMedian();
  copyFilteredToPointMap();

  if (globalMap != nullptr)
    insertGlobalMap();

  //segmentation
  computeNormals();
  findSegments();
  findSegmentsAdjacentToEdge();

  //object tracking
  createNewObjects();
  findObjectMappings();
  updateObjectTrajectories();
  updateObjectIDs();

  //postprocessing
  findSegmentTypeMap();
  extractDynamicObjects();
  findEdgeMap();
  findHeightVariances();

  //prediction maps
  createPredictionMaps();

  publishObjectPredictions();
}

void DynamicHeightMap::setGlobalMap(const GlobalMap* globalMap)
{
  this->globalMap = globalMap;
}

void DynamicHeightMap::findRobotCellList(const Real radius)
{
  cellListRobot.clear();
  cellListRobot.reserve(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);

  const Real radiusSq = pow(radius, 2);

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      if (pow(pointMapAnalysis[xIndex][yIndex].C0, 2) + pow(pointMapAnalysis[xIndex][yIndex].C1, 2) <= radiusSq)
        cellListRobot.push_back(Cell2D(xIndex, yIndex));
}

const std::vector<PointMap> &DynamicHeightMap::getPointMapsConst() const
{
  return pointMapsPlanning;
}

const std::vector<SegmentTypeMap> &DynamicHeightMap::getSegmentTypeMapsConst() const
{
  return segmentTypeMapsPlanning;
}

const std::vector<EdgeMap> &DynamicHeightMap::getEdgeMapsConst() const
{
  return edgeMapsPlanning;
}

const std::vector<Object> &DynamicHeightMap::getObjectsConst() const
{
  return objects[0];
}

const PointMap &DynamicHeightMap::getPointMapConst(const Real time) const
{
  const UInt index = DynamicHeightMapConfig::getIndexT(time);
  if (index < DynamicHeightMapConfig::sizeT)
    return pointMapsPlanning[index];
  else
    return pointMapsPlanning.back();
}

const NormalMap &DynamicHeightMap::getNormalMapConst() const
{
  return normalMapAnalysis;
}

const SegmentMap &DynamicHeightMap::getSegmentMapConst(const Real time) const
{
  const UInt index = DynamicHeightMapConfig::getIndexT(time);
  if (index < DynamicHeightMapConfig::sizeT)
    return segmentMapsPlanning[index];
  else
    return segmentMapsPlanning.back();
}

const SegmentTypeMap &DynamicHeightMap::getSegmentTypeMapConst(const Real time) const
{
  const UInt index = DynamicHeightMapConfig::getIndexT(time);
  if (index < DynamicHeightMapConfig::sizeT)
    return segmentTypeMapsPlanning[index];
  else
    return segmentTypeMapsPlanning.back();
}

const EdgeMap &DynamicHeightMap::getEdgeMapConst(const Real time) const
{
  const UInt index = DynamicHeightMapConfig::getIndexT(time);
  if (index < DynamicHeightMapConfig::sizeT)
    return edgeMapsPlanning[index];
  else
    return edgeMapsPlanning.back();
}

const PointMap &DynamicHeightMap::getPointMapConst(const UInt timeIndex) const
{
  if (timeIndex < DynamicHeightMapConfig::sizeT)
    return pointMapsPlanning[timeIndex];
  else
    return pointMapsPlanning.back();
}

const SegmentMap &DynamicHeightMap::getSegmentMapConst(const UInt timeIndex) const
{
  if (timeIndex < DynamicHeightMapConfig::sizeT)
    return segmentMapsPlanning[timeIndex];
  else
    return segmentMapsPlanning.back();
}

const SegmentTypeMap &DynamicHeightMap::getSegmentTypeMapConst(const UInt timeIndex) const
{
  if (timeIndex < DynamicHeightMapConfig::sizeT)
    return segmentTypeMapsPlanning[timeIndex];
  else
    return segmentTypeMapsPlanning.back();
}

const EdgeMap &DynamicHeightMap::getEdgeMapConst(const UInt timeIndex) const
{
  if (timeIndex < DynamicHeightMapConfig::sizeT)
    return edgeMapsPlanning[timeIndex];
  else
    return edgeMapsPlanning.back();
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// INITIALIZATION //////////////////////////////

bool DynamicHeightMap::initialize()
{
  ProgressPublisher::showMessage(progressMessageIndex, "Initializing...");
  DynamicHeightMapConfig::initialize(nh);
  CameraConfig::initialize(nh);

  initializeMaps();
  initializeUnitImage();

  subscriberDepthImage = nh.subscribe("/camera/depth/image_raw", 0, &DynamicHeightMap::subscriberDepthImageHandler, this);
  publisherObjectPredictions = nh.advertise<visualization_msgs::Marker>("object_predictions", 1);

  firstFrame = true;
  objectIDNext = 0;

  ProgressPublisher::showMessage(progressMessageIndex, "Done!", 3.0);
  return true;
}

void DynamicHeightMap::initializeMaps()
{
  pointMapAnalysis.resize(DynamicHeightMapConfig::sizeX, std::vector<PV3D>(DynamicHeightMapConfig::sizeY, PV3D(0.0, 0.0, 0.0)));
  normalMapAnalysis.resize(DynamicHeightMapConfig::sizeX, std::vector<PV3D>(DynamicHeightMapConfig::sizeY, PV3D(0.0, 0.0, 0.0)));
  segmentMapAnalysis.resize(DynamicHeightMapConfig::sizeX, std::vector<UInt>(DynamicHeightMapConfig::sizeY, SEGMENT_UNKNOWN));
  segmentTypeMapAnalysis.resize(DynamicHeightMapConfig::sizeX, std::vector<SegmentType>(DynamicHeightMapConfig::sizeY, SegmentType::None));
  edgeMapAnalysis.resize(DynamicHeightMapConfig::sizeX, std::vector<Int>(DynamicHeightMapConfig::sizeY, -1));

  pointMapsPlanning.resize(
      DynamicHeightMapConfig::sizeT,
      std::vector<std::vector<PV3D> >(DynamicHeightMapConfig::sizeX, std::vector<PV3D>(DynamicHeightMapConfig::sizeY, PV3D(0.0, 0.0, 0.0))));
  segmentTypeMapsPlanning.resize(
      DynamicHeightMapConfig::sizeT,
      std::vector<std::vector<SegmentType> >(DynamicHeightMapConfig::sizeX, std::vector<SegmentType>(DynamicHeightMapConfig::sizeY, SegmentType::None)));
  edgeMapsPlanning.resize(DynamicHeightMapConfig::sizeT,
                          std::vector<std::vector<Int> >(DynamicHeightMapConfig::sizeX, std::vector<Int>(DynamicHeightMapConfig::sizeY, -1)));
  segmentMapsPlanning.resize(DynamicHeightMapConfig::sizeT,
                             std::vector<std::vector<UInt> >(DynamicHeightMapConfig::sizeX, std::vector<UInt>(DynamicHeightMapConfig::sizeY, SEGMENT_UNKNOWN)));

  pointsInitial.resize(DynamicHeightMapConfig::sizeX, std::vector<Real>(DynamicHeightMapConfig::sizeY, 0.0));
  heightsFiltering.current->resize(DynamicHeightMapConfig::sizeX, std::vector<Real>(DynamicHeightMapConfig::sizeY, 0.0));
  heightsFiltering.next->resize(DynamicHeightMapConfig::sizeX, std::vector<Real>(DynamicHeightMapConfig::sizeY, 0.0));

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      pointMapAnalysis[xIndex][yIndex].C0 = DynamicHeightMapConfig::minX + (xIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace;
      pointMapAnalysis[xIndex][yIndex].C1 = DynamicHeightMapConfig::minY + (yIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace;
    }

  for (UInt tIndex = 0; tIndex < DynamicHeightMapConfig::sizeT; ++tIndex)
    for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
      for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      {
        pointMapsPlanning[tIndex][xIndex][yIndex].C0 = DynamicHeightMapConfig::minX + (xIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace;
        pointMapsPlanning[tIndex][xIndex][yIndex].C1 = DynamicHeightMapConfig::minY + (yIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace;
      }

  depthImagePointBuffer.resize(CameraConfig::height, std::vector<PV3D>(CameraConfig::width, PV3D()));
  objects.resize(DynamicHeightMapConfig::mapHistoryCount);

  segmentCellLists.resize(DynamicHeightMapConfig::maxSegmentsPerMap);
  for (UInt i = 0; i < segmentCellLists.size(); ++i)
    segmentCellLists[i].reserve(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);
}

void DynamicHeightMap::initializeUnitImage()
{
  unitImage.resize(CameraConfig::height, std::vector<PV3D>(CameraConfig::width, PV3D(1, 0, 0)));
  Real cellSizeHeight = 2 * tan(0.5 * CameraConfig::openingAngleHeight * TORAD) / (Real)(CameraConfig::height - 1);
  Real cellSizeWidth = 2 * tan(0.5 * CameraConfig::openingAngleWidth * TORAD) / (Real)(CameraConfig::width - 1);
  Real heightMax = tan(0.5 * CameraConfig::openingAngleHeight * TORAD);
  Real widthMax = tan(0.5 * CameraConfig::openingAngleWidth * TORAD);

  for (UInt height = 0; height < CameraConfig::height; ++height)
    for (UInt width = 0; width < CameraConfig::width; ++width)
    {
      unitImage[height][width].C1 = widthMax - width * cellSizeWidth;
      unitImage[height][width].C2 = heightMax - height * cellSizeHeight;
    }
}

//////////////////// MAP ANALYSIS ////////////////////////////////

void DynamicHeightMap::removeRobotFromHeightMap()
{
  HeightMap &heightMap = *heightsFiltering.current;
  for (Cell2DIt it = cellListRobot.begin(); it != cellListRobot.end(); ++it)
    heightMap[it->x][it->y] = HEIGHT_UNKNOWN;
}

void DynamicHeightMap::insertGlobalMap()
{
  Real cosA, sinA;
  sincos(robotPose.yaw, &sinA, &cosA);

//  segments.push_back(Segment());
//  Segment &segment = segments.back();
//  segment.type = SegmentType::Static;
//  const UInt segmentID = segments.size() - 1;
//  segment.ID = segmentID;
//  segment.cellList = &segmentCellLists[segmentID];
//  Cell2DList &cellList = *segment.cellList;
//  cellList.clear();

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      const Real x = pointMapAnalysis[xIndex][yIndex].C0 * cosA - pointMapAnalysis[xIndex][yIndex].C1 * sinA + robotPose.x;
      const Real y = pointMapAnalysis[xIndex][yIndex].C0 * sinA + pointMapAnalysis[xIndex][yIndex].C1 * cosA + robotPose.y;
      const Real height = globalMap->getHeight(x, y);

      if (pointMapAnalysis[xIndex][yIndex].C2 == HEIGHT_UNKNOWN)
        pointMapAnalysis[xIndex][yIndex].C2 = height - robotPose.z;

//      if (height != 0.0)
//      {
//        segmentMapAnalysis[xIndex][yIndex] = segmentID;
//        cellList.push_back(Cell2D(xIndex, yIndex));
//      }
    }
}

void DynamicHeightMap::filterSingleHoles()
{
  HeightMap &raw = *heightsFiltering.current;
  HeightMap &filtered = *heightsFiltering.next;

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
  {
    filtered[xIndex][0] = raw[xIndex][0];
    filtered[xIndex].back() = raw[xIndex].back();
  }

  for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
  {
    filtered[0][yIndex] = raw[0][yIndex];
    filtered.back()[yIndex] = raw.back()[yIndex];
  }

#ifdef OPENMP_MAP
#pragma omp parallel for num_threads(6)
#endif
  for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
    for (UInt yIndex = 1; yIndex < DynamicHeightMapConfig::sizeYMinus1; ++yIndex)
    {
      if (raw[xIndex][yIndex] != HEIGHT_UNKNOWN)
        filtered[xIndex][yIndex] = raw[xIndex][yIndex];
      else
      {
        Real height = 0.0;
        UInt heightCount = 0;

        if (raw[xIndex - 1][yIndex] != HEIGHT_UNKNOWN)
        {
          height += raw[xIndex - 1][yIndex];
          ++heightCount;
        }
        if (raw[xIndex + 1][yIndex] != HEIGHT_UNKNOWN)
        {
          height += raw[xIndex + 1][yIndex];
          ++heightCount;
        }
        if (raw[xIndex][yIndex - 1] != HEIGHT_UNKNOWN)
        {
          height += raw[xIndex][yIndex - 1];
          ++heightCount;
        }
        if (raw[xIndex][yIndex + 1] != HEIGHT_UNKNOWN)
        {
          height += raw[xIndex][yIndex + 1];
          ++heightCount;
        }

        if (heightCount >= 3)
          filtered[xIndex][yIndex] = height / heightCount;
        else
          filtered[xIndex][yIndex] = HEIGHT_UNKNOWN;
      }
    }

  heightsFiltering.toggle();
}

void DynamicHeightMap::filterMedian()
{
  const HeightMap &raw = *heightsFiltering.current;
  HeightMap &filtered = *heightsFiltering.next;

  for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
  {
    filtered[0][yIndex] = raw[0][yIndex];
    filtered[DynamicHeightMapConfig::sizeXMinus1][yIndex] = raw[DynamicHeightMapConfig::sizeXMinus1][yIndex];
  }

  for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
  {
    filtered[xIndex][0] = raw[xIndex][0];
    filtered[xIndex][DynamicHeightMapConfig::sizeYMinus1] = raw[xIndex][DynamicHeightMapConfig::sizeYMinus1];
  }

#ifdef OPENMP_MAP
#pragma omp parallel num_threads(6)
#endif
  {
    std::vector<Real> vec(9);
#ifdef OPENMP_MAP
#pragma omp for
#endif
    for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
    {
      const std::vector<Real> &rawRowPrev = raw[xIndex - 1];
      const std::vector<Real> &rawRowSame = raw[xIndex];
      const std::vector<Real> &rawRowNext = raw[xIndex + 1];
      std::vector<Real> &filteredRow = filtered[xIndex];

      for (UInt yIndex = 1; yIndex < DynamicHeightMapConfig::sizeYMinus1; ++yIndex)
      {
        if (yIndex != DynamicHeightMapConfig::sizeYMinus2
            && (rawRowPrev[yIndex + 1] == HEIGHT_UNKNOWN || rawRowSame[yIndex + 1] == HEIGHT_UNKNOWN || rawRowNext[yIndex + 1] == HEIGHT_UNKNOWN))
        {
          filteredRow[yIndex] = rawRowSame[yIndex];
          filteredRow[yIndex + 1] = rawRowSame[yIndex + 1];
          filteredRow[yIndex + 2] = rawRowSame[yIndex + 2];
          yIndex += 2;
          continue;
        }

        if (rawRowPrev[yIndex] == HEIGHT_UNKNOWN || rawRowSame[yIndex] == HEIGHT_UNKNOWN || rawRowNext[yIndex] == HEIGHT_UNKNOWN)
        {
          filteredRow[yIndex] = rawRowSame[yIndex];
          filteredRow[yIndex + 1] = rawRowSame[yIndex + 1];
          ++yIndex;
          continue;
        }

        if (rawRowPrev[yIndex - 1] == HEIGHT_UNKNOWN || rawRowSame[yIndex - 1] == HEIGHT_UNKNOWN || rawRowNext[yIndex - 1] == HEIGHT_UNKNOWN)
        {
          filteredRow[yIndex] = rawRowSame[yIndex];
          continue;
        }

        //median finding according to mahmoodi's algorithm
        vec[0] = rawRowPrev[yIndex - 1];
        vec[1] = rawRowPrev[yIndex];
        vec[2] = rawRowPrev[yIndex + 1];
        vec[3] = rawRowSame[yIndex - 1];
        vec[4] = rawRowSame[yIndex];
        vec[5] = rawRowSame[yIndex + 1];
        vec[6] = rawRowNext[yIndex - 1];
        vec[7] = rawRowNext[yIndex];
        vec[8] = rawRowNext[yIndex + 1];

        filteredRow[yIndex] = getMedian(vec);
      }
    }
  }

  heightsFiltering.toggle();
}

void DynamicHeightMap::computeNormals()
{
#ifdef OPENMP_MAP
#pragma omp parallel for num_threads(6)
#endif
  for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
    for (UInt yIndex = 1; yIndex < DynamicHeightMapConfig::sizeYMinus1; ++yIndex)
    {
      PV3D &pv = normalMapAnalysis[xIndex][yIndex];
      pv.C0 = pv.C1 = 0.0;

      pv.C0 += pointMapAnalysis[xIndex - 1][yIndex - 1].C2;
      pv.C0 += pointMapAnalysis[xIndex - 1][yIndex].C2;
      pv.C0 += pointMapAnalysis[xIndex - 1][yIndex + 1].C2;
      pv.C0 -= pointMapAnalysis[xIndex + 1][yIndex - 1].C2;
      pv.C0 -= pointMapAnalysis[xIndex + 1][yIndex].C2;
      pv.C0 -= pointMapAnalysis[xIndex + 1][yIndex + 1].C2;

      pv.C1 += pointMapAnalysis[xIndex - 1][yIndex - 1].C2;
      pv.C1 += pointMapAnalysis[xIndex][yIndex - 1].C2;
      pv.C1 += pointMapAnalysis[xIndex + 1][yIndex - 1].C2;
      pv.C1 -= pointMapAnalysis[xIndex - 1][yIndex + 1].C2;
      pv.C1 -= pointMapAnalysis[xIndex][yIndex + 1].C2;
      pv.C1 -= pointMapAnalysis[xIndex + 1][yIndex + 1].C2;

      pv.C2 = DynamicHeightMapConfig::normalFactor;

      pv.normalize();
    }
}

void DynamicHeightMap::findSegments()
{
  const PV3D vertical(0.0, 0.0, 1.0);

//first pass finds all planar segments
  for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
    for (UInt yIndex = 1; yIndex < DynamicHeightMapConfig::sizeYMinus1; ++yIndex)
    {
      if (segmentMapAnalysis[xIndex][yIndex] != SEGMENT_UNKNOWN || pointMapAnalysis[xIndex][yIndex].C2 == HEIGHT_UNKNOWN
          || normalMapAnalysis[xIndex][yIndex] * vertical < DynamicHeightMapConfig::segmentationCosMinVertical
          || normalMapAnalysis[xIndex][yIndex] * normalMapAnalysis[xIndex - 1][yIndex] < DynamicHeightMapConfig::segmentationCosMinDeviation
          || normalMapAnalysis[xIndex][yIndex] * normalMapAnalysis[xIndex + 1][yIndex] < DynamicHeightMapConfig::segmentationCosMinDeviation
          || normalMapAnalysis[xIndex][yIndex] * normalMapAnalysis[xIndex][yIndex - 1] < DynamicHeightMapConfig::segmentationCosMinDeviation
          || normalMapAnalysis[xIndex][yIndex] * normalMapAnalysis[xIndex][yIndex + 1] < DynamicHeightMapConfig::segmentationCosMinDeviation)
        continue;

      findNewSegmentPlanar(Cell2D(xIndex, yIndex));
    }

  for (UInt i = 0; i < segments.size(); ++i)
  {
    if (getSegmentDistanceDeviation(segments[i]) < 0.01)
      continue;

    segments[i].type = SegmentType::Nonplanar;
    continuePlanarSegmentAsNonPlanar(i);
  }

//second pass finds all remaining connected segments and marks them as non planar
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      if (segmentMapAnalysis[xIndex][yIndex] != SEGMENT_UNKNOWN || pointMapAnalysis[xIndex][yIndex].C2 == HEIGHT_UNKNOWN)
        continue;

      findNewSegmentNonPlanar(Cell2D(xIndex, yIndex));
    }

//third pass finds shadows
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      if (segmentMapAnalysis[xIndex][yIndex] != SEGMENT_UNKNOWN || pointMapAnalysis[xIndex][yIndex].C2 != HEIGHT_UNKNOWN)
        continue;

      findAndReplaceShadow(Cell2D(xIndex, yIndex));
    }
}

void DynamicHeightMap::findNewSegmentPlanar(const Cell2D &cellStart)
{
  segments.push_back(Segment());
  Segment &segment = segments.back();
  segment.type = SegmentType::Planar;
  const UInt segmentID = segments.size() - 1;
  segment.ID = segmentID;
  segment.cellList = &segmentCellLists[segmentID];
  Cell2DList* cellList = segment.cellList;
  cellList->clear();
  cellList->push_back(cellStart);

  segmentMapAnalysis[cellStart.x][cellStart.y] = segmentID;
  segment.heightAverage = pointMapAnalysis[cellStart.x][cellStart.y].C2;

  Matrix33 matrix(0.0);
  PV3D vector(0.0);
  matrix.C00 = pow(pointMapAnalysis[cellStart.x][cellStart.y].C0, 2);
  matrix.C11 = pow(pointMapAnalysis[cellStart.x][cellStart.y].C1, 2);
  matrix.C01 = pointMapAnalysis[cellStart.x][cellStart.y].C0 * pointMapAnalysis[cellStart.x][cellStart.y].C1;
  matrix.C02 = pointMapAnalysis[cellStart.x][cellStart.y].C0;
  matrix.C12 = pointMapAnalysis[cellStart.x][cellStart.y].C1;
  matrix.C22 = 1.0;
  vector.C0 = -pointMapAnalysis[cellStart.x][cellStart.y].C0 * pointMapAnalysis[cellStart.x][cellStart.y].C2;
  vector.C1 = -pointMapAnalysis[cellStart.x][cellStart.y].C1 * pointMapAnalysis[cellStart.x][cellStart.y].C2;
  vector.C2 = -pointMapAnalysis[cellStart.x][cellStart.y].C2;

  std::deque<Cell2D> cellsEdge;
  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList->size(); ++cellIndex)
  {
    const Cell2D &cell = (*cellList)[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      if (normalMapAnalysis[cell.x][cell.y] * normalMapAnalysis[indexTmp][cell.y] > DynamicHeightMapConfig::segmentationCosMinDeviation)
      {
        cellList->push_back(Cell2D(indexTmp, cell.y));

        matrix.C00 += pow(pointMapAnalysis[indexTmp][cell.y].C0, 2);
        matrix.C11 += pow(pointMapAnalysis[indexTmp][cell.y].C1, 2);
        matrix.C01 += pointMapAnalysis[indexTmp][cell.y].C0 * pointMapAnalysis[indexTmp][cell.y].C1;
        matrix.C02 += pointMapAnalysis[indexTmp][cell.y].C0;
        matrix.C12 += pointMapAnalysis[indexTmp][cell.y].C1;
        ++matrix.C22;
        vector.C0 -= pointMapAnalysis[indexTmp][cell.y].C0 * pointMapAnalysis[indexTmp][cell.y].C2;
        vector.C1 -= pointMapAnalysis[indexTmp][cell.y].C1 * pointMapAnalysis[indexTmp][cell.y].C2;
        vector.C2 -= pointMapAnalysis[indexTmp][cell.y].C2;

        segmentMapAnalysis[indexTmp][cell.y] = segmentID;
        segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
        segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
        segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
      }
      else
        cellsEdge.push_back(Cell2D(indexTmp, cell.y));
    }

    indexTmp = cell.x + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      if (normalMapAnalysis[cell.x][cell.y] * normalMapAnalysis[indexTmp][cell.y] > DynamicHeightMapConfig::segmentationCosMinDeviation)
      {
        cellList->push_back(Cell2D(indexTmp, cell.y));

        matrix.C00 += pow(pointMapAnalysis[indexTmp][cell.y].C0, 2);
        matrix.C11 += pow(pointMapAnalysis[indexTmp][cell.y].C1, 2);
        matrix.C01 += pointMapAnalysis[indexTmp][cell.y].C0 * pointMapAnalysis[indexTmp][cell.y].C1;
        matrix.C02 += pointMapAnalysis[indexTmp][cell.y].C0;
        matrix.C12 += pointMapAnalysis[indexTmp][cell.y].C1;
        ++matrix.C22;
        vector.C0 -= pointMapAnalysis[indexTmp][cell.y].C0 * pointMapAnalysis[indexTmp][cell.y].C2;
        vector.C1 -= pointMapAnalysis[indexTmp][cell.y].C1 * pointMapAnalysis[indexTmp][cell.y].C2;
        vector.C2 -= pointMapAnalysis[indexTmp][cell.y].C2;

        segmentMapAnalysis[indexTmp][cell.y] = segmentID;
        segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
        segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
        segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
      }
      else
        cellsEdge.push_back(Cell2D(indexTmp, cell.y));
    }

    indexTmp = cell.y - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      if (normalMapAnalysis[cell.x][cell.y] * normalMapAnalysis[cell.x][indexTmp] > DynamicHeightMapConfig::segmentationCosMinDeviation)
      {
        cellList->push_back(Cell2D(cell.x, indexTmp));

        matrix.C00 += pow(pointMapAnalysis[cell.x][indexTmp].C0, 2);
        matrix.C11 += pow(pointMapAnalysis[cell.x][indexTmp].C1, 2);
        matrix.C01 += pointMapAnalysis[cell.x][indexTmp].C0 * pointMapAnalysis[cell.x][indexTmp].C1;
        matrix.C02 += pointMapAnalysis[cell.x][indexTmp].C0;
        matrix.C12 += pointMapAnalysis[cell.x][indexTmp].C1;
        ++matrix.C22;
        vector.C0 -= pointMapAnalysis[cell.x][indexTmp].C0 * pointMapAnalysis[cell.x][indexTmp].C2;
        vector.C1 -= pointMapAnalysis[cell.x][indexTmp].C1 * pointMapAnalysis[cell.x][indexTmp].C2;
        vector.C2 -= pointMapAnalysis[cell.x][indexTmp].C2;

        segmentMapAnalysis[cell.x][indexTmp] = segmentID;
        segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
        segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
        segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
      }
      else
        cellsEdge.push_back(Cell2D(cell.x, indexTmp));
    }

    indexTmp = cell.y + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      if (normalMapAnalysis[cell.x][cell.y] * normalMapAnalysis[cell.x][indexTmp] > DynamicHeightMapConfig::segmentationCosMinDeviation)
      {
        cellList->push_back(Cell2D(cell.x, indexTmp));

        matrix.C00 += pow(pointMapAnalysis[cell.x][indexTmp].C0, 2);
        matrix.C11 += pow(pointMapAnalysis[cell.x][indexTmp].C1, 2);
        matrix.C01 += pointMapAnalysis[cell.x][indexTmp].C0 * pointMapAnalysis[cell.x][indexTmp].C1;
        matrix.C02 += pointMapAnalysis[cell.x][indexTmp].C0;
        matrix.C12 += pointMapAnalysis[cell.x][indexTmp].C1;
        ++matrix.C22;
        vector.C0 -= pointMapAnalysis[cell.x][indexTmp].C0 * pointMapAnalysis[cell.x][indexTmp].C2;
        vector.C1 -= pointMapAnalysis[cell.x][indexTmp].C1 * pointMapAnalysis[cell.x][indexTmp].C2;
        vector.C2 -= pointMapAnalysis[cell.x][indexTmp].C2;

        segmentMapAnalysis[cell.x][indexTmp] = segmentID;
        segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
        segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
        segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
      }
      else
        cellsEdge.push_back(Cell2D(cell.x, indexTmp));
    }
  }

  matrix.C10 = matrix.C01;
  matrix.C20 = matrix.C02;
  matrix.C21 = matrix.C12;

  Matrix33 matInv;
  matrix.inverse(matInv);
  PV3D pv = matInv * vector;
  segment.normal.C0 = pv.C0;
  segment.normal.C1 = pv.C1;
  segment.normal.C2 = 1.0;
  segment.normal.normalize();
  segment.distance = -pv.C2;

//add edge cells to segment if they are close enough to fittet plane
  while (cellsEdge.size() > 0)
  {
    const Cell2D &cellEdge = cellsEdge.front();
    if (segmentMapAnalysis[cellEdge.x][cellEdge.y] == SEGMENT_UNKNOWN && pointMapAnalysis[cellEdge.x][cellEdge.y].C2 != HEIGHT_UNKNOWN
        && fabs(
            segment.normal.C0 * pointMapAnalysis[cellEdge.x][cellEdge.y].C0 + segment.normal.C1 * pointMapAnalysis[cellEdge.x][cellEdge.y].C1
                + segment.normal.C2 * pointMapAnalysis[cellEdge.x][cellEdge.y].C2 - segment.distance) < DynamicHeightMapConfig::segmentationMaxSegmentDistance)
    {
      segmentMapAnalysis[cellEdge.x][cellEdge.y] = segmentID;
      cellList->push_back(Cell2D(cellEdge.x, cellEdge.y));
      segment.position.C0 += pointMapAnalysis[cellEdge.x][cellEdge.y].C0;
      segment.position.C1 += pointMapAnalysis[cellEdge.x][cellEdge.y].C1;
      segment.heightAverage += pointMapAnalysis[cellEdge.x][cellEdge.y].C2;
      cellsEdge.push_back(Cell2D(cellEdge.x, cellEdge.y));
    }
    indexTmp = cellEdge.x - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cellEdge.y] == SEGMENT_UNKNOWN
        && pointMapAnalysis[indexTmp][cellEdge.y].C2 != HEIGHT_UNKNOWN
        && fabs(
            segment.normal.C0 * pointMapAnalysis[indexTmp][cellEdge.y].C0 + segment.normal.C1 * pointMapAnalysis[indexTmp][cellEdge.y].C1
                + segment.normal.C2 * pointMapAnalysis[indexTmp][cellEdge.y].C2 - segment.distance) < DynamicHeightMapConfig::segmentationMaxSegmentDistance)
    {
      segmentMapAnalysis[indexTmp][cellEdge.y] = segmentID;
      cellList->push_back(Cell2D(indexTmp, cellEdge.y));
      segment.position.C0 += pointMapAnalysis[indexTmp][cellEdge.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cellEdge.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cellEdge.y].C2;
      cellsEdge.push_back(Cell2D(indexTmp, cellEdge.y));
    }
    indexTmp = cellEdge.x + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cellEdge.y] == SEGMENT_UNKNOWN
        && pointMapAnalysis[indexTmp][cellEdge.y].C2 != HEIGHT_UNKNOWN
        && fabs(
            segment.normal.C0 * pointMapAnalysis[indexTmp][cellEdge.y].C0 + segment.normal.C1 * pointMapAnalysis[indexTmp][cellEdge.y].C1
                + segment.normal.C2 * pointMapAnalysis[indexTmp][cellEdge.y].C2 - segment.distance) < DynamicHeightMapConfig::segmentationMaxSegmentDistance)
    {
      segmentMapAnalysis[indexTmp][cellEdge.y] = segmentID;
      cellList->push_back(Cell2D(indexTmp, cellEdge.y));
      segment.position.C0 += pointMapAnalysis[indexTmp][cellEdge.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cellEdge.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cellEdge.y].C2;
      cellsEdge.push_back(Cell2D(indexTmp, cellEdge.y));
    }
    indexTmp = cellEdge.y - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cellEdge.x][indexTmp] == SEGMENT_UNKNOWN
        && pointMapAnalysis[cellEdge.x][indexTmp].C2 != HEIGHT_UNKNOWN
        && fabs(
            segment.normal.C0 * pointMapAnalysis[cellEdge.x][indexTmp].C0 + segment.normal.C1 * pointMapAnalysis[cellEdge.x][indexTmp].C1
                + segment.normal.C2 * pointMapAnalysis[cellEdge.x][indexTmp].C2 - segment.distance) < DynamicHeightMapConfig::segmentationMaxSegmentDistance)
    {
      segmentMapAnalysis[cellEdge.x][indexTmp] = segmentID;
      cellList->push_back(Cell2D(cellEdge.x, indexTmp));
      segment.position.C0 += pointMapAnalysis[cellEdge.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cellEdge.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cellEdge.x][indexTmp].C2;
      cellsEdge.push_back(Cell2D(cellEdge.x, indexTmp));
    }
    indexTmp = cellEdge.y + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cellEdge.x][indexTmp] == SEGMENT_UNKNOWN
        && pointMapAnalysis[cellEdge.x][indexTmp].C2 != HEIGHT_UNKNOWN
        && fabs(
            segment.normal.C0 * pointMapAnalysis[cellEdge.x][indexTmp].C0 + segment.normal.C1 * pointMapAnalysis[cellEdge.x][indexTmp].C1
                + segment.normal.C2 * pointMapAnalysis[cellEdge.x][indexTmp].C2 - segment.distance) < DynamicHeightMapConfig::segmentationMaxSegmentDistance)
    {
      segmentMapAnalysis[cellEdge.x][indexTmp] = segmentID;
      cellList->push_back(Cell2D(cellEdge.x, indexTmp));
      segment.position.C0 += pointMapAnalysis[cellEdge.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cellEdge.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cellEdge.x][indexTmp].C2;
      cellsEdge.push_back(Cell2D(cellEdge.x, indexTmp));
    }

    cellsEdge.pop_front();
  }
  segment.position /= (Real)cellList->size();
  segment.heightAverage /= (Real)cellList->size();
}

void DynamicHeightMap::continuePlanarSegmentAsNonPlanar(const UInt segmentID)
{
  Segment &segment = segments[segmentID];
  Cell2DList &cellList = *segment.cellList;

  segment.position *= (Real)cellList.size();
  segment.heightAverage *= (Real)cellList.size();

  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList.size(); ++cellIndex)
  {
    const Cell2D &cell = cellList[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(indexTmp, cell.y));
      segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
    }

    indexTmp = cell.x + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(indexTmp, cell.y));
      segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
    }

    indexTmp = cell.y - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(cell.x, indexTmp));
      segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
    }

    indexTmp = cell.y + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(cell.x, indexTmp));
      segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
    }
  }
  segment.position /= (Real)cellList.size();
  segment.heightAverage /= (Real)cellList.size();
}

void DynamicHeightMap::findNewSegmentNonPlanar(const Cell2D &cellStart)
{
  segments.push_back(Segment());
  Segment &segment = segments.back();
  segment.type = SegmentType::Nonplanar;
  UInt segmentID = segments.size() - 1;
  segment.cellList = &segmentCellLists[segmentID];
  segment.ID = segmentID;
  Cell2DList &cellList = *segment.cellList;
  cellList.clear();
  cellList.push_back(cellStart);

  segmentMapAnalysis[cellStart.x][cellStart.y] = segmentID;
  segment.heightAverage = pointMapAnalysis[cellStart.x][cellStart.y].C2;

  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList.size(); ++cellIndex)
  {
    const Cell2D &cell = cellList[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(indexTmp, cell.y));
      segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
    }

    indexTmp = cell.x + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeX&& segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN && pointMapAnalysis[indexTmp][cell.y].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(indexTmp, cell.y));
      segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      segment.position.C0 += pointMapAnalysis[indexTmp][cell.y].C0;
      segment.position.C1 += pointMapAnalysis[indexTmp][cell.y].C1;
      segment.heightAverage += pointMapAnalysis[indexTmp][cell.y].C2;
    }

    indexTmp = cell.y - 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(cell.x, indexTmp));
      segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
    }

    indexTmp = cell.y + 1;
    if (indexTmp
        < DynamicHeightMapConfig::sizeY&& segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN && pointMapAnalysis[cell.x][indexTmp].C2 != HEIGHT_UNKNOWN)
    {
      cellList.push_back(Cell2D(cell.x, indexTmp));
      segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      segment.position.C0 += pointMapAnalysis[cell.x][indexTmp].C0;
      segment.position.C1 += pointMapAnalysis[cell.x][indexTmp].C1;
      segment.heightAverage += pointMapAnalysis[cell.x][indexTmp].C2;
    }
  }
//  if (cellList.size() < 15)
//  {
//    UInt adjacentPlanarSegmentID = findAdjacentPlanarSegmentID(segmentID);
//
//    if (adjacentPlanarSegmentID != SEGMENT_UNKNOWN)
//    {
//      Cell2DList &cellListNew = *segments[adjacentPlanarSegmentID].cellList;
//
//      for (UInt i = 0; i < cellList.size(); ++i)
//      {
//        const Cell2D &cell = cellList[i];
//        cellListNew.push_back(cell);
//        segmentMapAnalysis[cell.x][cell.y] = adjacentPlanarSegmentID;
//      }
//      segments.pop_back();
//      --segmentID;
//      return;
//    }
//  }

  segment.position /= (Real)cellList.size();
  segment.heightAverage /= (Real)cellList.size();
}

void DynamicHeightMap::findAndReplaceShadow(const Cell2D &cellStart)
{
  segments.push_back(Segment());
  Segment &segment = segments.back();
  segment.reset();
  segment.type = SegmentType::Shadow;
  UInt segmentID = segments.size() - 1;
  segment.cellList = &segmentCellLists[segmentID];
  Cell2DList &cellList = *segment.cellList;
  cellList.clear();
  cellList.push_back(cellStart);

  segmentMapAnalysis[cellStart.x][cellStart.y] = segmentID;

  std::vector<UInt> segmentMapping(segments.size() - 1, 0);

  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList.size(); ++cellIndex)
  {
    const Cell2D &cell = cellList[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX)
    {
      if (segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN)
      {
        cellList.push_back(Cell2D(indexTmp, cell.y));
        segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      }
      else if (segmentMapAnalysis[indexTmp][cell.y] != segmentID && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];
    }

    indexTmp = cell.x + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX)
    {
      if (segmentMapAnalysis[indexTmp][cell.y] == SEGMENT_UNKNOWN)
      {
        cellList.push_back(Cell2D(indexTmp, cell.y));
        segmentMapAnalysis[indexTmp][cell.y] = segmentID;
      }
      else if (segmentMapAnalysis[indexTmp][cell.y] != segmentID && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];
    }

    indexTmp = cell.y - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY)
    {
      if (segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN)
      {
        cellList.push_back(Cell2D(cell.x, indexTmp));
        segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      }
      else if (segmentMapAnalysis[cell.x][indexTmp] != segmentID && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];
    }

    indexTmp = cell.y + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY)
    {
      if (segmentMapAnalysis[cell.x][indexTmp] == SEGMENT_UNKNOWN)
      {
        cellList.push_back(Cell2D(cell.x, indexTmp));
        segmentMapAnalysis[cell.x][indexTmp] = segmentID;
      }
      else if (segmentMapAnalysis[cell.x][indexTmp] != segmentID && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];
    }
  }
}

UInt DynamicHeightMap::findAdjacentPlanarSegmentID(const UInt segmentID) const
{
  const Segment &segment = segments[segmentID];
  const Cell2DList &cellList = *segment.cellList;
  std::vector<UInt> segmentMapping(segments.size(), 0);

  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList.size(); ++cellIndex)
  {
    const Cell2D &cell = cellList[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cell.y] != segmentID && segmentMapAnalysis[indexTmp][cell.y] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Planar)
      ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

    indexTmp = cell.x + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cell.y] != segmentID && segmentMapAnalysis[indexTmp][cell.y] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Planar)
      ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

    indexTmp = cell.y - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cell.x][indexTmp] != segmentID && segmentMapAnalysis[cell.x][indexTmp] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Planar)
      ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];

    indexTmp = cell.y + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cell.x][indexTmp] != segmentID && segmentMapAnalysis[cell.x][indexTmp] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Planar)
      ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];
  }

  UInt indexMax = SEGMENT_UNKNOWN;
  getIndexMax(segmentMapping, indexMax);

  return indexMax;
}

UInt DynamicHeightMap::findAdjacentNonplanarSegmentID(const UInt segmentID) const
{
  const Segment &segment = segments[segmentID];
  const Cell2DList &cellList = *segment.cellList;
  std::vector<UInt> segmentMapping(segments.size(), 0);

  UInt indexTmp;
  for (UInt cellIndex = 0; cellIndex < cellList.size(); ++cellIndex)
  {
    const Cell2D &cell = cellList[cellIndex];

    indexTmp = cell.x - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cell.y] != segmentID && segmentMapAnalysis[indexTmp][cell.y] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Nonplanar)
      ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

    indexTmp = cell.x + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeX && segmentMapAnalysis[indexTmp][cell.y] != segmentID && segmentMapAnalysis[indexTmp][cell.y] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[indexTmp][cell.y]].type == SegmentType::Nonplanar)
      ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

    indexTmp = cell.y - 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cell.x][indexTmp] != segmentID && segmentMapAnalysis[cell.x][indexTmp] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Nonplanar)
      ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];

    indexTmp = cell.y + 1;
    if (indexTmp < DynamicHeightMapConfig::sizeY && segmentMapAnalysis[cell.x][indexTmp] != segmentID && segmentMapAnalysis[cell.x][indexTmp] != SEGMENT_UNKNOWN
        && segments[segmentMapAnalysis[cell.x][indexTmp]].type == SegmentType::Nonplanar)
      ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];
  }

  UInt indexMax = SEGMENT_UNKNOWN;
  UInt indexSecondMax = SEGMENT_UNKNOWN;
  UInt countMax = 0;
  UInt countSecondMax = 0;
  for (UInt i = 0; i < segmentMapping.size(); ++i)
    if (segmentMapping[i] > countMax)
    {
      countSecondMax = countMax;
      indexSecondMax = indexMax;
      countMax = segmentMapping[i];
      indexMax = i;
    }

  if (countMax > 3 * countSecondMax)
    return indexMax;
  else
    return SEGMENT_UNKNOWN;
}

void DynamicHeightMap::findSegmentsAdjacentToEdge()
{
  for (std::vector<Segment>::iterator segment = segments.begin(); segment != segments.end(); ++segment)
  {
    if (segment->type == SegmentType::Shadow || segment->type == SegmentType::None)
      continue;

    segment->adjacentToEdge = false;
    const Cell2DList &cellList = *segment->cellList;

    for (Cell2DList::const_iterator cell = cellList.begin(); cell != cellList.end(); ++cell)
    {
      if (cell->x == 0 || cell->y == 0 || cell->x == DynamicHeightMapConfig::sizeXMinus1 || cell->y == DynamicHeightMapConfig::sizeYMinus1)
        segment->adjacentToEdge = true;
      break;
    }
  }
}

void DynamicHeightMap::findSegmentTypeMap()
{
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      if (segmentMapAnalysis[xIndex][yIndex] != SEGMENT_GLOBAL_MAP)
        segmentTypeMapAnalysis[xIndex][yIndex] = segments[segmentMapAnalysis[xIndex][yIndex]].type;
      else
        segmentTypeMapAnalysis[xIndex][yIndex] = SegmentType::Static;
    }
}

void DynamicHeightMap::findHeightVariances()
{
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      if (segmentTypeMapAnalysis[xIndex][yIndex] == SegmentType::Planar || segmentTypeMapAnalysis[xIndex][yIndex] == SegmentType::Nonplanar)
      {
        Segment &segment = segments[segmentMapAnalysis[xIndex][yIndex]];
        segment.heightVariance += std::pow(segment.heightAverage - pointMapAnalysis[xIndex][yIndex].C2, 2);
      }
    }

  for (std::vector<Segment>::iterator segment = segments.begin(); segment != segments.end(); ++segment)
  {
    if (segment->type == SegmentType::Planar || segment->type == SegmentType::Nonplanar)
      segment->heightVariance /= (Real)(segment->cellList->size());
  }
}

Real DynamicHeightMap::getSegmentDistanceDeviation(const Segment &segment)
{
  const Cell2DList &cellList = *segment.cellList;

  const Real a = segment.normal.C0;
  const Real b = segment.normal.C1;
  const Real c = segment.normal.C2;
  const Real d = segment.distance;

  Real stdDev = 0.0;
  for (Cell2DList::const_iterator cell = cellList.begin(); cell != cellList.end(); ++cell)
  {
    const PV3D &point = pointMapAnalysis[cell->x][cell->y];
    stdDev += pow(a * point.C0 + b * point.C1 + c * point.C2 - d, 2);
  }
  return sqrt(stdDev / (segment.normal.norm() * cellList.size()));
}

void DynamicHeightMap::createNewObjects()
{
  objects.pop_back();
  objects.push_front(std::vector<Object>());
  for (UInt i = 0; i < objects.back().size(); ++i)
    objects.back()[i].objectPrevious = nullptr;

  for (std::vector<Segment>::iterator segment = segments.begin(); segment != segments.end(); ++segment)
  {
    //check if segment is large enough to be counted as object
    if (segment->type == SegmentType::Shadow || segment->type == SegmentType::None || segment->type == SegmentType::Static
        || fabs(segment->heightAverage) < 0.02 || segment->cellList->size() < DynamicHeightMapConfig::minObjectCellCount
        || segment->cellList->size() > DynamicHeightMapConfig::maxObjectCellCount || segment->adjacentToEdge)
      continue;

    objects.front().push_back(Object(*segment, timeStampCurrent, PV3D(robotPose.x, robotPose.y, robotPose.yaw)));
  }
}

void DynamicHeightMap::findObjectMappings()
{
  std::vector<Object> &objectsCurrent = objects[0];
  std::vector<Object> &objectsPrevious = objects[1];

  if (objectsCurrent.size() == 0)
    return;
  else if (objectsPrevious.size() == 0)
  {
    for (UInt i = 0; i < objectsCurrent.size(); ++i)
      objectsCurrent[i].updateTrajectory(DynamicHeightMapConfig::maxObjectTrackingTime);
    return;
  }

  std::vector<std::vector<Real> > mappingScores(objectsCurrent.size(), std::vector<Real>(objectsPrevious.size(), 0.0));
  std::vector<std::vector<UInt> > mappingPossibilities(objectsCurrent.size());

  Real maxDistance = 0.5;
  Real maxAreaDeviation = 0.5;
  Real maxHeightDeviation = 0.5;

  for (UInt i = 0; i < objectsCurrent.size(); ++i)
    for (UInt j = 0; j < objectsPrevious.size(); ++j)
    {
      const Real dist = MathStd::dist(objectsCurrent[i].position, objectsPrevious[j].position);
      const Real areaDev = fabs(((Real)objectsCurrent[i].sizeCells - (Real)objectsPrevious[j].sizeCells) / (Real)objectsCurrent[i].sizeCells);

      if (dist > maxDistance || areaDev > maxAreaDeviation || fabs(objectsCurrent[i].heightAverage - objectsPrevious[j].heightAverage) > maxHeightDeviation)
        continue;

      if (dist > 0.0)
        mappingScores[i][j] = 1.0 / dist;
      else
        mappingScores[i][j] = 1e5;
      mappingPossibilities[i].push_back(j);
    }

  std::vector<Int> permutation(objectsCurrent.size());
  std::vector<Int> permutationMax;
  Real scoreMax = 0;
  UInt index = 0;
  findMaximizingPermutation(index, permutation, permutationMax, scoreMax, mappingScores, mappingPossibilities);

  for (UInt i = 0; i < permutationMax.size(); ++i)
  {
    if (permutationMax[i] != -1)
      objectsCurrent[i].objectPrevious = &objectsPrevious[permutationMax[i]];
  }
}

void DynamicHeightMap::findMaximizingPermutation(Int index, std::vector<Int> &permutation, std::vector<Int> &permutationMax, Real &scoreMax,
                                                 const std::vector<std::vector<Real> > &mappingScores,
                                                 const std::vector<std::vector<UInt> > &mappingPossibilities)
{
  if (index == permutation.size() - 1)
  {
    permutation[index] = -1;
    Real score = 0;
    for (UInt i = 0; i < permutation.size(); ++i)
    {
      if (permutation[i] != -1)
        score += mappingScores[i][permutation[i]];
    }
    if (score >= scoreMax)
    {
      scoreMax = score;
      permutationMax = permutation;
    }

    for (UInt i = 0; i < mappingPossibilities[index].size(); ++i)
    {
      bool found = false;
      for (UInt j = 0; j < index; ++j)
        if (permutation[j] == mappingPossibilities[index][i])
        {
          found = true;
          break;
        }

      if (found)
        continue;

      permutation[index] = mappingPossibilities[index][i];

      Real score = 0;
      for (UInt j = 0; j < permutation.size(); ++j)
      {
        if (permutation[j] != -1)
          score += mappingScores[j][permutation[j]];
      }
      if (score >= scoreMax)
      {
        scoreMax = score;
        permutationMax = permutation;
      }
    }
  }
  else
  {
    permutation[index] = -1;
    findMaximizingPermutation(index + 1, permutation, permutationMax, scoreMax, mappingScores, mappingPossibilities);

    for (UInt i = 0; i < mappingPossibilities[index].size(); ++i)
    {
      bool found = false;
      for (UInt j = 0; j < index; ++j)
        if (permutation[j] == mappingPossibilities[index][i])
        {
          found = true;
          break;
        }

      if (found)
        continue;

      permutation[index] = mappingPossibilities[index][i];
      findMaximizingPermutation(index + 1, permutation, permutationMax, scoreMax, mappingScores, mappingPossibilities);
    }
  }
}

void DynamicHeightMap::updateObjectTrajectories()
{
  for (UInt i = 0; i < objects[0].size(); ++i)
    objects[0][i].updateTrajectory(0.3);
}

void DynamicHeightMap::updateObjectIDs()
{
  for (UInt i = 0; i < objects[0].size(); ++i)
  {
    Object &object = objects[0][i];

    if (object.objectPrevious == nullptr)
    {
      object.objectID = objectIDNext;
      ++objectIDNext;
    }
    else
      object.objectID = object.objectPrevious->objectID;
  }
}

void DynamicHeightMap::extractDynamicObjects()
{
  for (UInt i = 0; i < objects[0].size(); ++i)
  {
    Object &object = objects[0][i];
    Segment &segment = segments[object.segmentID];

    if (object.type != ObjectType::Dynamic)
      continue;

    segment.type = SegmentType::None;

    std::vector<UInt> segmentMapping(segments.size(), 0);

    const Cell2DList &cellList = *segment.cellList;
    Cell2DHeightList &cellListDynamic = object.cellListDynamic;
    const UInt cellListSize = cellList.size();
    cellListDynamic.resize(cellListSize);

    UInt indexTmp;
    for (UInt cellIndex = 0; cellIndex < cellListSize; ++cellIndex)
    {
      const Cell2D &cell = cellList[cellIndex];
      cellListDynamic[cellIndex].x = cell.x;
      cellListDynamic[cellIndex].y = cell.y;
      cellListDynamic[cellIndex].height = pointMapAnalysis[cell.x][cell.y].C2;

      indexTmp = cell.x - 1;
      if (indexTmp < DynamicHeightMapConfig::sizeX && segmentTypeMapAnalysis[indexTmp][cell.y] == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

      indexTmp = cell.x + 1;
      if (indexTmp < DynamicHeightMapConfig::sizeX && segmentTypeMapAnalysis[indexTmp][cell.y] == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[indexTmp][cell.y]];

      indexTmp = cell.y - 1;
      if (indexTmp < DynamicHeightMapConfig::sizeY && segmentTypeMapAnalysis[cell.x][indexTmp] == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];

      indexTmp = cell.y + 1;
      if (indexTmp < DynamicHeightMapConfig::sizeY && segmentTypeMapAnalysis[cell.x][indexTmp] == SegmentType::Planar)
        ++segmentMapping[segmentMapAnalysis[cell.x][indexTmp]];
    }

    UInt indexMax = SEGMENT_UNKNOWN;
    getIndexMax(segmentMapping, indexMax);

    if (indexMax == SEGMENT_UNKNOWN)
    {
      object.setObjectStatic();
      return;
    }

    const Segment &segmentMappingFinal = segments[indexMax];
    const SegmentType newType = segments[indexMax].type;
    Cell2DList &cellListMapping = *segmentMappingFinal.cellList;
    for (UInt cellIndex = 0; cellIndex < cellListSize; ++cellIndex)
    {
      const Cell2D &cell = cellList[cellIndex];

      segmentMapAnalysis[cell.x][cell.y] = indexMax;
      segmentTypeMapAnalysis[cell.x][cell.y] = newType;
      pointMapAnalysis[cell.x][cell.y].C2 = (segmentMappingFinal.distance - segmentMappingFinal.normal.C0 * pointMapAnalysis[cell.x][cell.y].C0
          - segmentMappingFinal.normal.C1 * pointMapAnalysis[cell.x][cell.y].C1) / segmentMappingFinal.normal.C2;
      cellListMapping.push_back(cell);
    }
  }
}

void DynamicHeightMap::createPredictionMaps()
{
#ifdef OPENMP_MAP
#pragma omp parallel for num_threads(6)
#endif
  for (UInt tIndex = 0; tIndex < DynamicHeightMapConfig::sizeT; ++tIndex)
  {
    SegmentTypeMap &segmentTypeMap = segmentTypeMapsPlanning[tIndex];
    SegmentMap &segmentMap = segmentMapsPlanning[tIndex];
    PointMap &pointMap = pointMapsPlanning[tIndex];
    EdgeMap &edgeMap = edgeMapsPlanning[tIndex];
    const Real timePredition = tIndex * DynamicHeightMapConfig::resolutionTime;

    std::vector<Cell2D> edgeQueue;
    edgeQueue.reserve(DynamicHeightMapConfig::sizeX * DynamicHeightMapConfig::sizeY);

    for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
      for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      {
        pointMap[xIndex][yIndex] = pointMapAnalysis[xIndex][yIndex];
        edgeMap[xIndex][yIndex] = edgeMapAnalysis[xIndex][yIndex];
        segmentTypeMap[xIndex][yIndex] = segmentTypeMapAnalysis[xIndex][yIndex];
        segmentMap[xIndex][yIndex] = segmentMapAnalysis[xIndex][yIndex];
        if (edgeMapAnalysis[xIndex][yIndex] == 0)
          edgeQueue.push_back(Cell2D(xIndex, yIndex));
      }

    for (UInt objectIndex = 0; objectIndex < objects[0].size(); ++objectIndex)
    {
      if (objects[0][objectIndex].type != ObjectType::Dynamic)
        continue;

      const Cell2DHeightList &cellList = objects[0][objectIndex].cellListDynamic;
      const PV2D position = objects[0][objectIndex].getRelativePosition(timePredition);

      const Int cellDisplacementX = DynamicHeightMapConfig::getIndexDisplacement(position.C0);
      const Int cellDisplacementY = DynamicHeightMapConfig::getIndexDisplacement(position.C1);

      for (Cell2DHeightList::const_iterator cell = cellList.begin(); cell != cellList.end(); ++cell)
      {
        const Cell2D cellTmp((Int)cell->x + cellDisplacementX, (Int)cell->y + cellDisplacementY);
        if (cellTmp.x >= DynamicHeightMapConfig::sizeX || cellTmp.y >= DynamicHeightMapConfig::sizeY)
          continue;

        pointMap[cellTmp.x][cellTmp.y].C2 = cell->height;
        segmentTypeMap[cellTmp.x][cellTmp.y] = SegmentType::Nonplanar;
        segmentMap[cellTmp.x][cellTmp.y] = 1000000;
        edgeMap[cellTmp.x][cellTmp.y] = 0;
        edgeQueue.push_back(cellTmp);
      }
    }

    inflateEdgeMap(edgeMap, edgeQueue);
  }
}

void DynamicHeightMap::findEdgeMap()
{
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      edgeMapAnalysis[xIndex][yIndex] = -1;

  for (UInt xIndex = 1; xIndex < DynamicHeightMapConfig::sizeXMinus1; ++xIndex)
    for (UInt yIndex = 1; yIndex < DynamicHeightMapConfig::sizeYMinus1; ++yIndex)
    {
      if (segmentTypeMapAnalysis[xIndex][yIndex] == SegmentType::Shadow)
        continue;

      if (segmentTypeMapAnalysis[xIndex][yIndex] == SegmentType::Nonplanar
          || (segmentTypeMapAnalysis[xIndex - 1][yIndex] != SegmentType::Shadow && segmentMapAnalysis[xIndex - 1][yIndex] != segmentMapAnalysis[xIndex][yIndex])
          || (segmentTypeMapAnalysis[xIndex + 1][yIndex] != SegmentType::Shadow && segmentMapAnalysis[xIndex + 1][yIndex] != segmentMapAnalysis[xIndex][yIndex])
          || (segmentTypeMapAnalysis[xIndex][yIndex - 1] != SegmentType::Shadow && segmentMapAnalysis[xIndex][yIndex - 1] != segmentMapAnalysis[xIndex][yIndex])
          || (segmentTypeMapAnalysis[xIndex][yIndex + 1] != SegmentType::Shadow && segmentMapAnalysis[xIndex][yIndex + 1] != segmentMapAnalysis[xIndex][yIndex]))
        edgeMapAnalysis[xIndex][yIndex] = 0;
    }
}

void DynamicHeightMap::inflateEdgeMap(EdgeMap &edgeMap, std::vector<Cell2D> &edgeQueue)
{
  for (UInt cellIndex = 0; cellIndex < edgeQueue.size(); ++cellIndex)
  {
    const Cell2D cellCenter = edgeQueue[cellIndex];

    Cell2D cell(cellCenter.x - 1, cellCenter.y - 1);

    if (cell.x < DynamicHeightMapConfig::sizeX && cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceDiagonalMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceStraightMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x + 1;
    if (cell.x < DynamicHeightMapConfig::sizeX && cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceDiagonalMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < DynamicHeightMapConfig::sizeX)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceStraightMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y + 1;
    if (cell.x < DynamicHeightMapConfig::sizeX && cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceDiagonalMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceStraightMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x - 1;
    if (cell.x < DynamicHeightMapConfig::sizeX && cell.y < DynamicHeightMapConfig::sizeY)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceDiagonalMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < DynamicHeightMapConfig::sizeX)
    {
      UInt newDistance = edgeMap[cellCenter.x][cellCenter.y] + DynamicHeightMapConfig::resolutionSpaceStraightMM;
      if (newDistance < DynamicHeightMapConfig::safetyDistanceMM && (edgeMap[cell.x][cell.y] < 0 || edgeMap[cell.x][cell.y] > newDistance))
      {
        edgeMap[cell.x][cell.y] = newDistance;
        edgeQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }
  }

//add border
  for (UInt i = 0; i < DynamicHeightMapConfig::sizeX; ++i)
  {
    edgeMap[i][0] = 0;
    edgeMap[i][1] = 0;
    edgeMap[i][DynamicHeightMapConfig::sizeYMinus1] = 0;
    edgeMap[i][DynamicHeightMapConfig::sizeYMinus2] = 0;
  }
  for (UInt i = 0; i < DynamicHeightMapConfig::sizeY; ++i)
  {
    edgeMap[0][i] = 0;
    edgeMap[1][i] = 0;
    edgeMap[DynamicHeightMapConfig::sizeXMinus1][i] = 0;
    edgeMap[DynamicHeightMapConfig::sizeXMinus2][i] = 0;
  }
}

//////////////////// CAMERA //////////////////////////////////////

void DynamicHeightMap::getFrameFromDepthImageBuffer(const Matrix34 &transform)
{
  HeightMap &heights = *heightsFiltering.current;
  for (UInt height = 0; height < CameraConfig::height; ++height)
  {
    for (UInt width = 0; width < CameraConfig::width; ++width)
    {
      const PV3D &point = depthImagePointBuffer[height][width];

      if (point.C2 == 0.0)
        continue;

      const UInt xIndex = DynamicHeightMapConfig::getIndexX(transform.C00 * point.C0 + transform.C01 * point.C1 + transform.C02 * point.C2 + transform.C03);
      const UInt yIndex = DynamicHeightMapConfig::getIndexY(transform.C10 * point.C0 + transform.C11 * point.C1 + transform.C12 * point.C2 + transform.C13);

      if (xIndex >= DynamicHeightMapConfig::sizeX || yIndex >= DynamicHeightMapConfig::sizeY)
        continue;

      const Real heightTmp = transform.C20 * point.C0 + transform.C21 * point.C1 + transform.C22 * point.C2 + transform.C23;

      if (heights[xIndex][yIndex] == HEIGHT_UNKNOWN)
      {
        heights[xIndex][yIndex] = heightTmp;
        pointsInitial[xIndex][yIndex] = heightTmp;
      }
      else if (heights[xIndex][yIndex] < heightTmp)
      {
        heights[xIndex][yIndex] = heightTmp;
        pointsInitial[xIndex][yIndex] = heightTmp;
      }
    }
  }
}

void DynamicHeightMap::subscriberDepthImageHandler(const sensor_msgs::Image &msg)
{
  const float* f = reinterpret_cast<const float*>(&msg.data[0]);

  UInt height = msg.height;
  UInt width = msg.width;

  for (UInt y = 0; y < height; ++y)
    for (UInt x = 0; x < width; ++x)
    {
      depthImagePointBuffer[y][x].C0 = *f * unitImage[y][x].C0;
      depthImagePointBuffer[y][x].C1 = *f * unitImage[y][x].C1;
      depthImagePointBuffer[y][x].C2 = *f * unitImage[y][x].C2;
      ++f;
    }
}

//////////////////// INLINES /////////////////////////////////////

inline void DynamicHeightMap::resetMaps()
{
  segments.clear();
  HeightMap &raw = *heightsFiltering.current;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      segmentMapAnalysis[xIndex][yIndex] = SEGMENT_UNKNOWN;
      pointsInitial[xIndex][yIndex] = HEIGHT_UNKNOWN;
      raw[xIndex][yIndex] = HEIGHT_UNKNOWN;
    }
}

inline Real DynamicHeightMap::getElapsedTime()
{
  return (timeStampCurrent - timeStart).toSec();
}

inline void DynamicHeightMap::copyFilteredToPointMap()
{
  const HeightMap &filtered = *heightsFiltering.current;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      pointMapAnalysis[xIndex][yIndex].C2 = filtered[xIndex][yIndex];
}

inline void DynamicHeightMap::copyInitialToRaw()
{
  HeightMap &raw = *heightsFiltering.current;
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      raw[xIndex][yIndex] = pointsInitial[xIndex][yIndex];
}

inline Real DynamicHeightMap::getMedian(std::vector<Real> &vec) const
{
  Real tmp;
//group 1
  if (vec[0] > vec[1])
  {
    tmp = vec[0];
    vec[0] = vec[1];
    vec[1] = tmp;
  }
  if (vec[3] > vec[4])
  {
    tmp = vec[3];
    vec[3] = vec[4];
    vec[4] = tmp;
  }
  if (vec[6] > vec[7])
  {
    tmp = vec[6];
    vec[6] = vec[7];
    vec[7] = tmp;
  }
//group 2
  if (vec[1] > vec[2])
  {
    tmp = vec[1];
    vec[1] = vec[2];
    vec[2] = tmp;
  }
  if (vec[4] > vec[5])
  {
    tmp = vec[4];
    vec[4] = vec[5];
    vec[5] = tmp;
  }
  if (vec[7] > vec[8])
  {
    tmp = vec[7];
    vec[7] = vec[8];
    vec[8] = tmp;
  }
//group 3
  if (vec[0] > vec[1])
  {
    tmp = vec[0];
    vec[0] = vec[1];
    vec[1] = tmp;
  }
  if (vec[3] > vec[4])
  {
    tmp = vec[3];
    vec[3] = vec[4];
    vec[4] = tmp;
  }
  if (vec[6] > vec[7])
  {
    tmp = vec[6];
    vec[6] = vec[7];
    vec[7] = tmp;
  }
//group 4
  if (vec[1] > vec[4])
  {
    tmp = vec[1];
    vec[1] = vec[4];
    vec[4] = tmp;
  }
//group 5
  if (vec[0] > vec[3])
  {
    tmp = vec[0];
    vec[0] = vec[3];
    vec[3] = tmp;
  }
  if (vec[5] > vec[8])
  {
    tmp = vec[5];
    vec[5] = vec[8];
    vec[8] = tmp;
  }
//group 6
  if (vec[4] > vec[7])
  {
    tmp = vec[4];
    vec[4] = vec[7];
    vec[7] = tmp;
  }
//group 7
  if (vec[3] > vec[6])
  {
    tmp = vec[3];
    vec[3] = vec[6];
    vec[6] = tmp;
  }
  if (vec[2] > vec[5])
  {
    tmp = vec[2];
    vec[2] = vec[5];
    vec[5] = tmp;
  }
//group 8
  if (vec[1] > vec[4])
  {
    tmp = vec[1];
    vec[1] = vec[4];
    vec[4] = tmp;
  }
//group 9
  if (vec[6] > vec[4])
  {
    tmp = vec[6];
    vec[6] = vec[4];
    vec[4] = tmp;
  }
//group 10
  if (vec[4] > vec[2])
  {
    tmp = vec[4];
    vec[4] = vec[2];
    vec[2] = tmp;
  }
//group 9
  if (vec[6] > vec[4])
  {
    tmp = vec[6];
    vec[6] = vec[4];
    vec[4] = tmp;
  }

  return vec[4];
}

inline void DynamicHeightMap::getIndexMax(const std::vector<UInt> &vec, UInt &index) const
{
  UInt countMax = 0;
  for (UInt i = 0; i < vec.size(); ++i)
    if (vec[i] > countMax)
    {
      countMax = vec[i];
      index = i;
    }
}

//////////////////// PUBLISHING //////////////////////////////////

void DynamicHeightMap::publishObjectPredictions()
{
  if (publisherObjectPredictions.getNumSubscribers() == 0 || objects[0].empty())
    return;

  visualization_msgs::Marker msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  //msg.pose.position.x = robotPose.C0;
  //msg.pose.position.y = robotPose.C1;
  tf::Quaternion quat;
  //quat.setRPY(0.0, 0.0, robotPose.C2);
  msg.pose.orientation.w = quat.w();
  msg.pose.orientation.x = quat.x();
  msg.pose.orientation.y = quat.y();
  msg.pose.orientation.z = quat.z();
  msg.type = visualization_msgs::Marker::POINTS;
  msg.color.a = 1.0;
  msg.color.r = msg.color.g = 0.0;
  msg.color.b = 0.7;
  msg.id = 5000;
  msg.points.reserve(objects[0].size() * 17);
  msg.scale.x = 0.01;

  for (UInt i = 0; i < objects[0].size(); ++i)
  {
    if (objects[0][i].type != ObjectType::Dynamic)
      continue;

    Real timeCurrent = 0.0;
    while (timeCurrent < 3.0)
    {
      const PV2D pos = objects[0][i].getAbsolutePosition(timeCurrent);

      msg.points.push_back(geometry_msgs::Point());
      msg.points.back().x = pos.C0;
      msg.points.back().y = pos.C1;
      msg.points.back().z = 0.1;
      timeCurrent += 0.2;
    }
  }

  if (msg.points.empty())
    return;

  publisherObjectPredictions.publish(msg);
}
