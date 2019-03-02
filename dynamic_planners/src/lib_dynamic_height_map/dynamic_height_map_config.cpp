#include <dynamic_height_map/dynamic_height_map_config.h>

// *********************************************************************

// ************************** DynamicHeightMapConfig *******************

// *********************************************************************

Real DynamicHeightMapConfig::minX = 0.0;
Real DynamicHeightMapConfig::maxX = 0.0;
Real DynamicHeightMapConfig::minY = 0.0;
Real DynamicHeightMapConfig::maxY = 0.0;
Real DynamicHeightMapConfig::resolutionSpace = 0.0;
Real DynamicHeightMapConfig::minT = 0.0;
Real DynamicHeightMapConfig::maxT = 0.0;
Real DynamicHeightMapConfig::resolutionTime = 0.0;
Real DynamicHeightMapConfig::safetyDistance = 0.0;
Real DynamicHeightMapConfig::safetyBoundary = 0.0;
Real DynamicHeightMapConfig::segmentationCosMinDeviation = 0.0;
Real DynamicHeightMapConfig::segmentationCosMinVertical = 0.0;
Real DynamicHeightMapConfig::segmentationMaxSegmentDistance = 0.0;
UInt DynamicHeightMapConfig::mapHistoryCount = 0;
Real DynamicHeightMapConfig::maxObjectTrackingTime = 0.0;
UInt DynamicHeightMapConfig::maxSegmentsPerMap = 0;
UInt DynamicHeightMapConfig::minObjectCellCount = 0;
UInt DynamicHeightMapConfig::maxObjectCellCount = 0;

Real DynamicHeightMapConfig::resolutionSpaceRecip = 0.0;
Real DynamicHeightMapConfig::resolutionTimeRecip = 0.0;
UInt DynamicHeightMapConfig::sizeT = 0;
UInt DynamicHeightMapConfig::sizeX = 0;
UInt DynamicHeightMapConfig::sizeY = 0;
UInt DynamicHeightMapConfig::sizeXMinus1 = 0;
UInt DynamicHeightMapConfig::sizeYMinus1 = 0;
UInt DynamicHeightMapConfig::sizeXMinus2 = 0;
UInt DynamicHeightMapConfig::sizeYMinus2 = 0;
Real DynamicHeightMapConfig::normalFactor = 0.0;
Int DynamicHeightMapConfig::resolutionSpaceStraightMM = 0;
Int DynamicHeightMapConfig::resolutionSpaceDiagonalMM = 0;
Int DynamicHeightMapConfig::safetyDistanceMM = 0;
Real DynamicHeightMapConfig::boundaryMinX = 0.0;
Real DynamicHeightMapConfig::boundaryMaxX = 0.0;
Real DynamicHeightMapConfig::boundaryMinY = 0.0;
Real DynamicHeightMapConfig::boundaryMaxY = 0.0;

void DynamicHeightMapConfig::initialize(const ros::NodeHandle &nh)
{
  Real tmpReal;
  UInt tmpUInt;
  std::string tmpString;

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "min_x", minX, -0.2);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_x", maxX, 1.8);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "min_y", minY, -1.0);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_y", maxY, 1.0);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "resolution_space", resolutionSpace, 1.0);

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "min_t", minT, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_t", maxT, 10.0);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "resolution_time", resolutionTime, 20);

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "safety_distance", safetyDistance, 0.25);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "safety_boundary", safetyBoundary, 0.2);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "segmentation_max_segment_distance", segmentationMaxSegmentDistance, 0.01);

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "map_history_count", mapHistoryCount, 3);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_segments_per_map", maxSegmentsPerMap, 100);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_object_tracking_time", maxObjectTrackingTime, 0.3);

  resolutionSpaceRecip = 1.0 / resolutionSpace;
  resolutionTimeRecip = 1.0 / resolutionTime;
  sizeX = (UInt)ceil((maxX - minX) * resolutionSpaceRecip);
  sizeY = (UInt)ceil((maxY - minY) * resolutionSpaceRecip);
  sizeT = (UInt)ceil((maxT - minT) * resolutionTimeRecip);
  if (sizeX < 2)
    sizeX = 2;
  if (sizeY < 2)
    sizeY = 2;
  if (sizeT < 1)
    sizeT = 1;
  sizeXMinus1 = sizeX - 1;
  sizeYMinus1 = sizeY - 1;
  sizeXMinus2 = sizeX - 2;
  sizeYMinus2 = sizeY - 2;

  resolutionSpaceStraightMM = (Int)(1000 * resolutionSpace);
  resolutionSpaceDiagonalMM = (Int)(resolutionSpaceStraightMM * M_SQRT2);
  safetyDistanceMM = (Int)(1000 * safetyDistance);
  normalFactor = 6 * resolutionSpace;

  boundaryMinX = minX + safetyBoundary;
  boundaryMaxX = maxX - safetyBoundary;
  boundaryMinY = minY + safetyBoundary;
  boundaryMaxY = maxY - safetyBoundary;

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "min_object_size", tmpReal, 0.03);
  minObjectCellCount = (UInt)(tmpReal * tmpReal / (resolutionSpace * resolutionSpace));

  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "max_object_size", tmpReal, 0.3);
  maxObjectCellCount = (UInt)(tmpReal * tmpReal / (resolutionSpace * resolutionSpace));

  //further parameters used for map analysis
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "segmentation_max_angle_deviation", tmpReal, 5);
  segmentationCosMinDeviation = cos(tmpReal * TORAD);
  RosParamLoader::getRosParam(nh, "DynamicHeightMapConfig::initialize", "segmentation_max_angle_vertical", tmpReal, 5);
  segmentationCosMinVertical = cos(tmpReal * TORAD);
}

UInt DynamicHeightMapConfig::getIndexX(const Real position)
{
  return (UInt)((position - minX) * resolutionSpaceRecip);
}

UInt DynamicHeightMapConfig::getIndexY(const Real position)
{
  return (UInt)((position - minY) * resolutionSpaceRecip);
}

UInt DynamicHeightMapConfig::getIndexT(const Real time)
{
  return (UInt)((time - minT) * resolutionTimeRecip);
}

Int DynamicHeightMapConfig::getIndexDisplacement(const Real position)
{
  return (Int)(position * resolutionSpaceRecip);
}

PV2D DynamicHeightMapConfig::getPoint(const Cell2D &cell)
{
  return PV2D(minX + (cell.x + 0.5) * resolutionSpace, minY + (cell.y + 0.5) * resolutionSpace);
}

Cell2D DynamicHeightMapConfig::getCell(const PV2D &pv)
{
  return Cell2D((UInt)((pv.C0 - minX) * resolutionSpaceRecip), (UInt)((pv.C1 - minY) * resolutionSpaceRecip));
}

// *********************************************************************

// ************************** SimulationConfig *************************

// *********************************************************************

Real SimulationConfig::simulationTimeInterval = 0.0;
Real SimulationConfig::simulationNoiseMean = 0.0;
Real SimulationConfig::simulationNoiseStdDev = 0.0;
std::string SimulationConfig::simulationFilePath = "";

void SimulationConfig::initialize(const ros::NodeHandle &nh)
{
  RosParamLoader::getRosParam(nh, "SimulationConfig::initialize()", "simulation_time_interval", simulationTimeInterval, 0.03);
  RosParamLoader::getRosParam(nh, "SimulationConfig::initialize()", "simulation_noise_mean", simulationNoiseMean, 0.0);
  RosParamLoader::getRosParam(nh, "SimulationConfig::initialize()", "simulation_noise_stddev", simulationNoiseStdDev, 0.0);

  simulationFilePath = ros::package::getPath("dynamic_planners") + "/simulation_scenarios/empty";
}

void SimulationConfig::updateFilePath(const std::string &simulationName)
{
  simulationFilePath = ros::package::getPath("dynamic_planners") + "/simulation_scenarios/" + simulationName;
}

// *********************************************************************

// ************************** CameraConfig *****************************

// *********************************************************************

UInt CameraConfig::width = 0;
UInt CameraConfig::height = 0;
UInt CameraConfig::fps = 0;
Real CameraConfig::openingAngleWidth = 0.0;
Real CameraConfig::openingAngleHeight = 0.0;

void CameraConfig::initialize(const ros::NodeHandle &nh)
{
  RosParamLoader::getRosParam(nh, "CameraConfig::CameraConfig()", "camera_width", width, 640);
  RosParamLoader::getRosParam(nh, "CameraConfig::CameraConfig()", "camera_height", height, 480);
  RosParamLoader::getRosParam(nh, "CameraConfig::CameraConfig()", "camera_fps", fps, 30);
  RosParamLoader::getRosParam(nh, "CameraConfig::CameraConfig()", "camera_opening_angle_width", openingAngleWidth, 60);
  RosParamLoader::getRosParam(nh, "CameraConfig::CameraConfig()", "camera_opening_angle_height", openingAngleHeight, 46);
}
