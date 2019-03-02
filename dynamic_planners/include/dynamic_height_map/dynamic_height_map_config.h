#ifndef DYNAMIC_HEIGHT_MAP_CONFIGS_H_
#define DYNAMIC_HEIGHT_MAP_CONFIGS_H_

//personal
#include <tools_std/ros_param_loader.h>
#include <math_std/math_std.h>

//local
#include "dynamic_height_map/cells.h"
#include "dynamic_height_map/types.h"

struct DynamicHeightMapConfig
{
  static Real minX;
  static Real maxX;
  static Real minY;
  static Real maxY;
  static Real resolutionSpace;

  static Real minT;
  static Real maxT;
  static Real resolutionTime;

  static Real safetyDistance;
  static Real safetyBoundary;

  static Real segmentationCosMinDeviation;
  static Real segmentationCosMinVertical;
  static Real segmentationMaxSegmentDistance;

  static UInt mapHistoryCount;
  static Real maxObjectTrackingTime;
  static UInt maxSegmentsPerMap;
  static UInt minObjectCellCount;
  static UInt maxObjectCellCount;

  //these are set automatically depending on the bounds and sizes
  static Real resolutionSpaceRecip;
  static Real resolutionTimeRecip;
  static UInt sizeT;
  static UInt sizeX;
  static UInt sizeY;
  static UInt sizeXMinus1;
  static UInt sizeYMinus1;
  static UInt sizeXMinus2;
  static UInt sizeYMinus2;
  static Real normalFactor;

  static Int resolutionSpaceStraightMM;
  static Int resolutionSpaceDiagonalMM;
  static Int safetyDistanceMM;

  static Real boundaryMinX;
  static Real boundaryMaxX;
  static Real boundaryMinY;
  static Real boundaryMaxY;

  static void initialize(const ros::NodeHandle &nh);

  static UInt getIndexX(const Real position);

  static UInt getIndexY(const Real position);

  static UInt getIndexT(const Real time);

  static Int getIndexDisplacement(const Real position);

  static PV2D getPoint(const Cell2D &cell);

  static Cell2D getCell(const PV2D &pv);
};

struct SimulationConfig
{
  static Real simulationTimeInterval;
  static Real simulationNoiseMean;
  static Real simulationNoiseStdDev;
  static std::string simulationFilePath;

  static void initialize(const ros::NodeHandle &nh);

  static void updateFilePath(const std::string &simulationName);
};

struct CameraConfig
{
  static UInt width;
  static UInt height;
  static UInt fps;
  static Real openingAngleWidth;
  static Real openingAngleHeight;

  static void initialize(const ros::NodeHandle &nh);
};

#endif //DYNAMIC_HEIGHT_MAP_CONFIGS_H_
