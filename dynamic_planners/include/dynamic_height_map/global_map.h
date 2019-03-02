#ifndef DYNAMIC_HEIGHT_MAP_GLOBAL_MAP_H_
#define DYNAMIC_HEIGHT_MAP_GLOBAL_MAP_H_

#include "dynamic_height_map/types.h"
#include "dynamic_height_map/cells.h"
#include "dynamic_planners/get_height_map.h"

struct GlobalMap
{
  PointMap pointMap;
  OccupancyMap occupancyMap;
  Real minX;
  Real maxX;
  Real minY;
  Real maxY;
  UInt sizeX;
  UInt sizeY;
  Real resolution;
  Real resolutionRecip;

  GlobalMap();

  void setMap(const dynamic_planners::get_height_map::Request &req, const dynamic_planners::get_height_map::Response &res, const Real safetyDistance);

  void generateOccupancyMap(std::vector<std::vector<Int> > &distanceMap, std::vector<Cell2D> &occupancyQueue, const Int safetyDistanceMM);

  Real getHeight(const Real x, const Real y) const;

  Cell2D getCell(const PV2D &point) const;

  PV2D getPoint(const Cell2D &cell) const;
};

#endif // DYNAMIC_HEIGHT_MAP_GLOBAL_MAP_H_
