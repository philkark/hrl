#ifndef DYNAMIC_HEIGHT_MAP_OBJECTS_H_
#define DYNAMIC_HEIGHT_MAP_OBJECTS_H_

//personal
#include <math_std/math_std.h>

//local
#include "dynamic_height_map/cells.h"
#include "dynamic_height_map/dynamic_height_map_config.h"
#include "dynamic_height_map/segment.h"
#include "dynamic_height_map/types.h"

class Object
{
public:
  ObjectType type;
  SegmentType segmentType;
  UInt objectID;
  UInt segmentID;
  Object* objectPrevious;

  ros::Time timeStamp;
  PV2D position;
  PV2D velocity;
  PV3D robotPose;
  Real cosA;
  Real sinA;

  Real sizeCells;
  Real heightAverage;
  Real heightVariance;

  Cell2DHeightList cellListDynamic;

  Object(const Segment &segment, const ros::Time &timeStamp, const PV3D &robotPose);

  PV2D getRelativePosition(const Real time) const;

  PV2D getAbsolutePosition(const Real time) const;

  // ************* TRAJECTORY FINDING *************

  void updateTrajectory(const Real maxAveragingTime);

  void setObjectStatic();

private:

  void getTransformedPositionsAndTimes(std::vector<PV2D> &positions, std::vector<Real> &times, const UInt maxChainLength);

  void findMovingDirection(const std::vector<PV2D> &positions);

  void setSpeed(const std::vector<PV2D> &positions, const std::vector<Real> &times);
};

#endif // DYNAMIC_HEIGHT_MAP_OBJECTS_H_

