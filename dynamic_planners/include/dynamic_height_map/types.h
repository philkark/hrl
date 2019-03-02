#ifndef DYNAMIC_HEIGHT_MAP_TYPES_H_
#define DYNAMIC_HEIGHT_MAP_TYPES_H_

#include <vector>
#include <math_std/common.h>
#include <math_std/pv.h>
#include <ros/ros.h>
#include <utility>

#define HEIGHT_UNKNOWN 1000000.0
#define SEGMENT_UNKNOWN 10000
#define SEGMENT_GLOBAL_MAP 10002

enum class SegmentType
{
  None, Planar, Nonplanar, Shadow, Static
};

enum class ObjectType
{
  None, Static, Dynamic
};

enum class SimulationObjectType
{
  Cylinder, Sphere, Cuboid
};

struct Cell2D;
struct Cell2DHeight;
struct Cell2DSigned;
struct Cell2DSignedHeight;

struct Object;

class SimulationObject;

typedef std::vector<Cell2D> Cell2DList;
typedef std::vector<Cell2DSigned> Cell2DSignedList;
typedef std::vector<Cell2DHeight> Cell2DHeightList;
typedef std::vector<Cell2DSignedHeight> Cell2DSignedHeightList;

typedef Cell2DList::iterator Cell2DIt;
typedef Cell2DSignedList::iterator Cell2DSignedIt;
typedef Cell2DHeightList::iterator Cell2DHeightIt;
typedef Cell2DSignedHeightList::iterator Cell2DSignedHeightIt;

typedef Cell2DList::const_iterator Cell2DCIt;
typedef Cell2DSignedList::const_iterator Cell2DSignedCIt;
typedef Cell2DHeightList::const_iterator Cell2DHeightCIt;
typedef Cell2DSignedHeightList::const_iterator Cell2DSignedHeightCIt;

typedef std::vector<std::vector<Real> > HeightMap;
typedef std::vector<std::vector<PV3D> > PointMap;
typedef std::vector<std::vector<UInt> > SegmentMap;
typedef std::vector<std::vector<Int> > EdgeMap;
typedef std::vector<std::vector<PV3D> > NormalMap;
typedef std::vector<std::vector<SegmentType> > SegmentTypeMap;
typedef std::vector<std::vector<bool> > OccupancyMap;
typedef std::vector<std::pair<ros::Time, PV2D> > Trajectory;

typedef std::vector<Object*>::const_iterator ObjectPtrIterator;

typedef std::vector<SimulationObject>::const_iterator SimulationObjectIterator;

struct ParabolicTrajectory
{
  PV2D positionZero;
  PV2D direction;

  Real timeStop;
  Real coeffSquare;
  Real coeffLinear;
};

struct Pose6D
{
  Real x;
  Real y;
  Real z;
  Real roll;
  Real pitch;
  Real yaw;
};

#endif //DYNAMIC_HEIGHT_MAP_TYPES_H_
