#ifndef DYNAMIC_HEIGHT_MAP_SEGMENT_HPP_
#define DYNAMIC_HEIGHT_MAP_SEGMENT_HPP_

#include <dynamic_height_map/cells.h>
#include <dynamic_height_map/dynamic_height_map_config.h>
#include <dynamic_height_map/types.h>
#include <math_std/common.h>
#include <math_std/pv.h>

struct Segment
{
  SegmentType type;
  UInt ID;
  Cell2DList* cellList;

  PV2D position;

  Real heightAverage;
  Real heightVariance;

  PV3D normal;
  Real distance;

  bool adjacentToEdge;

  Segment();

  void reset();
};

#endif // DYNAMIC_HEIGHT_MAP_SEGMENT_HPP_
