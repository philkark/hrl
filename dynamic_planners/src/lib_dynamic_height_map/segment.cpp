#include <dynamic_height_map/segment.h>


// *********************************************************************

// ************************** Segment **********************************

// *********************************************************************

Segment::Segment()
{
  reset();
}

void Segment::reset()
{
  cellList = nullptr;
  heightAverage = 0;
  heightVariance = 0;
  position.C0 = position.C1 = 0.0;
}
