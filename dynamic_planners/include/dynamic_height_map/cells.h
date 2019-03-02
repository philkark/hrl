#ifndef DYNAMIC_HEIGHT_MAP_CELLS_H_
#define DYNAMIC_HEIGHT_MAP_CELLS_H_

#include <vector>
#include <math_std/common.h>

struct Cell2D
{
  UInt x;
  UInt y;

  Cell2D();

  Cell2D(const UInt &x, const UInt &y);

  bool operator==(const Cell2D &cell) const;
};

struct Cell2DHeight : Cell2D
{
  Real height;

  Cell2DHeight();

  Cell2DHeight(const UInt &x, const UInt &y, const Real &height);
};

struct Cell2DSigned
{
  Int x;
  Int y;

  Cell2DSigned();

  Cell2DSigned(const Int &x, const Int &y);
};

struct Cell2DSignedHeight : Cell2DSigned
{
  Real height;

  Cell2DSignedHeight();

  Cell2DSignedHeight(const Int &x, const Int &y, const Real &height);
};

#endif //DYNAMIC_HEIGHT_MAP_CELLS_H_
