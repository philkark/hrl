#include <dynamic_height_map/cells.h>

// *********************************************************************

// ************************** Cell2D ***********************************

// *********************************************************************

Cell2D::Cell2D()
{
}

Cell2D::Cell2D(const UInt &x, const UInt &y) :
    x(x), y(y)
{
}

bool Cell2D::operator==(const Cell2D &cell) const
{
  return x == cell.x && y == cell.y;
}

// *********************************************************************

// ************************** Cell2DHeight *****************************

// *********************************************************************

Cell2DHeight::Cell2DHeight()
{
}

Cell2DHeight::Cell2DHeight(const UInt &x, const UInt &y, const Real &height) :
    Cell2D(x, y), height(height)
{

}

// *********************************************************************

// ************************** Cell2DSigned *****************************

// *********************************************************************

Cell2DSigned::Cell2DSigned()
{
}

Cell2DSigned::Cell2DSigned(const Int &x, const Int &y) :
    x(x), y(y)
{
}

// *********************************************************************

// ************************** Cell2DSignedHeight ***********************

// *********************************************************************

Cell2DSignedHeight::Cell2DSignedHeight()
{
}

Cell2DSignedHeight::Cell2DSignedHeight(const Int &x, const Int &y, const Real &height) :
    Cell2DSigned(x, y), height(height)
{
}
