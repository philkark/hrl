#include<math_std/geometry.h>

//############# Ellipse #############

Ellipse::Ellipse() :
    radiusX(1), radiusY(1)
{
}
Ellipse::Ellipse(Real radiusX, Real radiusY) :
    radiusX(abs(radiusX)), radiusY(abs(radiusY))
{
}

//############# Ellipsoid #############

Ellipsoid::Ellipsoid() :
    radiusX(1), radiusY(1), radiusZ(1)
{
}
Ellipsoid::Ellipsoid(Real radiusX, Real radiusY, Real radiusZ) :
    radiusX(abs(radiusX)), radiusY(abs(radiusY)), radiusZ(abs(radiusZ))
{
}

//############# Box2D #############

Box2D::Box2D() :
    minimum(), maximum()
{
}
Box2D::Box2D(const PV2D &minInit, const PV2D &maxInit) :
    minimum(minInit), maximum(maxInit)
{
}
std::ostream& operator<<(std::ostream &out, Box2D &box)
{
  out << "Min: " << std::endl << "  " << box.minimum;
  out << "Max: " << std::endl << "  " << box.maximum;
  return out;
}

void Box2D::findBox(const std::vector<PV2D> &points)
{
  minimum = maximum = points[0];
  for (Int i = 1; i < points.size(); ++i)
  {
    if (points[i].C0 < minimum.C0)
      minimum.C0 = points[i].C0;
    if (points[i].C1 < minimum.C1)
      minimum.C1 = points[i].C1;
    if (points[i].C0 > maximum.C0)
      maximum.C0 = points[i].C0;
    if (points[i].C1 > maximum.C1)
      maximum.C1 = points[i].C1;
  }
}

//############# Box3D #############

Box3D::Box3D() :
    minimum(), maximum()
{
}
Box3D::Box3D(const PV3D &minInit, const PV3D &maxInit) :
    minimum(minInit), maximum(maxInit)
{
}
std::ostream& operator<<(std::ostream &out, Box3D &box)
{
  out << "Min: " << std::endl << "  " << box.minimum;
  out << "Max: " << std::endl << "  " << box.maximum;
  return out;
}

void Box3D::findBox(const std::vector<PV3D> &points)
{
  minimum = maximum = points[0];
  for (Int i = 1; i < points.size(); ++i)
  {
    if (points[i].C0 < minimum.C0)
      minimum.C0 = points[i].C0;
    if (points[i].C1 < minimum.C1)
      minimum.C1 = points[i].C1;
    if (points[i].C2 < minimum.C2)
      minimum.C2 = points[i].C2;
    if (points[i].C0 > maximum.C0)
      maximum.C0 = points[i].C0;
    if (points[i].C1 > maximum.C1)
      maximum.C1 = points[i].C1;
    if (points[i].C2 > maximum.C2)
      maximum.C2 = points[i].C2;
  }
}
