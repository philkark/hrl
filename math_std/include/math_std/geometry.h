#ifndef MATH_STD_GEOMETRY_H
#define MATH_STD_GEOMETRY_H

#include<math_std/common.h>
#include<math_std/pv.h>

struct Ellipse
{
  Real radiusX, radiusY;

  Ellipse();
  Ellipse(Real radiusX, Real radiusY);
};

struct Ellipsoid
{
  Real radiusX, radiusY, radiusZ;

  Ellipsoid();
  Ellipsoid(Real radiusX, Real radiusY, Real radiusZ);
};

struct Box2D
{
  PV2D minimum, maximum;

  Box2D();
  Box2D(const PV2D &minInit, const PV2D &maxInit);
  friend std::ostream& operator<<(std::ostream &out, Box2D &box);

  void findBox(const std::vector<PV2D> &points);
};

struct Box3D
{
  PV3D minimum, maximum;

  Box3D();
  Box3D(const PV3D &minInit, const PV3D &maxInit);
  friend std::ostream& operator<<(std::ostream &out, Box3D &box);

  void findBox(const std::vector<PV3D> &points);
};

#endif //MATH_PATH_GEOMETRY_H
