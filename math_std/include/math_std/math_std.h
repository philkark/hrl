#ifndef MATH_STD_H
#define MATH_STD_H

#include <math_std/common.h>
#include <math_std/pv.h>
#include <math_std/matrix.h>
#include <math_std/line.h>
#include <math_std/geometry.h>

class MathStd
{
public:
  static std::mt19937 RNG;

  static PV2D getClosest(const PV2D &pv, const Line2D &line);
  static PV2D getClosest(const PV2D &pv, const LineSegment2D &line);

  static Real dist(const PV2D &pv1, const PV2D &pv2);
  static Real distSquared(const PV2D &pv1, const PV2D &pv2);
  static Real dist(const PV2D &pv, const Line2D &line);
  static Real dist(const PV2D &pv, const Line2D &line, PV2D &pvClosest);
  static Real dist(const PV2D &pv, const LineSegment2D &line);
  static Real dist(const PV2D &pv, const LineSegment2D &line, PV2D &pvClosest);

  //angles are given as the first direction relative to the second
  static Real angle(const PV2D &direction);
  static Real angle(const PV2D &direction1, const PV2D &direction2);
  static Real angle(const PV3D &direction1, const PV3D &direction2);
  static Real angle(const PV2D &pv1, const PV2D &pv2, const PV2D &pv3);
  static Real angle(const Line2D &line1, const Line2D &line2);

  static PV2D transform(const PV2D &pv, const PV2D translation, const Real angle);
  static PV2D transform(const PV2D &pv, const PV2D translation, const Real angleSin, const Real angleCos);
  static Line2D transform(const Line2D &line, const PV2D translation, const Real angle);
  static LineSegment2D transform(const LineSegment2D &line, const PV2D translation, const Real angle);
  static PV2D transformInverse(const PV2D &pv, const PV2D translation, const Real angle);
  static PV2D transformInverse(const PV2D &pv, const PV2D translation, const Real angleSin, const Real angleCos);
  static Line2D transformInverse(const Line2D &line, const PV2D translation, const Real angle);
  static LineSegment2D transformInverse(const LineSegment2D &line, const PV2D translation, const Real angle);

  static PV2D rotateAroundPoint(const PV2D &pv, const PV2D &center, Real angle);
  static LineSegment2D rotateAroundPoint(const LineSegment2D &line, const PV2D &center, Real angle);

  static bool onLine(const PV2D &pv, const LineSegment2D &line);

  static bool isInside(const PV2D &pv, const Box2D &box);
  static bool isInside(const LineSegment2D &line, const Box2D &box);
  static bool isInside(const PV2D &pv, const PV2D &pvMin, const PV2D &pvMax);
  static bool isInside(const LineSegment2D &line, const PV2D &pvMin, const PV2D &pvMax);
  static bool isOutside(const PV2D &pv, const Box2D &box);
  static bool isOutside(const LineSegment2D &line, const Box2D &box);
  static bool isOutside(const PV2D &pv, const PV2D &pvMin, const PV2D &pvMax);
  static bool isOutside(const LineSegment2D &line, const PV2D &pvMin, const PV2D &pvMax);

  static PV2D getPointOnEllipse(const Real &angle, const Ellipse &ellipse);
  static PV3D getPointOnEllipsoid(const Real &height, const Real &angle, const Ellipsoid &ellipsoid);
  static PV2D getMidPoint(const PV2D &pv1, const PV2D & pv2);

  static bool getIntersect(const Line2D &line1, const Line2D &line2, PV2D &intersect);
  static bool getIntersect(const Line2D &line, const LineSegment2D &lineSegment, PV2D &intersect);
  static bool getIntersect(const LineSegment2D &lineSegment, const Line2D &line, PV2D &intersect);
  static bool getIntersect(const LineSegment2D &line1, const LineSegment2D &line2, PV2D &intersect);
  static void getIntersect(const LineSegment2D &line, const Box2D &box, std::vector<PV2D> &intersect);
  static void getIntersect(const Line2D &line, const Ellipse &ellipse, std::vector<PV2D> &intersect);
  static void getIntersect(const LineSegment2D &line, const Ellipse &ellipse, std::vector<PV2D> &intersect);
private:
  static bool getIntersectCommon(const Line2DCommon &line1, const Line2DCommon &line2, PV2D &intersect);
};

#endif //MATH_STD_H
