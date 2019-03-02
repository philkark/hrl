#ifndef MATH_STD_LINE_H
#define MATH_STD_LINE_H

#include<math_std/common.h>
#include<math_std/pv.h>

struct Line2DCommon
{
  PV2D start, direction;
};

struct Line3DCommon
{
  PV3D start, direction;
};

struct Line2D : Line2DCommon
{
  friend std::ostream& operator<<(std::ostream &out, Line2D &line);

  Line2D();
  Line2D(const PV2D &start, const PV2D &direction);
  void setDirection(const PV2D &pv1, const PV2D &pv2);
  void setDirection(const PV2D &directionNew);
};

struct Line3D : Line3DCommon
{
  friend std::ostream& operator<<(std::ostream &out, Line3D &line);

  Line3D();
  Line3D(const PV3D &start, const PV3D &direction);
  void setDirection(const PV3D &pv1, const PV3D &pv2);
  void setDirection(const PV3D &directionNew);
};

struct LineSegment2D : Line2DCommon
{
  PV2D end, directionFull;
  Real length;

  LineSegment2D();
  LineSegment2D(const PV2D &startInit, const PV2D &endInit);
  bool operator==(const LineSegment2D &line) const;
  friend std::ostream& operator<<(std::ostream &out, LineSegment2D &line);

  void setPoints(const PV2D &startNew, const PV2D &endNew);

  PV2D fromStartAbs(Real distance) const;
  PV2D fromStartRel(Real percent) const;
  PV2D fromEndAbs(Real distance) const;
  PV2D fromEndRel(Real percent) const;
  Line2D toLine() const;
};

struct LineSegment3D : Line3DCommon
{
  PV3D end, directionFull;
  Real length;
  friend std::ostream& operator<<(std::ostream &out, LineSegment3D &line);

  LineSegment3D();
  LineSegment3D(const PV3D &startInit, const PV3D &endInit);
  void setPoints(const PV3D &startNew, const PV3D &endNew);

  PV3D fromStartAbs(Real distance) const;
  PV3D fromStartRel(Real percent) const;
  PV3D fromEndAbs(Real distance) const;
  PV3D fromEndRel(Real percent) const;
  Line3D toLine() const;
};


#endif //MATH_PATH_LINE_H
