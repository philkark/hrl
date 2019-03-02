#ifndef MATH_STD_PV_H
#define MATH_STD_PV_H

#include<math_std/common.h>

struct PV2D;
struct PV3D;
struct PV4D;
struct LineSegment2D;
struct LineSegment3D;

struct PV2D
{
  Real C0, C1;

  PV2D();
  PV2D(const PV2D &pv);
  PV2D(const PV3D &pv);
  PV2D(Real c0, Real c1);
  void operator=(const PV3D &pv);
  PV2D operator-() const;
  PV2D operator+(const LineSegment2D &line) const;
  PV2D operator+(const PV2D &pv) const;
  PV2D operator-(const LineSegment2D &line) const;
  PV2D operator-(const PV2D &pv) const;
  void operator+=(const LineSegment2D &line);
  void operator+=(const PV2D &pv);
  void operator-=(const LineSegment2D &line);
  void operator-=(const PV2D &pv);
  Real operator*(const PV2D &pv) const;
  PV2D operator*(const Real &sca) const;
  PV2D operator/(const Real &sca) const;
  void operator*=(const Real &sca);
  void operator/=(const Real &sca);
  Real operator^(const PV2D &pv) const;
  bool operator==(const PV2D &pv) const;
  bool operator!=(const PV2D &pv) const;
  friend std::ostream& operator<<(std::ostream &out, PV2D &pv);

  void normalize();
  PV2D getNormalized();
  Real norm() const;
  Real normSquared() const;
};

struct PV3D
{
  Real C0, C1, C2;

  PV3D();
  PV3D(const PV3D &pv);
  PV3D(const PV2D &pv);
  PV3D(const PV2D &pv, const Real &c2);
  PV3D(Real c0, Real c1, Real c2);
  PV3D(const Real C);
  PV3D operator-() const;
  PV3D operator+(const LineSegment3D &line) const;
  PV3D operator+(const PV3D &pv) const;
  PV3D operator-(const LineSegment3D &line) const;
  PV3D operator-(const PV3D &pv) const;
  void operator+=(const LineSegment3D &line);
  void operator+=(const PV3D &pv);
  void operator+=(const PV2D &pv);
  void operator-=(const LineSegment3D &line);
  void operator-=(const PV3D &pv);
  Real operator*(const PV3D &pv) const;
  PV3D operator*(const Real &sca) const;
  PV3D operator/(const Real &sca) const;
  void operator*=(const Real &sca);
  void operator/=(const Real &sca);
  PV3D operator^(const PV3D &pv) const;
  bool operator==(const PV3D &pv) const;
  bool operator!=(const PV3D &pv) const;
  friend std::ostream& operator<<(std::ostream &out, PV3D &pv);

  void normalize();
  PV3D getNormalized();
  Real norm() const;
  Real normSquared() const;
};

struct PV4D
{
  Real C0, C1, C2, C3;

  PV4D();
  PV4D(const PV4D &pv);
  PV4D(Real c0, Real c1, Real c2, Real c3);
  PV4D operator-() const;
  PV4D operator+(const PV4D &pv) const;
  PV4D operator-(const PV4D &pv) const;
  void operator=(const PV3D &pv);
  void operator+=(const PV4D &pv);
  void operator-=(const PV4D &pv);
  Real operator*(const PV4D &pv) const;
  PV4D operator*(const Real &sca) const;
  void operator*=(const Real &sca);
  void operator/=(const Real &sca);
  bool operator==(const PV4D &pv) const;
  bool operator!=(const PV4D &pv) const;
  friend std::ostream& operator<<(std::ostream &out, PV4D &pv);

  void normalize();
  Real norm() const;
  Real normSquared() const;
};

struct Tuple3PV2D
{
  PV2D first, second, third;
};

PV2D operator*(const Real &sca, const PV2D &pv);
PV3D operator*(const Real &sca, const PV3D &pv);
PV4D operator*(const Real &sca, const PV4D &pv);

#endif //MATH_STD_PV_H

