#include<math_std/pv.h>
#include<math_std/line.h>

//############# PV2D #############

PV2D::PV2D() :
    C0(0), C1(0)
{
}
PV2D::PV2D(Real C0, Real C1) :
    C0(C0), C1(C1)
{
}
PV2D::PV2D(const PV2D &pv) :
    C0(pv.C0), C1(pv.C1)
{
}
PV2D::PV2D(const PV3D &pv) :
    C0(pv.C0), C1(pv.C1)
{
}
void PV2D::operator=(const PV3D &pv)
{
  C0 = pv.C0;
  C1 = pv.C1;
}
PV2D PV2D::operator-() const
{
  return PV2D(-C0, -C1);
}
PV2D PV2D::operator+(const PV2D &pv) const
{
  return PV2D(C0 + pv.C0, C1 + pv.C1);
}
PV2D PV2D::operator+(const LineSegment2D &line) const
{
  return PV2D(C0 + line.directionFull.C0, C1 + line.directionFull.C1);
}
PV2D PV2D::operator-(const PV2D &pv) const
{
  return PV2D(C0 - pv.C0, C1 - pv.C1);
}
PV2D PV2D::operator-(const LineSegment2D &line) const
{
  return PV2D(C0 - line.directionFull.C0, C1 - line.directionFull.C1);
}
void PV2D::operator+=(const PV2D &pv)
{
  C0 += pv.C0;
  C1 += pv.C1;
}
void PV2D::operator+=(const LineSegment2D &line)
{
  C0 += line.directionFull.C0;
  C1 += line.directionFull.C1;
}
void PV2D::operator-=(const PV2D &pv)
{
  C0 -= pv.C0;
  C1 -= pv.C1;
}
void PV2D::operator-=(const LineSegment2D &line)
{
  C0 -= line.directionFull.C0;
  C1 -= line.directionFull.C1;
}
Real PV2D::operator*(const PV2D &pv) const
{
  return C0 * pv.C0 + C1 * pv.C1;
}
PV2D PV2D::operator*(const Real &sca) const
{
  return PV2D(C0 * sca, C1 * sca);
}
PV2D PV2D::operator/(const Real &sca) const
{
  return PV2D(C0 / sca, C1 / sca);
}
void PV2D::operator*=(const Real &sca)
{
  C0 *= sca;
  C1 *= sca;
}
void PV2D::operator/=(const Real &sca)
{
  C0 /= sca;
  C1 /= sca;
}
Real PV2D::operator^(const PV2D &pv) const
{
  return C0 * pv.C1 - C1 * pv.C0;
}
bool PV2D::operator==(const PV2D &pv) const
{
  return C0 == pv.C0 && C1 == pv.C1;
}
bool PV2D::operator!=(const PV2D &pv) const
{
  return !(C0 == pv.C0 && C1 == pv.C1);
}
std::ostream& operator<<(std::ostream &out, PV2D &pv)
{
  out << pv.C0 << ", " << pv.C1 << std::endl;
  return out;
}
PV2D operator*(const Real &sca, const PV2D &pv)
{
  return PV2D(sca * pv.C0, sca * pv.C1);
}

void PV2D::normalize()
{
  Real norm = this->norm();
  if (norm < TINYFLOAT)
    return;

  *this /= norm;
}
PV2D PV2D::getNormalized()
{
  return *this / this->norm();
}
Real PV2D::norm() const
{
  return sqrt(C0 * C0 + C1 * C1);
}
Real PV2D::normSquared() const
{
  return C0 * C0 + C1 * C1;
}

//############# PV3D #############

PV3D::PV3D() :
    C0(0), C1(0), C2(0)
{
}
PV3D::PV3D(Real c0, Real c1, Real c2) :
    C0(c0), C1(c1), C2(c2)
{
}
PV3D::PV3D(const PV3D &pv) :
    C0(pv.C0), C1(pv.C1), C2(pv.C2)
{
}
PV3D::PV3D(const PV2D &pv) :
    C0(pv.C0), C1(pv.C1), C2(0.0)
{
}
PV3D::PV3D(const PV2D &pv, const Real &c2):
        C0(pv.C0), C1(pv.C1), C2(c2)
{

}
PV3D::PV3D(const Real C) : C0(C), C1(C), C2(C)
{

}
PV3D PV3D::operator-() const
{
  return PV3D(-C0, -C1, -C2);
}
PV3D PV3D::operator+(const PV3D &pv) const
{
  return PV3D(C0 + pv.C0, C1 + pv.C1, C2 + pv.C2);
}
PV3D PV3D::operator+(const LineSegment3D &line) const
{
  return PV3D(C0 + line.directionFull.C0, C1 + line.directionFull.C1, C2 + line.directionFull.C2);
}
PV3D PV3D::operator-(const PV3D &pv) const
{
  return PV3D(C0 - pv.C0, C1 - pv.C1, C2 - pv.C2);
}
PV3D PV3D::operator-(const LineSegment3D &line) const
{
  return PV3D(C0 - line.directionFull.C0, C1 - line.directionFull.C1, C2 - line.directionFull.C2);
}
void PV3D::operator+=(const PV3D &pv)
{
  C0 += pv.C0;
  C1 += pv.C1;
  C2 += pv.C2;
}
void PV3D::operator+=(const PV2D &pv)
{
  C0 += pv.C0;
  C1 += pv.C1;
}
void PV3D::operator+=(const LineSegment3D &line)
{
  C0 += line.directionFull.C0;
  C1 += line.directionFull.C1;
  C2 += line.directionFull.C2;
}
void PV3D::operator-=(const PV3D &pv)
{
  C0 -= pv.C0;
  C1 -= pv.C1;
  C2 -= pv.C2;
}
void PV3D::operator-=(const LineSegment3D &line)
{
  C0 -= line.directionFull.C0;
  C1 -= line.directionFull.C1;
  C2 -= line.directionFull.C2;
}
Real PV3D::operator*(const PV3D &pv) const
{
  return C0 * pv.C0 + C1 * pv.C1 + C2 * pv.C2;
}
PV3D PV3D::operator*(const Real &sca) const
{
  return PV3D(C0 * sca, C1 * sca, C2 * sca);
}
PV3D PV3D::operator/(const Real &sca) const
{
  return PV3D(C0 / sca, C1 / sca, C2 / sca);
}
void PV3D::operator*=(const Real &sca)
{
  C0 *= sca;
  C1 *= sca;
  C2 *= sca;
}
void PV3D::operator/=(const Real &sca)
{
  C0 /= sca;
  C1 /= sca;
  C2 /= sca;
}
PV3D PV3D::operator^(const PV3D &pv) const
{
  return PV3D(C1 * pv.C2 - C2 * pv.C1, C2 * pv.C0 - C0 * pv.C2, C0 * pv.C1 - C1 * pv.C0);
}
bool PV3D::operator==(const PV3D &pv) const
{
  return C0 == pv.C0 && C1 == pv.C1 && C2 == pv.C2;
}
bool PV3D::operator!=(const PV3D &pv) const
{
  return !(C0 == pv.C0 && C1 == pv.C1 && C2 == pv.C2);
}
std::ostream& operator<<(std::ostream &out, PV3D &pv)
{
  out << pv.C0 << ", " << pv.C1 << ", " << pv.C2 << std::endl;
  return out;
}
PV3D operator*(const Real &sca, const PV3D &pv)
{
  return PV3D(sca * pv.C0, sca * pv.C1, sca * pv.C2);
}

void PV3D::normalize()
{
  Real norm = this->norm();
  if (norm < TINYFLOAT)
    return;

  *this /= norm;
}
PV3D PV3D::getNormalized()
{
  return *this / this->norm();
}
Real PV3D::norm() const
{
  return sqrt(C0 * C0 + C1 * C1 + C2 * C2);
}
Real PV3D::normSquared() const
{
  return C0 * C0 + C1 * C1 + C2 * C2;
}

//############# PV4D #############

PV4D::PV4D() :
    C0(0), C1(0), C2(0), C3(0)
{
}
PV4D::PV4D(Real c0, Real c1, Real c2, Real c3) :
    C0(c0), C1(c1), C2(c2), C3(c3)
{
}
PV4D::PV4D(const PV4D &pv) :
    C0(pv.C0), C1(pv.C1), C2(pv.C2), C3(pv.C3)
{
}
PV4D PV4D::operator-() const
{
  return PV4D(-C0, -C1, -C2, -C3);
}
PV4D PV4D::operator+(const PV4D &pv) const
{
  return PV4D(C0 + pv.C0, C1 + pv.C1, C2 + pv.C2, C3 + pv.C3);
}
PV4D PV4D::operator-(const PV4D &pv) const
{
  return PV4D(C0 - pv.C0, C1 - pv.C1, C2 - pv.C2, C3 - pv.C3);
}
void PV4D::operator=(const PV3D &pv)
{
  C0 = pv.C0;
  C1 = pv.C1;
  C2 = pv.C2;
}
void PV4D::operator+=(const PV4D &pv)
{
  C0 += pv.C0;
  C1 += pv.C1;
  C2 += pv.C2;
  C3 += pv.C3;
}
void PV4D::operator-=(const PV4D &pv)
{
  C0 -= pv.C0;
  C1 -= pv.C1;
  C2 -= pv.C2;
  C3 -= pv.C3;
}
Real PV4D::operator*(const PV4D &pv) const
{
  return C0 * pv.C0 + C1 * pv.C1 + C2 * pv.C2 + C3 * pv.C3;
}
PV4D PV4D::operator*(const Real &sca) const
{
  return PV4D(C0 * sca, C1 * sca, C2 * sca, C3 * sca);
}
void PV4D::operator*=(const Real &sca)
{
  C0 *= sca;
  C1 *= sca;
  C2 *= sca;
  C3 *= sca;
}
void PV4D::operator/=(const Real &sca)
{
  C0 /= sca;
  C1 /= sca;
  C2 /= sca;
  C3 /= sca;
}
bool PV4D::operator==(const PV4D &pv) const
{
  return C0 == pv.C0 && C1 == pv.C1 && C2 == pv.C2 && C3 == pv.C3;
}
bool PV4D::operator!=(const PV4D &pv) const
{
  return !(C0 == pv.C0 && C1 == pv.C1 && C2 == pv.C2 && C3 == pv.C3);
}
std::ostream& operator<<(std::ostream &out, PV4D &pv)
{
  out << pv.C0 << ", " << pv.C1 << ", " << pv.C2 << ", " << pv.C3 << std::endl;
  return out;
}
PV4D operator*(const Real &sca, const PV4D &pv)
{
  return PV4D(sca * pv.C0, sca * pv.C1, sca * pv.C2, sca * pv.C3);
}

void PV4D::normalize()
{
  Real norm = this->norm();
  if (norm < TINYFLOAT)
    return;

  *this /= norm;
}
Real PV4D::norm() const
{
  return sqrt(C0 * C0 + C1 * C1 + C2 * C2 + C3 * C3);
}
Real PV4D::normSquared() const
{
  return C0 * C0 + C1 * C1 + C2 * C2 + C3 * C3;
}
