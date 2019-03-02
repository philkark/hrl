#include<math_std/math_std.h>

std::random_device RD;
std::mt19937 rng(RD());
std::mt19937 MathStd::RNG = rng;

PV2D MathStd::getClosest(const PV2D &pv, const Line2D &line)
{
  return line.start + line.direction * ((pv - line.start) * line.direction);
}
PV2D MathStd::getClosest(const PV2D &pv, const LineSegment2D &line)
{
  const PV2D pvClosest = line.start + line.direction * ((pv - line.start) * line.direction);

  if (MathStd::dist(pvClosest, line.start) <= line.length && MathStd::dist(pvClosest, line.end) <= line.length)
    return pvClosest;

  Real distStart = MathStd::dist(pv, line.start), distEnd = MathStd::dist(pv, line.end);

  if (distStart <= distEnd)
    return line.start;
  else
    return line.end;
}

Real MathStd::dist(const PV2D &pv1, const PV2D &pv2)
{
  return (pv1 - pv2).norm();
}
Real MathStd::distSquared(const PV2D &pv1, const PV2D &pv2)
{
  return (pv1 - pv2).normSquared();
}
Real MathStd::dist(const PV2D &pv, const Line2D &line)
{
  PV2D pvTmp = line.start + line.direction * ((pv - line.start) * line.direction);
  return MathStd::dist(pv, pvTmp);
}
Real MathStd::dist(const PV2D &pv, const Line2D &line, PV2D &pvClosest)
{
  pvClosest = line.start + line.direction * ((pv - line.start) * line.direction);
  return MathStd::dist(pv, pvClosest);
}
Real MathStd::dist(const PV2D &pv, const LineSegment2D &line)
{
  const PV2D pvClosest = line.start + line.direction * ((pv - line.start) * line.direction);

  if (MathStd::dist(pvClosest, line.start) <= line.length && MathStd::dist(pvClosest, line.end) <= line.length)
    return MathStd::dist(pv, pvClosest);

  Real distStart = MathStd::dist(pv, line.start), distEnd = MathStd::dist(pv, line.end);

  if (distStart <= distEnd)
    return distStart;
  else
    return distEnd;
}
Real MathStd::dist(const PV2D &pv, const LineSegment2D &line, PV2D &pvClosest)
{
  pvClosest = line.start + line.direction * ((pv - line.start) * line.direction);

  if (MathStd::dist(pvClosest, line.start) <= line.length && MathStd::dist(pvClosest, line.end) <= line.length)
    return MathStd::dist(pv, pvClosest);

  Real distStart = MathStd::dist(pv, line.start), distEnd = MathStd::dist(pv, line.end);

  if (distStart <= distEnd)
  {
    pvClosest = line.start;
    return distStart;
  }
  else
  {
    pvClosest = line.end;
    return distEnd;
  }
}

Real MathStd::angle(const PV2D &direction)
{
  Real arg = direction.C0 / direction.norm();
  if (arg > 1.0)
    return 0.0;
  else if (arg < -1.0)
    return M_PI;

  if (direction.C1 >= 0)
    return acos(arg);
  else
    return -acos(arg);
}
Real MathStd::angle(const PV2D &direction1, const PV2D &direction2)
{
  Real arg = direction1 * direction2 / sqrt(direction1.normSquared() * direction2.normSquared());
  if (arg > 1.0)
    return 0;
  else if (arg < -1.0)
    return M_PI;

  if ((direction1 ^ direction2) <= 0)
    return acos(arg);
  else
    return -acos(arg);
}
Real MathStd::angle(const PV3D &direction1, const PV3D &direction2)
{
  Real arg = direction1 * direction2 / sqrt(direction1.normSquared() * direction2.normSquared());
  if (arg > 1.0)
    return 0;
  else if (arg < -1.0)
    return M_PI;
  else
    return acos(arg);
}

Real MathStd::angle(const PV2D &pv1, const PV2D &pv2, const PV2D &pv3)
{
  PV2D pvDiff1 = pv1 - pv2, pvDiff2 = pv3 - pv2;
  return MathStd::angle(pvDiff1, pvDiff2);
}
Real MathStd::angle(const Line2D &line1, const Line2D &line2)
{
  return MathStd::angle(line1.direction, line2.direction);
}

PV2D MathStd::transformInverse(const PV2D &pv, const PV2D translation, const Real angle)
{
  PV2D pvTmp(pv - translation);
  Real cosTmp = cos(angle), sinTmp = sin(angle);
  Real xNew = cosTmp * pvTmp.C0 + sinTmp * pvTmp.C1, yNew = -sinTmp * pvTmp.C0 + cosTmp * pvTmp.C1;
  return PV2D(std::abs(xNew) < TINYFLOAT ? 0 : xNew, std::abs(yNew) < TINYFLOAT ? 0 : yNew);
}
PV2D MathStd::transformInverse(const PV2D &pv, const PV2D translation, const Real angleSin, const Real angleCos)
{
  PV2D pvTmp(pv - translation);
  Real xNew = angleCos * pvTmp.C0 + angleSin * pvTmp.C1, yNew = -angleSin * pvTmp.C0 + angleCos * pvTmp.C1;
  return PV2D(std::abs(xNew) < TINYFLOAT ? 0 : xNew, std::abs(yNew) < TINYFLOAT ? 0 : yNew);
}
Line2D MathStd::transformInverse(const Line2D &line, const PV2D translation, const Real angle)
{
  PV2D pv1 = transformInverse(line.start, translation, angle), pv2 = transformInverse(line.start + line.direction, translation, angle);
  return Line2D(pv1, pv2 - pv1);
}
LineSegment2D MathStd::transformInverse(const LineSegment2D &line, const PV2D translation, const Real angle)
{
  PV2D startNew = transformInverse(line.start, translation, angle), endNew = transformInverse(line.end, translation, angle);
  return LineSegment2D(startNew, endNew);
}
PV2D MathStd::transform(const PV2D &pv, const PV2D translation, const Real angle)
{
  Real cosTmp = cos(angle), sinTmp = sin(angle);
  Real xNew = cosTmp * pv.C0 - sinTmp * pv.C1, yNew = sinTmp * pv.C0 + cosTmp * pv.C1;
  return PV2D(std::abs(xNew) < TINYFLOAT ? 0 : xNew, std::abs(yNew) < TINYFLOAT ? 0 : yNew) + translation;
}
PV2D MathStd::transform(const PV2D &pv, const PV2D translation, const Real angleSin, const Real angleCos)
{
  Real xNew = angleCos * pv.C0 - angleSin * pv.C1, yNew = angleSin * pv.C0 + angleCos * pv.C1;
  return PV2D(std::abs(xNew) < TINYFLOAT ? 0 : xNew, std::abs(yNew) < TINYFLOAT ? 0 : yNew) + translation;
}
Line2D MathStd::transform(const Line2D &line, const PV2D translation, const Real angle)
{
  PV2D pv1 = transform(line.start, translation, angle), pv2 = transform(line.start + line.direction, translation, angle);
  return Line2D(pv1, pv2 - pv1);
}
LineSegment2D MathStd::transform(const LineSegment2D &line, const PV2D translation, const Real angle)
{
  PV2D startNew = transform(line.start, translation, angle), endNew = transform(line.end, translation, angle);
  return LineSegment2D(startNew, endNew);
}

PV2D MathStd::rotateAroundPoint(const PV2D &pv, const PV2D &center, Real angle)
{
  return transform(transformInverse(pv, center, -angle), center, 0);
}
LineSegment2D MathStd::rotateAroundPoint(const LineSegment2D &line, const PV2D &center, Real angle)
{
  return transform(transformInverse(line, center, -angle), center, 0);
}

bool MathStd::onLine(const PV2D &pv, const LineSegment2D &line)
{
  if (dist(pv, line) > TINYFLOAT)
    return false;
  else
    return true;
}

bool MathStd::isInside(const PV2D &pv, const Box2D &box)
{
  if (pv.C0 < box.minimum.C0 || pv.C0 > box.maximum.C0 || pv.C1 < box.minimum.C1 || pv.C1 > box.maximum.C1)
    return false;
  else
    return true;
}
bool MathStd::isInside(const LineSegment2D &line, const Box2D &box)
{
  if (line.end.C0 < box.minimum.C0 || line.end.C0 > box.maximum.C0 || line.end.C1 < box.minimum.C1 || line.end.C1 > box.maximum.C1 || line.start.C0 < box.minimum.C0
      || line.start.C0 > box.maximum.C0 || line.start.C1 < box.minimum.C1 || line.start.C1 > box.maximum.C1)
    return false;
  else
    return true;
}
bool MathStd::isInside(const PV2D &pv, const PV2D &pvMin, const PV2D &pvMax)
{
  if (pv.C0 < pvMin.C0 || pv.C0 > pvMax.C0 || pv.C1 < pvMin.C1 || pv.C1 > pvMax.C1)
    return false;
  else
    return true;
}
bool MathStd::isInside(const LineSegment2D &line, const PV2D &pvMin, const PV2D &pvMax)
{
  if (line.end.C0 < pvMin.C0 || line.end.C0 > pvMax.C0 || line.end.C1 < pvMin.C1 || line.end.C1 > pvMax.C1 || line.start.C0 < pvMin.C0 || line.start.C0 > pvMax.C0
      || line.start.C1 < pvMin.C1 || line.start.C1 > pvMax.C1)
    return false;
  else
    return true;
}
bool MathStd::isOutside(const PV2D &pv, const Box2D &box)
{
  if (pv.C0 > box.minimum.C0 && pv.C0 < box.maximum.C0 && pv.C1 > box.minimum.C1 && pv.C1 < box.maximum.C1)
    return false;
  else
    return true;
}
bool MathStd::isOutside(const LineSegment2D &line, const Box2D &box)
{
  if (line.end.C0 < box.minimum.C0 && line.end.C0 > box.maximum.C0 && line.end.C1 < box.minimum.C1 && line.end.C1 > box.maximum.C1 && line.start.C0 < box.minimum.C0
      && line.start.C0 > box.maximum.C0 && line.start.C1 < box.minimum.C1 && line.start.C1 > box.maximum.C1)
  {
    std::vector<PV2D> intersects;
    getIntersect(line, box, intersects);
    if (intersects.size() > 0)
      return false;
    else
      return true;
  }
  else
    return false;
}
bool MathStd::isOutside(const PV2D &pv, const PV2D &pvMin, const PV2D &pvMax)
{
  if (pv.C0 > pvMin.C0 && pv.C0 < pvMax.C0 && pv.C1 > pvMin.C1 && pv.C1 < pvMax.C1)
    return false;
  else
    return true;
}
bool MathStd::isOutside(const LineSegment2D &line, const PV2D &pvMin, const PV2D &pvMax)
{
  if (line.end.C0 < pvMin.C0 && line.end.C0 > pvMax.C0 && line.end.C1 < pvMin.C1 && line.end.C1 > pvMax.C1 && line.start.C0 < pvMin.C0 && line.start.C0 > pvMax.C0
      && line.start.C1 < pvMin.C1 && line.start.C1 > pvMax.C1)
  {
    std::vector<PV2D> intersects;
    getIntersect(line, Box2D(pvMin, pvMax), intersects);
    if (intersects.size() > 0)
      return false;
    else
      return true;
  }
  else
    return false;
}

PV2D MathStd::getPointOnEllipse(const Real &angle, const Ellipse &ellipse)
{
  Real angleTmp = angle;
  while (angleTmp > M_PI)
    angleTmp -= PITWO;
  while (angleTmp < -M_PI)
    angleTmp += PITWO;

  if (angleTmp == PIHALF)
    return PV2D(0, ellipse.radiusY);
  else if (angleTmp == - PIHALF)
    return PV2D(0, -ellipse.radiusY);
  else
  {
    PV2D pvNew(ellipse.radiusX * ellipse.radiusY / sqrt(ellipse.radiusY * ellipse.radiusY + pow(ellipse.radiusX * tan(angleTmp), 2)),
               ellipse.radiusX * ellipse.radiusY / sqrt(ellipse.radiusX * ellipse.radiusX + pow(ellipse.radiusY * tan(PIHALF - angleTmp), 2)));
    if (angleTmp < 0)
      pvNew.C1 *= -1.0;
    if (angleTmp > PIHALF || angleTmp < -PIHALF)
      pvNew.C0 *= -1.0;

    if (std::abs(pvNew.C0) < TINYFLOAT)
      pvNew.C0 = 0.0;
    if (std::abs(pvNew.C1) < TINYFLOAT)
      pvNew.C1 = 0.0;

    return pvNew;
  }
}
PV3D MathStd::getPointOnEllipsoid(const Real &height, const Real &angle, const Ellipsoid &ellipsoid)
{
  if (height >= ellipsoid.radiusZ)
    return PV3D(0, 0, ellipsoid.radiusZ);
  else if (height <= -ellipsoid.radiusZ)
    return PV3D(0, 0, -ellipsoid.radiusZ);

  Real scaleFactor = sqrt(1 - pow(height / ellipsoid.radiusZ, 2));
  PV2D ellipsePoint = getPointOnEllipse(angle, Ellipse(ellipsoid.radiusX * scaleFactor, ellipsoid.radiusY * scaleFactor));

  return PV3D(ellipsePoint.C0, ellipsePoint.C1, height);
}
PV2D MathStd::getMidPoint(const PV2D &pv1, const PV2D & pv2)
{
  return PV2D((pv2.C0 + pv1.C0) / 2.0, (pv2.C1 + pv1.C1) / 2.0);
}

bool MathStd::getIntersectCommon(const Line2DCommon &line1, const Line2DCommon &line2, PV2D &intersect)
{
  Real numX, numY, denom;

  denom = line1.direction.C1 * line2.direction.C0 - line1.direction.C0 * line2.direction.C1;
  if (std::abs(denom) < TINYFLOAT)
    return false;

  numX = line1.start.C0 * line1.direction.C1 * line2.direction.C0 - line2.start.C0 * line1.direction.C0 * line2.direction.C1
      - line1.direction.C0 * line2.direction.C0 * (line1.start.C1 - line2.start.C1);
  numY = -line1.start.C1 * line1.direction.C0 * line2.direction.C1 + line2.start.C1 * line1.direction.C1 * line2.direction.C0
      + line1.direction.C1 * line2.direction.C1 * (line1.start.C0 - line2.start.C0);

  intersect.C0 = numX / denom;
  intersect.C1 = numY / denom;

  return true;
}
bool MathStd::getIntersect(const Line2D &line1, const Line2D &line2, PV2D &intersect)
{
  return getIntersectCommon(line1, line2, intersect);
}
bool MathStd::getIntersect(const Line2D &line, const LineSegment2D &lineSegment, PV2D &intersect)
{
  bool initialIntersect;
  initialIntersect = getIntersectCommon(line, lineSegment, intersect);
  if (!initialIntersect)
    return false;

  if (dist(intersect, lineSegment, intersect) < TINYFLOAT)
    return true;
  else
    return false;
}
bool MathStd::getIntersect(const LineSegment2D &lineSegment, const Line2D &line, PV2D &intersect)
{
  return getIntersectCommon(line, lineSegment, intersect);
}
bool MathStd::getIntersect(const LineSegment2D &line1, const LineSegment2D &line2, PV2D &intersect)
{
  bool initialIntersect;
  initialIntersect = getIntersectCommon(line1, line2, intersect);
  if (!initialIntersect)
    return false;

  if (dist(intersect, line1, intersect) < TINYFLOAT && dist(intersect, line2, intersect) < TINYFLOAT)
    return true;
  else
    return false;
}
void MathStd::getIntersect(const LineSegment2D &line, const Box2D &box, std::vector<PV2D> &intersect)
{
  LineSegment2D lineTmp;
  PV2D pvTmp;

  if (getIntersect(line, LineSegment2D(box.minimum, PV2D(box.minimum.C0, box.maximum.C1)), pvTmp))
    intersect.push_back(pvTmp);
  if (getIntersect(line, LineSegment2D(box.minimum, PV2D(box.maximum.C0, box.minimum.C1)), pvTmp))
    intersect.push_back(pvTmp);
  if (getIntersect(line, LineSegment2D(box.maximum, PV2D(box.minimum.C0, box.maximum.C1)), pvTmp))
    intersect.push_back(pvTmp);
  if (getIntersect(line, LineSegment2D(box.maximum, PV2D(box.maximum.C0, box.minimum.C1)), pvTmp))
    intersect.push_back(pvTmp);
}
void MathStd::getIntersect(const Line2D &line, const Ellipse &ellipse, std::vector<PV2D> &intersect)
{
  if (std::abs(line.direction.C0) > TINYFLOAT)
  {
    Real alpha = line.direction.C1 / line.direction.C0, beta = line.start.C1 - line.direction.C1 * line.start.C0 / line.direction.C0;
    Real a = 1 / (ellipse.radiusX * ellipse.radiusX), b = 1 / (ellipse.radiusY * ellipse.radiusY);
    Real A = a + b * alpha * alpha, B = 2 * alpha * beta * b, C = beta * beta * b - 1;
    Real tmp, rootArg = B * B - 4 * A * C;

    if (rootArg < 0.0)
      return;
    else if (rootArg < TINYFLOAT)
    {
      tmp = -B * 0.5 / A;
      intersect.push_back(PV2D(tmp, alpha * tmp + beta));
    }
    else
    {
      tmp = (-B + sqrt(rootArg)) * 0.5 / A;
      intersect.push_back(PV2D(tmp, alpha * tmp + beta));
      tmp = (-B - sqrt(rootArg)) * 0.5 / A;
      intersect.push_back(PV2D(tmp, alpha * tmp + beta));
    }
  }
  else
  {
    Real rootArg = 1 - line.start.C0 * line.start.C0 / (ellipse.radiusX * ellipse.radiusX);

    if (rootArg < 0.0)
      return;
    intersect.push_back(PV2D(line.start.C0, ellipse.radiusY * sqrt(rootArg)));
    if (rootArg > 0.0)
      intersect.push_back(PV2D(line.start.C0, -ellipse.radiusY * sqrt(rootArg)));
    return;
  }
}
void MathStd::getIntersect(const LineSegment2D &line, const Ellipse &ellipse, std::vector<PV2D> &intersect)
{
  if (line.direction.C0 != 0.0)
  {
    Real alpha = line.direction.C1 / line.direction.C0, beta = line.start.C1 - line.direction.C1 * line.start.C0 / line.direction.C0;
    Real a = 1 / (ellipse.radiusX * ellipse.radiusX), b = ellipse.radiusY * ellipse.radiusY;
    Real A = a * b + alpha * alpha, B = 2 * alpha * beta * b, C = beta * beta - b;
    Real tmp, rootArg = B * B - 4 * A * C;

    if (rootArg < TINYFLOAT)
    {
      tmp = -B * 0.5 / A;
      PV2D intersect1Tmp(tmp, alpha * tmp + beta);
      if (MathStd::dist(intersect1Tmp, line, intersect1Tmp) < TINYFLOAT)
        intersect.push_back(intersect1Tmp);
    }
    else if (rootArg > 0.0)
    {
      tmp = (-B + sqrt(rootArg)) * 0.5 / A;
      PV2D intersect1Tmp(tmp, alpha * tmp + beta);
      if (MathStd::dist(intersect1Tmp, line, intersect1Tmp) < TINYFLOAT)
        intersect.push_back(intersect1Tmp);
      tmp = (-B - sqrt(rootArg)) * 0.5 / A;
      PV2D intersect2Tmp(tmp, alpha * tmp + beta);
      if (MathStd::dist(intersect2Tmp, line, intersect2Tmp) < TINYFLOAT)
        intersect.push_back(intersect2Tmp);
    }
  }
  else
  {
    Real rootArg = 1 - line.start.C0 * line.start.C0 / (ellipse.radiusX * ellipse.radiusX);

    if (rootArg == 0.0)
      intersect.push_back(PV2D(line.start.C0, 0.0));
    else if (rootArg > 0.0)
    {
      intersect.push_back(PV2D(line.start.C0, ellipse.radiusY * sqrt(rootArg)));
      intersect.push_back(PV2D(line.start.C0, -ellipse.radiusY * sqrt(rootArg)));
    }
    return;
  }
}
