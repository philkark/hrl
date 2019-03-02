#include <dynamic_astar_grid_planner/dynamic_astar_grid_planner_structs.h>

//////////////////////////////////////////////////////////////////

//////////////////// ASTARPATH2D /////////////////////////////////

//////////////////////////////////////////////////////////////////

AStarPath2D::AStarPath2D() : valid(false)
{
}

//////////////////////////////////////////////////////////////////

//////////////////// LINESEG2D ///////////////////////////////////

//////////////////////////////////////////////////////////////////

LineSeg2D::LineSeg2D()
{
}

LineSeg2D::LineSeg2D(const PV2D &start, const PV2D &end) :
    start(start), end(end)
{
  length = MathStd::dist(end, start);
  if (length < TINY_FLT)
  {
    length = 0.0;
    direction = PV2D(0.0, 0.0);
    angle = 0.0;
  }
  else
  {
    direction = (end - start) / length;
    angle = atan2(direction.C1, direction.C0);
  }
}

void LineSeg2D::setPoints(const PV2D &start, const PV2D &end)
{
  this->start = start;
  this->end = end;
  length = MathStd::dist(end, start);
  if (length < TINY_FLT)
  {
    length = 0.0;
    direction = PV2D(0.0, 0.0);
  }
  else
    direction = (end - start) / length;
}

void LineSeg2D::clipStart(const Real &distance)
{
  if (distance >= length)
  {
    start = end;
    length = 0.0;
  }
  else
  {
    start = start + direction * distance;
    length -= distance;
  }
}

void LineSeg2D::clipEnd(const Real &distance)
{
  if (distance >= length)
  {
    end = start;
    length = 0.0;
  }
  else
  {
    end = end - direction * distance;
    length -= distance;
  }
}

void LineSeg2D::clipBoth(const Real &distance)
{
  if (distance * 2 >= length)
  {
    start = end = start + direction * 0.5 * length;
    length = 0.0;
  }
  else
  {
    start = start + direction * distance;
    end = end - direction * distance;
    length -= 2 * distance;
  }
}

PV2D LineSeg2D::getPointAbsolute(const Real &distanceAbsolute) const
{
  return start + direction * distanceAbsolute;
}

PV2D LineSeg2D::getPointRelative(const Real &distanceRelative) const
{
  return start + direction * distanceRelative * length;
}

PV2D LineSeg2D::getPointMiddle() const
{
  return getPointRelative(0.5);
}

//////////////////////////////////////////////////////////////////

//////////////////// PARAMETRICFUNCTIONCUBIC2D ///////////////////

//////////////////////////////////////////////////////////////////

ParametricFunctionCubic2D::ParametricFunctionCubic2D()
{
  xA = xB = xC = xD = yA = yB = yC = yD = length = 0;
  lengthRecip = std::numeric_limits<Real>::infinity();
}

ParametricFunctionCubic2D::ParametricFunctionCubic2D(const PV2D &pointStart, const PV2D &pointEnd, const PV2D &pointRef, const Real &smoothingFactor,
                                                     const UInt &lengthDiscretization)
{
  xA = (pointStart.C0 - pointEnd.C0) * (2 - smoothingFactor);
  xB = 3 * (pointEnd.C0 - pointStart.C0) + smoothingFactor * (2 * pointStart.C0 - pointEnd.C0 - pointRef.C0);
  xC = smoothingFactor * (pointRef.C0 - pointStart.C0);
  xD = pointStart.C0;
  yA = (pointStart.C1 - pointEnd.C1) * (2 - smoothingFactor);
  yB = 3 * (pointEnd.C1 - pointStart.C1) + smoothingFactor * (2 * pointStart.C1 - pointEnd.C1 - pointRef.C1);
  yC = smoothingFactor * (pointRef.C1 - pointStart.C1);
  yD = pointStart.C1;

  length = 0.0;
  const Real stepSize = 1.0 / lengthDiscretization;
  Real t = 0.0;
  for (UInt i = 0; i < lengthDiscretization; ++i, t += stepSize)
  {
    const PV2D point1 = getPointParametric(t);
    const PV2D point2 = getPointParametric(t + stepSize);
    const Real dx = point1.C0 - point2.C0;
    const Real dy = point1.C1 - point2.C1;
    length += sqrt(dx * dx + dy * dy);
  }

  if (length > 0)
    lengthRecip = 1 / length;
  else
    lengthRecip = std::numeric_limits<Real>::infinity();
}

PV2D ParametricFunctionCubic2D::getPointParametric(const Real &t) const
{
  return PV2D(xA * t * t * t + xB * t * t + xC * t + xD, yA * t * t * t + yB * t * t + yC * t + yD);
}

PV2D ParametricFunctionCubic2D::getPointAbsolute(const Real &distance) const
{
  const Real t = distance * lengthRecip;
  return PV2D(xA * t * t * t + xB * t * t + xC * t + xD, yA * t * t * t + yB * t * t + yC * t + yD);
}

PV2D ParametricFunctionCubic2D::getDirectionParametric(const Real &t) const
{
  return PV2D(3 * xA * t * t + 2 * xB * t + xC, 3 * yA * t * t + 2 * yB * t + yC);
}

PV2D ParametricFunctionCubic2D::getDirectionAbsolute(const Real &distance) const
{
  const Real t = distance * lengthRecip;
  return PV2D(3 * xA * t * t + 2 * xB * t + xC, 3 * yA * t * t + 2 * yB * t + yC);
}

Real ParametricFunctionCubic2D::getAngleAbsolute(const Real &distance) const
{
  const PV2D direction = getDirectionAbsolute(distance);
  return atan2(direction.C1, direction.C0);
}
