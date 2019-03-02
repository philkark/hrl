#include<math_std/line.h>
#include<math_std/pv.h>

//############# Line2D #############

Line2D::Line2D()
{
}
Line2D::Line2D(const PV2D &start, const PV2D &direction)
{
  this->start = start;
  setDirection(direction);
}
std::ostream& operator<<(std::ostream &out, Line2D &line)
{
  out << "Pos: " << line.start;
  out << "Dir: " << line.direction;
  return out;
}

void Line2D::setDirection(const PV2D &pv1, const PV2D &pv2)
{
  this->direction = pv2 - pv1;
  this->direction.normalize();
}
void Line2D::setDirection(const PV2D &directionNew)
{
  this->direction = directionNew;
  this->direction.normalize();
}

//#############Line3D #############

Line3D::Line3D()
{
}
Line3D::Line3D(const PV3D &start, const PV3D &direction)
{
  this->start = start;
  setDirection(direction);
}
std::ostream& operator<<(std::ostream &out, Line3D &line)
{
  out << "Pos: " << line.start;
  out << "Dir: " << line.direction;
  return out;
}

void Line3D::setDirection(const PV3D &pv1, const PV3D &pv2)
{
  this->direction = pv2 - pv1;
  this->direction.normalize();
}
void Line3D::setDirection(const PV3D &directionNew)
{
  this->direction = directionNew;
  this->direction.normalize();
}

//############# LineSegment2D #############

LineSegment2D::LineSegment2D()
{
  setPoints(PV2D(0, 0), PV2D(0, 0));
}
LineSegment2D::LineSegment2D(const PV2D &startInit, const PV2D &endInit)
{
  setPoints(startInit, endInit);
}
bool LineSegment2D::operator==(const LineSegment2D &line) const
{
  return start == line.start && end == line.end;

}
std::ostream& operator<<(std::ostream &out, LineSegment2D &line)
{
  out << "Start:   " << line.start;
  out << "End:     " << line.end;
  out << "Dir:     " << line.direction;
  out << "DirFull: " << line.directionFull;
  return out;
}

void LineSegment2D::setPoints(const PV2D &startNew, const PV2D &endNew)
{
  start = startNew;
  end = endNew;
  directionFull = direction = end - start;
  direction.normalize();
  length = directionFull.norm();
}
PV2D LineSegment2D::fromStartAbs(Real distance) const
{
  return start + direction * distance;
}
PV2D LineSegment2D::fromStartRel(Real percent) const
{
  return start +  direction* length * percent;
}
PV2D LineSegment2D::fromEndAbs(Real distance) const
{
  return end - direction * distance;
}
PV2D LineSegment2D::fromEndRel(Real percent) const
{
  return end -  direction* length * percent;
}
Line2D LineSegment2D::toLine() const
{
  return Line2D(start, direction);
}

//############# LineSegment3D #############

LineSegment3D::LineSegment3D()
{
  setPoints(PV3D(0, 0, 0), PV3D(0, 0, 0));
}
LineSegment3D::LineSegment3D(const PV3D &startInit, const PV3D &endInit)
{
  setPoints(startInit, endInit);
}
std::ostream& operator<<(std::ostream &out, LineSegment3D &line)
{
  out << "Start: " << line.start;
  out << "End:   " << line.end;
  return out;
}

void LineSegment3D::setPoints(const PV3D &startNew, const PV3D &endNew)
{
  start = startNew;
  end = endNew;
  directionFull = direction = end - start;
  direction.normalize();
  length = directionFull.norm();
}
PV3D LineSegment3D::fromStartAbs(Real distance) const
{
  return start + direction * distance;
}
PV3D LineSegment3D::fromStartRel(Real percent) const
{
  return start +  direction* length * percent;
}
PV3D LineSegment3D::fromEndAbs(Real distance) const
{
  return end - direction * distance;
}
PV3D LineSegment3D::fromEndRel(Real percent) const
{
  return end - direction* length * percent;
}
Line3D LineSegment3D::toLine() const
{
  return Line3D(start, direction);
}
