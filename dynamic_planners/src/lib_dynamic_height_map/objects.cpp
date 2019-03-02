#include <dynamic_height_map/objects.h>

// *********************************************************************

// ************************** OBJECT ***********************************

// *********************************************************************

// ************************** PUBLIC ***********************************

Object::Object(const Segment &segment, const ros::Time &timeStamp, const PV3D &robotPose) :
    timeStamp(timeStamp), robotPose(robotPose)
{
  type = ObjectType::None;
  segmentType = segment.type;
  segmentID = segment.ID;
  objectPrevious = nullptr;
  position = segment.position;
  sizeCells = (Real)segment.cellList->size();
  heightAverage = segment.heightAverage;
  heightVariance = segment.heightVariance;
  sincos(robotPose.C2, &sinA, &cosA);
}

PV2D Object::getRelativePosition(const Real time) const
{
  if (type == ObjectType::Static)
    return PV2D();
  else
    return velocity * time;
}

PV2D Object::getAbsolutePosition(const Real time) const
{
  return position + getRelativePosition(time);
}

void Object::updateTrajectory(const Real speedComputationTime)
{
  const UInt speedChainLength = 100;
  std::vector<PV2D> positions;
  std::vector<Real> times;
  getTransformedPositionsAndTimes(positions, times, speedChainLength);

  if (times.size() < 15)
    return;

  type = ObjectType::Dynamic;
  findMovingDirection(positions);

  velocity = PV2D(velocity.C0 * cosA + velocity.C1 * sinA, -velocity.C0 * sinA + velocity.C1 * cosA);
  setSpeed(positions, times);

  if (velocity.normSquared() < 0.0001)
    setObjectStatic();
}

void Object::setObjectStatic()
{
  type = ObjectType::Static;
  velocity.C0 = velocity.C1 = 0;
}

// ************************** PRIVATE **********************************

void Object::getTransformedPositionsAndTimes(std::vector<PV2D> &positions, std::vector<Real> &times, const UInt maxChainLength)
{
  UInt chainLength = 1;
  positions.push_back(PV2D(position.C0 * cosA - position.C1 * sinA + robotPose.C0, position.C0 * sinA + position.C1 * cosA + robotPose.C1));
  times.push_back(timeStamp.toSec());
  Object *objectPrev = objectPrevious;
  while (objectPrev != nullptr && chainLength < maxChainLength)
  {
    positions.push_back(
        PV2D(objectPrev->position.C0 * objectPrev->cosA - objectPrev->position.C1 * objectPrev->sinA + objectPrev->robotPose.C0,
             objectPrev->position.C0 * objectPrev->sinA + objectPrev->position.C1 * objectPrev->cosA + objectPrev->robotPose.C1));
    times.push_back(objectPrev->timeStamp.toSec());
    ++chainLength;
    objectPrev = objectPrev->objectPrevious;
  }
}

void Object::findMovingDirection(const std::vector<PV2D> &positions)
{
  Real chainSizeRecip = 1.0 / positions.size();
  Real meanX = 0;
  Real meanY = 0;
  for (std::vector<PV2D>::const_iterator pos = positions.begin(); pos != positions.end(); ++pos)
  {
    meanX += pos->C0;
    meanY += pos->C1;
  }
  meanX *= chainSizeRecip;
  meanY *= chainSizeRecip;

  Real slope = 0;
  Real slopeDenominator = 0;
  for (std::vector<PV2D>::const_iterator pos = positions.begin(); pos != positions.end(); ++pos)
  {
    slope += (pos->C0 - meanX) * (pos->C1 - meanY);
    slopeDenominator += pow((pos->C0 - meanX), 2);
  }

  if (slopeDenominator < 1e-10)
    velocity = PV2D(0, 1);
  else
  {
    slope /= slopeDenominator;
    velocity = PV2D(1, slope);
    velocity.normalize();
  }

  if (velocity * (positions.front() - positions.back()) < 0.0)
    velocity *= -1;
}

void Object::setSpeed(const std::vector<PV2D> &positions, const std::vector<Real> &times)
{
  velocity *= (positions.front() - positions[14]).norm() / (times.front() - times[14]);
}
