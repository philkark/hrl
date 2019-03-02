//local
#include "footstep_executor/pose6d.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC MEMBERS //////////////////////////////

//////////////////////////////////////////////////////////////////

Pose6D::Pose6D()
{
}

Pose6D::Pose6D(const Real value)
{
  x = y = z = roll = pitch = yaw = value;
}

Pose6D::Pose6D(const Real x, const Real y, const Real z, const Real roll, const Real pitch, const Real yaw) :
    x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
{
}

void Pose6D::setZero()
{
  x = y = z = roll = pitch = yaw = 0.0;
}

Pose6D Pose6D::displace(const Pose6D &displacement) const
{
  Real yawNew = yaw + displacement.yaw;
  if (yawNew > M_PI)
    yawNew -= PITWO;
  else if (yawNew < -M_PI)
    yawNew += PITWO;

  return Pose6D(x + displacement.x, y + displacement.y, z + displacement.z, roll, pitch, yawNew);
}

Real Pose6D::distance3D(const Pose6D &pose) const
{
  return sqrt(pow(x - pose.x, 2) + pow(y - pose.y, 2) + pow(z - pose.z, 2));
}

Real Pose6D::distance2D(const Pose6D &pose) const
{
  return sqrt(pow(x - pose.x, 2) + pow(y - pose.y, 2));
}
