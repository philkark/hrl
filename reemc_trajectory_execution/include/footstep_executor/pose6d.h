#ifndef REEMC_TRAJECTORY_EXECUTION_POSE6D_H_
#define REEMC_TRAJECTORY_EXECUTION_POSE6D_H_

//personal
#include <math_std/math_std.h>

struct Pose6D
{
  Real x;
  Real y;
  Real z;
  Real roll;
  Real pitch;
  Real yaw;

  Pose6D();

  Pose6D(const Real value);

  Pose6D(const Real x, const Real y, const Real z, const Real roll, const Real pitch, const Real yaw);

  void setZero();

  Pose6D displace(const Pose6D &displacement) const;

  Real distance3D(const Pose6D &pose) const;

  Real distance2D(const Pose6D &pose) const;
};

#endif // REEMC_TRAJECTORY_EXECUTION_POSE6D_H_
