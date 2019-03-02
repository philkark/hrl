#ifndef REEMC_TRAJECTORY_EXECUTION_TRAJECTORY_GENERATOR_H_
#define REEMC_TRAJECTORY_EXECUTION_TRAJECTORY_GENERATOR_H_

//peronsal
#include <math_std/math_std.h>

//local
#include "footstep_executor/pose6d.h"

class TrajectoryGenerator
{
public:
  virtual void findPoses(std::vector <Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const = 0;

  void setPoses(const Pose6D &initPose, const Pose6D &goalPose);

protected:
  Pose6D initPose;
  Pose6D goalPose;

  TrajectoryGenerator();

  void adjustAngle(Real &angle) const;
};

#endif //REEMC_TRAJECTORY_EXECUTION_TRAJECTORY_GENERATOR_H_
