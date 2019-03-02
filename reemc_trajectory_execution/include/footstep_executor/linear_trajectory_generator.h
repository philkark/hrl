#ifndef REEMC_TRAJECTORY_EXECUTION_LINEAR_TRAJECTORY_GENERATOR_H_
#define REEMC_TRAJECTORY_EXECUTION_LINEAR_TRAJECTORY_GENERATOR_H_

//personal
#include <math_std/math_std.h>

//local
#include "footstep_executor/trajectory_generator.h"

class LinearTrajectoryGenerator : public TrajectoryGenerator
{
public:
  void findPoses(std::vector <Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const override;
};

#endif //REEMC_TRAJECTORY_EXECUTION_LINEAR_TRAJECTORY_GENERATOR_H_
