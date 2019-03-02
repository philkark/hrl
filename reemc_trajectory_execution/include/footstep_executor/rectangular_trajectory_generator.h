#ifndef REEMC_TRAJECTORY_EXECUTION_RECTANGULAR_TRAJECTORY_GENERATOR_H_
#define REEMC_TRAJECTORY_EXECUTION_RECTANGULAR_TRAJECTORY_GENERATOR_H_

//personal
#include <math_std/math_std.h>

//local
#include "footstep_executor/trajectory_generator.h"

class RectangularTrajectoryGenerator : public TrajectoryGenerator
{
public:
  RectangularTrajectoryGenerator();

  void findPoses(std::vector <Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const override;

  void setHeight(const Real height);

private:
  Real height;

};

#endif //REEMC_TRAJECTORY_EXECUTION_RECTANGULAR_TRAJECTORY_GENERATOR_H_
