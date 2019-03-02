//local
#include "footstep_executor/linear_trajectory_generator.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC MEMBERS //////////////////////////////

//////////////////////////////////////////////////////////////////

void LinearTrajectoryGenerator::findPoses(std::vector<Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const
{
  if (poseCount < 1)
    poseCount = 1;

  poses.reserve(poseCount);
  distances.reserve(poseCount);

  const Real distance = goalPose.distance3D(initPose) / poseCount;
  const Real directionX = (goalPose.x - initPose.x) / poseCount;
  const Real directionY = (goalPose.y - initPose.y) / poseCount;
  const Real directionZ = (goalPose.z - initPose.z) / poseCount;
  Real directionYaw = goalPose.yaw - initPose.yaw;
  adjustAngle(directionYaw);
  directionYaw /= poseCount;

  Pose6D poseTmp = initPose;
  poseTmp.pitch = poseTmp.roll = 0.0;
  for (UInt i = 0; i < poseCount - 1; ++i)
  {
    poseTmp.x += directionX;
    poseTmp.y += directionY;
    poseTmp.z += directionZ;
    poseTmp.yaw += directionYaw;
    adjustAngle(poseTmp.yaw);
    if (i > 0)
      distances.push_back(distances.back() + distance);
    else
      distances.push_back(distance);
    poses.push_back(poseTmp);
  }
  distances.push_back(distances.back() + distance);
  poses.push_back(goalPose);
}
