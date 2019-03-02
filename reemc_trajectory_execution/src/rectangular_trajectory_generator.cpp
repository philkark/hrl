//local
#include "footstep_executor/rectangular_trajectory_generator.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC MEMBERS //////////////////////////////

//////////////////////////////////////////////////////////////////

RectangularTrajectoryGenerator::RectangularTrajectoryGenerator() :
    height(0.06)
{
}

void RectangularTrajectoryGenerator::setHeight(const Real height)
{
  this->height = height;
}

void RectangularTrajectoryGenerator::findPoses(std::vector<Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const
{
  if (poseCount < 1)
    poseCount = 1;

  poses.reserve(poseCount);
  distances.reserve(poseCount);

  //find pose count splits
  Real distanceAlong = goalPose.distance2D(initPose);
  Real distanceUp = goalPose.z > initPose.z ? height + goalPose.z - initPose.z : height;
  Real distanceDown = goalPose.z > initPose.z ? height : height + initPose.z - goalPose.z;
  const Real distanceTotal = distanceAlong + distanceUp + distanceDown;

  UInt poseCountUp = (UInt)(poseCount * distanceUp / distanceTotal);
  if (poseCountUp == 0)
    poseCountUp = 1;
  UInt poseCountDown = (UInt)(poseCount * distanceDown / distanceTotal);
  if (poseCountDown == 0)
    poseCountDown = 1;
  const UInt poseCountAlong = poseCount - poseCountUp - poseCountDown;

  //set motion directions
  const Real directionX = (goalPose.x - initPose.x) / poseCountAlong;
  const Real directionY = (goalPose.y - initPose.y) / poseCountAlong;
  const Real directionZUp = distanceUp / poseCountUp;
  const Real directionZDown = -distanceDown / poseCountDown;
  Real directionYaw = goalPose.yaw - initPose.yaw;
  adjustAngle(directionYaw);
  directionYaw /= poseCountAlong;
  distanceAlong /= poseCountAlong;

  Pose6D poseTmp = initPose;

  Real currentDistance = 0.0;
  //add poses up
  for (UInt i = 0; i < poseCountUp; ++i)
  {
    poseTmp.z += directionZUp;
    currentDistance += directionZUp;
    distances.push_back(currentDistance);
    poses.push_back(poseTmp);
  }

  //add poses along
  for (UInt i = 0; i < poseCountAlong; ++i)
  {
    poseTmp.x += directionX;
    poseTmp.y += directionY;
    poseTmp.yaw += directionYaw;
    adjustAngle(poseTmp.yaw);
    currentDistance += distanceAlong;
    distances.push_back(currentDistance);
    poses.push_back(poseTmp);
  }


  //add poses down
  for (UInt i = 0; i < poseCountDown -1; ++i)
  {
    poseTmp.z += directionZDown;
    currentDistance += fabs(directionZDown);
    distances.push_back(currentDistance);
    poses.push_back(poseTmp);
  }
  currentDistance += fabs(directionZDown);
  distances.push_back(currentDistance);
  poses.push_back(goalPose);
}

