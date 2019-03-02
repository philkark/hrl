//local
#include "footstep_executor/ellipse_trajectory_generator.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC MEMBERS //////////////////////////////

//////////////////////////////////////////////////////////////////

EllipseTrajectoryGenerator::EllipseTrajectoryGenerator() :
    height(0.06)
{
}

void EllipseTrajectoryGenerator::setHeight(const Real height)
{
  this->height = height;
}

void EllipseTrajectoryGenerator::findPoses(std::vector<Pose6D> &poses, std::vector<Real> &distances, UInt poseCount) const
{
  if (poseCount < 1)
    poseCount = 1;

  poses.reserve(poseCount);
  distances.reserve(poseCount);

  Real distance = goalPose.distance2D(initPose);

  Real coeffA, coeffB;
  findParabolaCoefficients(distance, goalPose.z - initPose.z, height, coeffA, coeffB);

  const Real directionX = (goalPose.x - initPose.x) / poseCount;
  const Real directionY = (goalPose.y - initPose.y) / poseCount;
  Real directionYaw = goalPose.yaw - initPose.yaw;
  adjustAngle(directionYaw);
  directionYaw /= poseCount;
  distance /= poseCount;

  Pose6D poseTmp = initPose;

  Real positionOnParabola = distance;
  for (UInt i = 0; i < poseCount - 1; ++i)
  {
    poseTmp.x += directionX;
    poseTmp.y += directionY;
    poseTmp.z = initPose.z + coeffA * pow(positionOnParabola, 2) + coeffB * positionOnParabola;
    poseTmp.yaw += directionYaw;
    adjustAngle(poseTmp.yaw);
    if (i > 0)
      distances.push_back(poses.back().distance3D(poseTmp));
    else
      distances.push_back(initPose.distance3D(poseTmp));
    poses.push_back(poseTmp);
    positionOnParabola += distance;
  }
  distances.push_back(poses.back().distance3D(goalPose));
  poses.push_back(goalPose);
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE MEMBERS /////////////////////////////

//////////////////////////////////////////////////////////////////

void EllipseTrajectoryGenerator::findParabolaCoefficients(const Real &distance, const Real &heightEnd, Real stepHeight, Real &coeffA, Real &coeffB) const
{
  if (heightEnd < 0)
    stepHeight = -heightEnd + stepHeight;

  coeffA = (-heightEnd - 2 * stepHeight - 2 * sqrt(pow(stepHeight, 2) + stepHeight * heightEnd)) / pow(distance, 2);
  coeffB = heightEnd / distance - coeffA * distance;
}

