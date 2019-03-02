//std
#include "footstep_executor/robot_trajectory_generator.h"

#include <iostream>
#include <fstream>
#include <sstream>

//personal
#include <tools_std/ros_param_loader.h>

//local

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC MEMBERS //////////////////////////////

//////////////////////////////////////////////////////////////////

RobotTrajectoryGenerator::RobotTrajectoryGenerator(const std::string &baseFrame, const std::string &leftFootFrame, const std::string &rightFootFrame) :
    baseFrame(baseFrame), leftFootFrame(leftFootFrame), rightFootFrame(rightFootFrame)
{
  if (!initialize())
    ros::shutdown();
}

void RobotTrajectoryGenerator::setInitialJointStates(const std::vector<Real> &jointStates)
{
  if (jointStates.size() != 14)
  {
    std::cerr << "Cannot set current joint states. Size must be 14 but is " << jointStates.size() << std::endl;
    return;
  }

  splitVector(initialJointStatesLeft, initialJointStatesRight, jointStates);
}

bool RobotTrajectoryGenerator::addTrajectoryBaseAboveFoot(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot, const Real height, const Real angle,
                                                          const Real speed)
{
  if (foot == Foot::left)
    setInitialPosesLeftSupport();
  else if (foot == Foot::right)
    setInitialPosesRightSupport();

  const Pose6D goalPose(0.0, 0.0, height, 0.0, 0.0, angle);
  const Real timeFrame = initialBasePose.distance3D(goalPose) / speed;
  if (timeFrame < 0.01)
  {
    goal.trajectory.joint_names = jointNames;
    goal.trajectory.points.resize(goal.trajectory.points.size() + 1);
    goal.trajectory.points.back().time_from_start.fromSec(timeFrame);
    combineVector(initialJointStatesLeft, initialJointStatesRight, goal.trajectory.points.back().positions);
    return true;
  }
  const UInt poseCount = motionSplits.size();
  const Real timeStep = timeFrame / poseCount;
  std::vector<Pose6D> poses;
  std::vector<Real> distances;

  linearTrajectoryGenerator.setPoses(initialBasePose, goalPose);
  linearTrajectoryGenerator.findPoses(poses, distances, poseCount);

  control_msgs::FollowJointTrajectoryGoal::_trajectory_type &trajectory = goal.trajectory;

  Real startTime = 0.0;
  UInt startIndex = 0;
  if (trajectory.points.size() > 0)
  {
    startTime = trajectory.points.back().time_from_start.toSec();
    startIndex = trajectory.points.size();
  }
  else
    trajectory.joint_names = jointNames;

  trajectory.points.resize(startIndex + poseCount);

  KDL::JntArray jointStatesInitialLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesInitialRight = initialJointStatesRight;
  KDL::JntArray jointStatesLeft(7);
  KDL::JntArray jointStatesRight(7);
  Real currentTime = timeStep + startTime;
  for (UInt i = 0; i < poseCount; ++i)
  {
    trajectory.points[i + startIndex].time_from_start.fromSec(currentTime);
    trajectory.points[i + startIndex].positions.resize(14);

    if (i > 0)
    {
      for (UInt j = 0; j < 7; ++j)
      {
        jointStatesInitialLeft.data[j] = trajectory.points[i - 1 + startIndex].positions[j];
        jointStatesInitialRight.data[j] = trajectory.points[i - 1 + startIndex].positions[j + 7];
      }
    }

    if (!findJointStates(initialLeftFootPose, poses[i], ikSolverPosJLLeft, jointStatesInitialLeft, jointStatesLeft))
      return false;

    if (!findJointStates(initialRightFootPose, poses[i], ikSolverPosJLRight, jointStatesInitialRight, jointStatesRight))
      return false;

    combineVector(jointStatesLeft, jointStatesRight, trajectory.points[i + startIndex].positions);
    currentTime += timeStep;
  }

  return true;
}

bool RobotTrajectoryGenerator::addTrajectoryBaseBetweenFeet(control_msgs::FollowJointTrajectoryGoal &goal, const Real height, const Real speed)
{
  setInitialPosesLeftSupport();

  const Pose6D goalPose(0.5 * initialRightFootPose.x, 0.5 * initialRightFootPose.y, height, 0.0, 0.0, 0.5 * initialRightFootPose.yaw);
  const Real timeFrame = initialBasePose.distance3D(goalPose) / speed;
  if (timeFrame < 0.01)
    return true;
  const UInt poseCount = motionSplits.size();
  const Real timeStep = timeFrame / poseCount;
  std::vector<Pose6D> poses;
  std::vector<Real> distances;

  linearTrajectoryGenerator.setPoses(initialBasePose, goalPose);
  linearTrajectoryGenerator.findPoses(poses, distances, poseCount);

  control_msgs::FollowJointTrajectoryGoal::_trajectory_type &trajectory = goal.trajectory;

  Real startTime = 0.0;
  UInt startIndex = 0;
  if (trajectory.points.size() > 0)
  {
    startTime = trajectory.points.back().time_from_start.toSec();
    startIndex = trajectory.points.size();
  }
  else
    trajectory.joint_names = jointNames;

  trajectory.points.resize(startIndex + poseCount);

  KDL::JntArray jointStatesInitialLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesInitialRight = initialJointStatesRight;
  KDL::JntArray jointStatesLeft(7);
  KDL::JntArray jointStatesRight(7);
  Real currentTime = timeStep + startTime;
  for (UInt i = 0; i < poseCount; ++i)
  {
    trajectory.points[i + startIndex].time_from_start.fromSec(currentTime);
    trajectory.points[i + startIndex].positions.resize(14);

    if (i > 0)
    {
      for (UInt j = 0; j < 7; ++j)
      {
        jointStatesInitialLeft.data[j] = trajectory.points[i - 1 + startIndex].positions[j];
        jointStatesInitialRight.data[j] = trajectory.points[i - 1 + startIndex].positions[j + 7];
      }
    }

    if (!findJointStates(initialLeftFootPose, poses[i], ikSolverPosJLLeft, jointStatesInitialLeft, jointStatesLeft))
      return false;

    if (!findJointStates(initialRightFootPose, poses[i], ikSolverPosJLRight, jointStatesInitialRight, jointStatesRight))
      return false;

    combineVector(jointStatesLeft, jointStatesRight, trajectory.points[i + startIndex].positions);
    currentTime += timeStep;
  }

  return true;
}

bool RobotTrajectoryGenerator::addTrajectoryFootstepRelativePose(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot, const Real stepHeight,
                                                                 const Pose6D &pose, const Real timeFrame)
{
  if (foot == Foot::left)
  {
    setInitialPosesRightSupport();
    ellipseTrajectoryGenerator.setPoses(initialLeftFootPose, pose);
  }
  else if (foot == Foot::right)
  {
    setInitialPosesLeftSupport();
    ellipseTrajectoryGenerator.setPoses(initialRightFootPose, pose);
  }
  ellipseTrajectoryGenerator.setHeight(stepHeight);

  const UInt poseCount = motionSplits.size();
  const Real timeStep = timeFrame / poseCount;
  std::vector<Pose6D> poses;
  std::vector<Real> distances;
  ellipseTrajectoryGenerator.findPoses(poses, distances, poseCount);

  control_msgs::FollowJointTrajectoryGoal::_trajectory_type &trajectory = goal.trajectory;
  Real startTime = 0.0;
  UInt startIndex = 0;
  if (trajectory.points.size() > 0)
  {
    startTime = trajectory.points.back().time_from_start.toSec();
    startIndex = trajectory.points.size();
  }
  else
    trajectory.joint_names = jointNames;

  trajectory.points.resize(startIndex + poseCount);

  KDL::JntArray jointStatesInitialLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesInitialRight = initialJointStatesRight;
  KDL::JntArray jointStatesLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesRight = initialJointStatesRight;

  for (UInt i = 0; i < poseCount; ++i)
  {
    trajectory.points[i + startIndex].positions.resize(14);

    if (i > 0)
    {
      for (UInt j = 0; j < 7; ++j)
      {
        jointStatesInitialLeft.data[j] = trajectory.points[i - 1 + startIndex].positions[j];
        jointStatesInitialRight.data[j] = trajectory.points[i - 1 + startIndex].positions[j + 7];
      }
    }

    if (foot == Foot::left)
    {
      if (!findJointStates(poses[i], initialBasePose, ikSolverPosJLLeft, jointStatesInitialLeft, jointStatesLeft))
      {
        std::cout << "Fail at iteration " << i << std::endl;
        return false;
      }
    }
    else if (foot == Foot::right)
    {
      if (!findJointStates(poses[i], initialBasePose, ikSolverPosJLRight, jointStatesInitialRight, jointStatesRight))
      {
        std::cout << "Fail at iteration " << i << std::endl;
        return false;
      }
    }

    combineVector(jointStatesLeft, jointStatesRight, trajectory.points[i + startIndex].positions);
  }

  Real totalDistance = 0.0;
  Real correctionFactor = 0.0;
  for (UInt i = 0; i < poseCount; ++i)
    totalDistance += distances[i];
  for (UInt i = 0; i < poseCount; ++i)
  {
    correctionFactor += timeFrame * motionSplits[i] * distances[i] / totalDistance;
  }
  correctionFactor = timeFrame / correctionFactor;

  for (UInt i = 0; i < poseCount; ++i)
  {
    if (i > 0)
    {
      trajectory.points[i + startIndex].time_from_start.fromSec(
          correctionFactor * timeFrame * motionSplits[i] * distances[i] / totalDistance + trajectory.points[i - 1 + startIndex].time_from_start.toSec());
    }
    else
      trajectory.points[i + startIndex].time_from_start.fromSec(startTime + correctionFactor * timeFrame * motionSplits[i] * distances[i] / totalDistance);
  }

  return true;
}

bool RobotTrajectoryGenerator::addTrajectoryFootstepRectangularRelativePose(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot,
                                                                            const Real stepHeight, const Pose6D &pose, const Real timeFrame)
{
  if (foot == Foot::left)
  {
    setInitialPosesRightSupport();
    rectangularTrajectoryGenerator.setPoses(initialLeftFootPose, pose);
  }
  else if (foot == Foot::right)
  {
    setInitialPosesLeftSupport();
    rectangularTrajectoryGenerator.setPoses(initialRightFootPose, pose);
  }
  rectangularTrajectoryGenerator.setHeight(stepHeight);

  const UInt poseCount = motionSplits.size();
  const Real timeStep = timeFrame / poseCount;
  std::vector<Pose6D> poses;
  std::vector<Real> distances;
  rectangularTrajectoryGenerator.findPoses(poses, distances, poseCount);

  control_msgs::FollowJointTrajectoryGoal::_trajectory_type &trajectory = goal.trajectory;
  Real startTime = 0.0;
  UInt startIndex = 0;
  if (trajectory.points.size() > 0)
  {
    startTime = trajectory.points.back().time_from_start.toSec();
    startIndex = trajectory.points.size();
  }
  else
    trajectory.joint_names = jointNames;

  trajectory.points.resize(startIndex + poseCount);

  KDL::JntArray jointStatesInitialLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesInitialRight = initialJointStatesRight;
  KDL::JntArray jointStatesLeft = initialJointStatesLeft;
  KDL::JntArray jointStatesRight = initialJointStatesRight;

  for (UInt i = 0; i < poseCount; ++i)
  {
    trajectory.points[i + startIndex].positions.resize(14);

    if (i > 0)
    {
      for (UInt j = 0; j < 7; ++j)
      {
        jointStatesInitialLeft.data[j] = trajectory.points[i - 1 + startIndex].positions[j];
        jointStatesInitialRight.data[j] = trajectory.points[i - 1 + startIndex].positions[j + 7];
      }
    }

    if (foot == Foot::left)
    {
      if (!findJointStates(poses[i], initialBasePose, ikSolverPosJLLeft, jointStatesInitialLeft, jointStatesLeft))
      {
        std::cout << "Fail at iteration " << i << std::endl;
        return false;
      }
    }
    else if (foot == Foot::right)
    {
      if (!findJointStates(poses[i], initialBasePose, ikSolverPosJLRight, jointStatesInitialRight, jointStatesRight))
      {
        std::cout << "Fail at iteration " << i << std::endl;
        return false;
      }
    }

    combineVector(jointStatesLeft, jointStatesRight, trajectory.points[i + startIndex].positions);
  }

  Real totalDistance = 0.0;
  Real correctionFactor = 0.0;
  for (UInt i = 0; i < poseCount; ++i)
    totalDistance += distances[i];
  for (UInt i = 0; i < poseCount; ++i)
  {
    correctionFactor += timeFrame * motionSplits[i] * distances[i] / totalDistance;
  }
  correctionFactor = timeFrame / correctionFactor;

  for (UInt i = 0; i < poseCount; ++i)
  {
    if (i > 0)
    {
      trajectory.points[i + startIndex].time_from_start.fromSec(
          correctionFactor * timeFrame * motionSplits[i] * distances[i] / totalDistance + trajectory.points[i - 1 + startIndex].time_from_start.toSec());
    }
    else
      trajectory.points[i + startIndex].time_from_start.fromSec(startTime + correctionFactor * timeFrame * motionSplits[i] * distances[i] / totalDistance);
  }

  return true;
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE MEMBERS /////////////////////////////

//////////////////////////////////////////////////////////////////

bool RobotTrajectoryGenerator::initialize()
{
  initializeJointNames();
  initializeMotionSplits();
  initialJointStatesLeft.resize(7);
  initialJointStatesRight.resize(7);

  if (!initializeKDLSolvers())
    return false;

  return true;
}

void RobotTrajectoryGenerator::initializeJointNames()
{
  jointNames.reserve(14);
  jointNames.push_back("leg_left_sole_joint");
  jointNames.push_back("leg_left_6_joint");
  jointNames.push_back("leg_left_5_joint");
  jointNames.push_back("leg_left_4_joint");
  jointNames.push_back("leg_left_3_joint");
  jointNames.push_back("leg_left_2_joint");
  jointNames.push_back("leg_left_1_joint");
  jointNames.push_back("leg_right_sole_joint");
  jointNames.push_back("leg_right_6_joint");
  jointNames.push_back("leg_right_5_joint");
  jointNames.push_back("leg_right_4_joint");
  jointNames.push_back("leg_right_3_joint");
  jointNames.push_back("leg_right_2_joint");
  jointNames.push_back("leg_right_1_joint");
}

bool RobotTrajectoryGenerator::initializeKDLSolvers()
{
  KDL::Tree kdlTree;
  KDL::Chain chainRight, chainLeft;
  KDL::JntArray jointArrayMinLeft;
  KDL::JntArray jointArrayMaxLeft;
  KDL::JntArray jointArrayMinRight;
  KDL::JntArray jointArrayMaxRight;

  if (!kdl_parser::treeFromParam("/robot_description", kdlTree))
  {
    std::cerr << "Could not parse urdf." << std::endl;
    return false;
  }

  if (!kdlTree.getChain(leftFootFrame, baseFrame, chainLeft))
  {
    std::cerr << "Could not find chain between " << leftFootFrame << " and " << baseFrame << std::endl;
    return false;
  }
  if (!kdlTree.getChain(rightFootFrame, baseFrame, chainRight))
  {
    std::cerr << "Could not find chain between " << rightFootFrame << " and " << baseFrame << std::endl;
    return false;
  }

  jointArrayMinLeft.resize(7);
  jointArrayMaxLeft.resize(7);
  jointArrayMinRight.resize(7);
  jointArrayMaxRight.resize(7);

  jointArrayMinLeft.data[0] = -0.00001;
  jointArrayMaxLeft.data[0] = 0.00001;
  jointArrayMinLeft.data[1] = -0.523598775598;
  jointArrayMaxLeft.data[1] = 0.261799387799;
  jointArrayMinLeft.data[2] = -1.308996939;
  jointArrayMaxLeft.data[2] = 0.785398163397;
  jointArrayMinLeft.data[3] = 0.0;
  jointArrayMaxLeft.data[3] = 2.61799387799;
  jointArrayMinLeft.data[4] = -1.74532925199;
  jointArrayMaxLeft.data[4] = 0.785398163397;
  jointArrayMinLeft.data[5] = -0.261799387799;
  jointArrayMaxLeft.data[5] = 0.523598775598;
  jointArrayMinLeft.data[6] = -0.785398163397;
  jointArrayMaxLeft.data[6] = 0.523598775598;

  jointArrayMinRight.data[0] = -0.00001;
  jointArrayMinRight.data[0] = 0.00001;
  jointArrayMinRight.data[1] = -0.261799387799;
  jointArrayMaxRight.data[1] = 0.523598775598;
  jointArrayMinRight.data[2] = -1.308996939;
  jointArrayMaxRight.data[2] = 0.785398163397;
  jointArrayMinRight.data[3] = 0.0;
  jointArrayMaxRight.data[3] = 2.61799387799;
  jointArrayMinRight.data[4] = -1.74532925199;
  jointArrayMaxRight.data[4] = 0.785398163397;
  jointArrayMinRight.data[5] = -0.523598775598;
  jointArrayMaxRight.data[5] = 0.261799387799;
  jointArrayMinRight.data[6] = -0.523598775598;
  jointArrayMaxRight.data[6] = 0.785398163397;

  fkSolverPosLeft = new KDL::ChainFkSolverPos_recursive(chainLeft);
  fkSolverPosRight = new KDL::ChainFkSolverPos_recursive(chainRight);
  ikSolverVelLeft = new KDL::ChainIkSolverVel_wdls(chainLeft, 1e-5, 500);
  ikSolverVelRight = new KDL::ChainIkSolverVel_wdls(chainRight, 1e-5, 500);

  ikSolverPosJLLeft = new KDL::ChainIkSolverPos_NR_JL(chainLeft, jointArrayMinLeft, jointArrayMaxLeft, *fkSolverPosLeft, *ikSolverVelLeft, 500, 1e-5);
  ikSolverPosJLRight = new KDL::ChainIkSolverPos_NR_JL(chainRight, jointArrayMinRight, jointArrayMaxRight, *fkSolverPosRight, *ikSolverVelRight, 500, 1e-5);

  return true;
}

void RobotTrajectoryGenerator::initializeMotionSplits()
{
  motionSplits.resize(24);
  motionSplits[0] = 12;
  motionSplits[1] = 8;
  motionSplits[2] = 6;
  motionSplits[3] = 2;
  motionSplits[4] = 2;
  motionSplits[5] = 2;
  motionSplits[6] = 2;
  motionSplits[7] = 2;
  motionSplits[8] = 2;
  motionSplits[9] = 2;
  motionSplits[10] = 2;
  motionSplits[11] = 2;
  motionSplits[12] = 2;
  motionSplits[13] = 2;
  motionSplits[14] = 2;
  motionSplits[15] = 2;
  motionSplits[16] = 2;
  motionSplits[17] = 2;
  motionSplits[18] = 2;
  motionSplits[19] = 2;
  motionSplits[20] = 2;
  motionSplits[21] = 6;
  motionSplits[22] = 8;
  motionSplits[23] = 12;

  Real sum = 0.0;
  for (UInt i = 0; i < motionSplits.size(); ++i)
    sum += motionSplits[i];
  for (UInt i = 0; i < motionSplits.size(); ++i)
    motionSplits[i] /= sum;
}

void RobotTrajectoryGenerator::setInitialPosesLeftSupport()
{
  initialLeftFootPose.setZero();

  KDL::Frame frameLeft;
  fkSolverPosLeft->JntToCart(initialJointStatesLeft, frameLeft);
  frameLeft.M.GetRPY(initialBasePose.roll, initialBasePose.pitch, initialBasePose.yaw);
  initialBasePose.x = frameLeft.p[0];
  initialBasePose.y = frameLeft.p[1];
  initialBasePose.z = frameLeft.p[2];

  KDL::Frame frameRight;
  fkSolverPosRight->JntToCart(initialJointStatesRight, frameRight);
  frameRight = frameLeft * frameRight.Inverse();
  frameRight.M.GetRPY(initialRightFootPose.roll, initialRightFootPose.pitch, initialRightFootPose.yaw);
  initialRightFootPose.x = frameRight.p[0];
  initialRightFootPose.y = frameRight.p[1];
  initialRightFootPose.z = frameRight.p[2];
}

void RobotTrajectoryGenerator::setInitialPosesRightSupport()
{
  initialRightFootPose.setZero();

  KDL::Frame frameRight;
  fkSolverPosRight->JntToCart(initialJointStatesRight, frameRight);
  frameRight.M.GetRPY(initialBasePose.roll, initialBasePose.pitch, initialBasePose.yaw);
  initialBasePose.x = frameRight.p[0];
  initialBasePose.y = frameRight.p[1];
  initialBasePose.z = frameRight.p[2];

  KDL::Frame frameLeft;
  fkSolverPosLeft->JntToCart(initialJointStatesLeft, frameLeft);
  frameLeft = frameRight * frameLeft.Inverse();
  frameLeft.M.GetRPY(initialLeftFootPose.roll, initialLeftFootPose.pitch, initialLeftFootPose.yaw);
  initialLeftFootPose.x = frameLeft.p[0];
  initialLeftFootPose.y = frameLeft.p[1];
  initialLeftFootPose.z = frameLeft.p[2];
}

bool RobotTrajectoryGenerator::findJointStates(const Pose6D &rootPose, const Pose6D &tipPose, KDL::ChainIkSolverPos_NR_JL* solver,
                                               const KDL::JntArray &jointStatesInitial, KDL::JntArray &jointStates)
{
  KDL::Frame frame;

  Real cosA, sinA;
  sincos(rootPose.yaw, &sinA, &cosA);
  const Real xDiff = -rootPose.x + tipPose.x;
  const Real yDiff = -rootPose.y + tipPose.y;

  frame.p.data[0] = cosA * xDiff + sinA * yDiff;
  frame.p.data[1] = -sinA * xDiff + cosA * yDiff;
  frame.p.data[2] = tipPose.z - rootPose.z;
  frame.M = KDL::Rotation::RPY(0, 0, tipPose.yaw - rootPose.yaw);

  Int errorCode = solver->CartToJnt(jointStatesInitial, frame, jointStates);

  if (errorCode == KDL::ChainFkSolverPos_recursive::E_NOERROR || errorCode == KDL::ChainIkSolverVel_wdls::E_CONVERGE_PINV_SINGULAR)
    return true;
  else
    return false;
}

void RobotTrajectoryGenerator::splitVector(std::vector<Real> &firstTargetPositions, std::vector<Real> &secondTargetPositions,
                                           const std::vector<Real> &positions)
{
  UInt index = 0;
  const UInt size1 = firstTargetPositions.size();
  const UInt size2 = secondTargetPositions.size();
  for (UInt i = 0; i < size1; ++i, ++index)
    firstTargetPositions[i] = positions[index];
  for (UInt i = 0; i < size2; ++i, ++index)
    secondTargetPositions[i] = positions[index];
}

void RobotTrajectoryGenerator::splitVector(KDL::JntArray &firstTargetPositions, KDL::JntArray &secondTargetPositions, const std::vector<Real> &positions)
{
  UInt index = 0;
  const UInt size1 = firstTargetPositions.data.size();
  const UInt size2 = secondTargetPositions.data.size();
  for (UInt i = 0; i < size1; ++i, ++index)
    firstTargetPositions.data[i] = positions[index];
  for (UInt i = 0; i < size2; ++i, ++index)
    secondTargetPositions.data[i] = positions[index];
}

void RobotTrajectoryGenerator::combineVector(const std::vector<Real> &firstPositions, const std::vector<Real> &secondPositions,
                                             std::vector<Real> &targetPositions)
{
  UInt index = 0;
  const UInt size1 = firstPositions.size();
  const UInt size2 = secondPositions.size();
  for (UInt i = 0; i < size1; ++i, ++index)
    targetPositions[index] = firstPositions[i];
  for (UInt i = 0; i < size2; ++i, ++index)
    targetPositions[index] = secondPositions[i];
}

void RobotTrajectoryGenerator::combineVector(const KDL::JntArray &firstPositions, const KDL::JntArray &secondPositions, std::vector<Real> &targetPositions)
{
  UInt index = 0;
  const UInt size1 = firstPositions.data.size();
  const UInt size2 = secondPositions.data.size();
  for (UInt i = 0; i < size1; ++i, ++index)
    targetPositions[index] = firstPositions.data[i];
  for (UInt i = 0; i < size2; ++i, ++index)
    targetPositions[index] = secondPositions.data[i];
}
