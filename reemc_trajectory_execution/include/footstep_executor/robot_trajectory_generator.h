#ifndef REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_TRAJECTORY_FINDER_H_
#define REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_TRAJECTORY_FINDER_H_

//std
#include <string>
#include <vector>

//personal
#include <math_std/math_std.h>

//ros
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

//kdl
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

//local
#include "footstep_executor/ellipse_trajectory_generator.h"
#include "footstep_executor/rectangular_trajectory_generator.h"
#include "footstep_executor/linear_trajectory_generator.h"
#include "footstep_executor/pose6d.h"

class RobotTrajectoryGenerator
{
public:
  enum Foot
  {
    left, right
  };

  RobotTrajectoryGenerator(const std::string &baseFrame, const std::string &leftFootFrame, const std::string &rightFootFrame);

  void setInitialJointStates(const std::vector<Real> &jointStates);

  bool addTrajectoryBaseAboveFoot(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot, const Real height, const Real angle, const Real speed);

  bool addTrajectoryBaseBetweenFeet(control_msgs::FollowJointTrajectoryGoal &goal, const Real height, const Real speed);

  bool addTrajectoryFootstepRelativePose(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot, const Real stepHeight, const Pose6D &pose, const Real timeFrame);

  bool addTrajectoryFootstepRectangularRelativePose(control_msgs::FollowJointTrajectoryGoal &goal, const Foot foot, const Real stepHeight, const Pose6D &pose, const Real timeFrame);


private:
  std::string baseFrame;
  std::string leftFootFrame;
  std::string rightFootFrame;
  std::vector<std::string> jointNames;
  std::vector<Real> motionSplits;

  Pose6D initialBasePose;
  Pose6D initialLeftFootPose;
  Pose6D initialRightFootPose;

  KDL::ChainFkSolverPos_recursive* fkSolverPosLeft;
  KDL::ChainFkSolverPos_recursive* fkSolverPosRight;
  KDL::ChainIkSolverVel_wdls* ikSolverVelLeft;
  KDL::ChainIkSolverVel_wdls* ikSolverVelRight;
  KDL::ChainIkSolverPos_NR_JL* ikSolverPosJLLeft;
  KDL::ChainIkSolverPos_NR_JL* ikSolverPosJLRight;

  KDL::JntArray initialJointStatesLeft;
  KDL::JntArray initialJointStatesRight;

  EllipseTrajectoryGenerator ellipseTrajectoryGenerator;
  RectangularTrajectoryGenerator rectangularTrajectoryGenerator;
  LinearTrajectoryGenerator linearTrajectoryGenerator;

  bool initialize();

  void initializeJointNames();

  bool initializeKDLSolvers();

  void initializeMotionSplits();

  void setInitialPosesLeftSupport();

  void setInitialPosesRightSupport();

  bool findJointStates(const Pose6D &rootPose, const Pose6D &tipPose, KDL::ChainIkSolverPos_NR_JL* solver, const KDL::JntArray &jointStatesInitial,
                       KDL::JntArray &jointStates);

  //HELPERS

  void splitVector(std::vector<Real> &firstTargetPositions, std::vector<Real> &secondTargetPositions, const std::vector<Real> &positions);

  void splitVector(KDL::JntArray &firstTargetPositions, KDL::JntArray &secondTargetPositions, const std::vector<Real> &positions);

  void combineVector(const std::vector<Real> &firstPositions, const std::vector<Real> &secondPositions, std::vector<Real> &targetPositions);

  void combineVector(const KDL::JntArray &firstPositions, const KDL::JntArray &secondPositions, std::vector<Real> &targetPositions);
};

#endif // REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_TRAJECTORY_FINDER_H_
