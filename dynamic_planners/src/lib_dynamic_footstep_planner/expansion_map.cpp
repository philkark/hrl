#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>

#include "dynamic_footstep_planner/expansion_map.h"
#include "common/progress_publisher.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

ExpansionMap::ExpansionMap() :
    valid(false)
{
  rightExpansionNodes.reserve(500);
  leftExpansionNodes.reserve(500);
}

bool ExpansionMap::isValid()
{
  return valid;
}

const std::vector<ExpansionNode> &ExpansionMap::getLeftExpansionNodes() const
{
  return leftExpansionNodes;
}

const std::vector<ExpansionNode> &ExpansionMap::getRightExpansionNodes() const
{
  return rightExpansionNodes;
}

const ExpansionConnection &ExpansionMap::getExpansionConnection(const Real distance, const Real height) const
{
  return expansionConnections.get(distance).get(height);
}

void ExpansionMap::initializeExpansionMap(bool validate, Int progressIndex)
{
  this->progressIndex = progressIndex;
  ProgressPublisher::showMessage(progressIndex, "Generating expansion maps...");
  generateExpansionMaps();
  findNextExpansionIndices();
  addZeroAndSideSteps();
  ProgressPublisher::showMessage(progressIndex, "Generating expansion connections...");
  generateExpansionConnections();

  if (validate)
  {
    ProgressPublisher::showMessage(progressIndex, "Validating reachabilities...");
    valid = validateReachabilities();
  }
  else
    valid = true;

  if (valid)
    ProgressPublisher::showMessage(progressIndex, "Done!", 3.0);
}

UInt ExpansionMap::size() const
{
  return leftExpansionNodes.size();
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void ExpansionMap::generateExpansionMaps()
{
  rightExpansionNodes.clear();
  leftExpansionNodes.clear();

  ExpansionNode nodeLeft, nodeRight;
  Real distanceMax;

// loop that generates all expansion nodes with appropriate distances from each other
  for (Real alpha = 0.0; alpha < M_PI; alpha += DynamicFootstepPlannerConfig::expansionNodeDistance / distanceMax)
  {
    const Int indexLastAngle = leftExpansionNodes.size() - 1;
    Real sinA, cosA;
    sincos(alpha, &sinA, &cosA);

    nodeLeft.angle = alpha;
    nodeRight.angle = -alpha;

    if (alpha > DynamicFootstepPlannerConfig::expansionMaxFootRotation)
    {
      nodeLeft.footAngle = DynamicFootstepPlannerConfig::expansionMaxFootRotation;
      nodeRight.footAngle = -DynamicFootstepPlannerConfig::expansionMaxFootRotation;
    }
    else
    {
      nodeLeft.footAngle = alpha;
      nodeRight.footAngle = -alpha;
    }
    nodeLeft.footDirection = PV2D(cos(nodeLeft.footAngle), sin(nodeLeft.footAngle));
    nodeRight.footDirection = PV2D(cos(nodeRight.footAngle), sin(nodeRight.footAngle));

    if (alpha < PIHALF)
      distanceMax = DynamicFootstepPlannerConfig::expansionForward * DynamicFootstepPlannerConfig::expansionSideways
          / sqrt(pow(DynamicFootstepPlannerConfig::expansionForward * sinA, 2) + pow(DynamicFootstepPlannerConfig::expansionSideways * cosA, 2));
    else
      distanceMax = DynamicFootstepPlannerConfig::expansionBackward * DynamicFootstepPlannerConfig::expansionSideways
          / sqrt(pow(DynamicFootstepPlannerConfig::expansionBackward * sinA, 2) + pow(DynamicFootstepPlannerConfig::expansionSideways * cosA, 2));

    nodeLeft.position = PV2D(distanceMax * cosA, distanceMax * sinA);
    nodeRight.position = PV2D(distanceMax * cosA, -distanceMax * sinA);

    PV2D displacement(-nodeLeft.position);
    displacement *= DynamicFootstepPlannerConfig::expansionNodeDistance / displacement.norm();

    setHeightLimits(alpha, nodeLeft, nodeRight);
    leftExpansionNodes.push_back(nodeLeft);
    rightExpansionNodes.push_back(nodeRight);

    for (Real distance = distanceMax - DynamicFootstepPlannerConfig::expansionNodeDistance; distance > 0.0; distance -=
        DynamicFootstepPlannerConfig::expansionNodeDistance)
    {
      nodeLeft.position += displacement;
      if (!isPointDistanceAboveThreshold(nodeLeft.position, indexLastAngle, leftExpansionNodes))
        continue;
      nodeRight.position = PV2D(nodeLeft.position.C0, -nodeLeft.position.C1);
      setHeightLimits(alpha, nodeLeft, nodeRight);
      leftExpansionNodes.push_back(nodeLeft);
      rightExpansionNodes.push_back(nodeRight);
    }
  }
}

bool ExpansionMap::isPointDistanceAboveThreshold(const PV2D &point, Int index, const std::vector<ExpansionNode> &nodes)
{
  const Real distSq = pow(DynamicFootstepPlannerConfig::expansionNodeDistance, 2);

  for (; index >= 0; --index)
  {
    if ((point - nodes[index].position).normSquared() < distSq)
      return false;
  }

  return true;
}

void ExpansionMap::setHeightLimits(const Real angle, ExpansionNode &nodeLeft, ExpansionNode &nodeRight)
{
  Real rootArg;
  if (angle < PIHALF)
    rootArg = 1 - pow(nodeLeft.position.C0 / DynamicFootstepPlannerConfig::expansionForward, 2)
        - pow(nodeLeft.position.C1 / DynamicFootstepPlannerConfig::expansionSideways, 2);
  else
    rootArg = 1 - pow(nodeLeft.position.C0 / DynamicFootstepPlannerConfig::expansionBackward, 2)
        - pow(nodeLeft.position.C1 / DynamicFootstepPlannerConfig::expansionSideways, 2);

  if (rootArg < 0)
    rootArg = 0;
  const Real root = sqrt(rootArg);

  nodeLeft.heightDown = -DynamicFootstepPlannerConfig::expansionDown * root;
  if (nodeLeft.heightDown > -0.01)
    nodeLeft.heightDown = -0.01;
  nodeRight.heightDown = nodeLeft.heightDown;

  nodeLeft.heightUp = DynamicFootstepPlannerConfig::expansionUp * root;
  if (nodeLeft.heightUp < 0.01)
    nodeLeft.heightUp = 0.01;
  nodeRight.heightUp = nodeLeft.heightUp;
}

void ExpansionMap::findNextExpansionIndices()
{
  std::vector<std::pair<Real, UInt> > firstAngleIndices;
  firstAngleIndices.push_back(std::make_pair(0.0, 0));

  Real lastAngle = 0.0;
  for (UInt i = 1; i < leftExpansionNodes.size(); ++i)
  {
    if (leftExpansionNodes[i].angle == lastAngle)
      continue;
    lastAngle = leftExpansionNodes[i].angle;
    firstAngleIndices.push_back(std::make_pair(lastAngle, i));
  }

  UInt index = 0;
  while (index < leftExpansionNodes.size())
  {
    const Real currentAngle = leftExpansionNodes[index].angle;
    const Real nextAngle = currentAngle * (1 + DynamicFootstepPlannerConfig::angularSkipIncrease) + DynamicFootstepPlannerConfig::angularSkipDistance;

    UInt nextIndex = 0;
    for (UInt j = 0; j < firstAngleIndices.size(); ++j)
    {
      if (firstAngleIndices[j].first < nextAngle)
        continue;

      nextIndex = firstAngleIndices[j].second;
      break;
    }
    if (nextIndex == 0)
      nextIndex = leftExpansionNodes.size();

    while (index < leftExpansionNodes.size() && leftExpansionNodes[index].angle == currentAngle)
    {
      leftExpansionNodes[index].nextExpansionNodeIndex = nextIndex;
      rightExpansionNodes[index].nextExpansionNodeIndex = nextIndex;
      ++index;
    }
  }
}

void ExpansionMap::addZeroAndSideSteps()
{
  ExpansionNode nodeLeft, nodeRight;

//add zero step
  nodeLeft.angle = nodeRight.angle = 0.0;
  nodeLeft.footAngle = nodeRight.footAngle = 0.0;
  nodeLeft.footDirection = nodeRight.footDirection = PV2D(1.0, 0.0);
  nodeLeft.heightUp = nodeRight.heightUp = 0.01;
  nodeLeft.heightDown = nodeRight.heightDown = -0.01;
  nodeLeft.position = nodeRight.position = PV2D(0.0, 0.0);
  leftExpansionNodes.push_back(nodeLeft);
  rightExpansionNodes.push_back(nodeRight);
  leftExpansionNodes.back().nextExpansionNodeIndex = leftExpansionNodes.size();
  rightExpansionNodes.back().nextExpansionNodeIndex = rightExpansionNodes.size();

//add side step
  nodeLeft.position = PV2D(0.0, DynamicFootstepPlannerConfig::expansionSideways);
  nodeRight.position = PV2D(0.0, -DynamicFootstepPlannerConfig::expansionSideways);
  leftExpansionNodes.push_back(nodeLeft);
  rightExpansionNodes.push_back(nodeRight);
  leftExpansionNodes.back().nextExpansionNodeIndex = leftExpansionNodes.size();
  rightExpansionNodes.back().nextExpansionNodeIndex = rightExpansionNodes.size();
}

void ExpansionMap::generateExpansionConnections()
{
  const Real maxDistance = 2 * std::max(DynamicFootstepPlannerConfig::expansionForward, DynamicFootstepPlannerConfig::expansionBackward);
  const Real maxHeight = std::max(DynamicFootstepPlannerConfig::expansionUp, fabs(DynamicFootstepPlannerConfig::expansionDown));
  const Real heightResolution = 0.01;

  expansionConnections.resize(0.0, maxDistance, DynamicFootstepPlannerConfig::expansionNodeDistance);
  for (UInt i = 0; i < expansionConnections.size(); ++i)
    expansionConnections[i].resize(-maxHeight, maxHeight, heightResolution);

  for (UInt i = 0; i < expansionConnections.size(); ++i)
  {
    const Real distance = expansionConnections.getPosition(i);
    for (UInt j = 0; j < expansionConnections[i].size(); ++j)
    {
      const Real height = expansionConnections[i].getPosition(j);
      ExpansionConnection &connection = expansionConnections[i][j];
      Real coeffANormal, coeffBNormal, coeffAOver, coeffBOver;
      getParabolaCoefficients(distance, height, DynamicFootstepPlannerConfig::stepNormalHeight, coeffANormal, coeffBNormal);
      getParabolaCoefficients(distance, height, DynamicFootstepPlannerConfig::stepOverHeight, coeffAOver, coeffBOver);

      const UInt size = (Int)(distance / DynamicFootstepPlannerConfig::trajectoryPoseDistances);
      if (size == 0)
        continue;

      const Real posChange = distance / (Real)(size + 1);

      connection.positionsNormal.resize(size);
      connection.positionsOver.resize(size);

      Real pos = posChange;
      for (UInt posIndex = 0; posIndex < size; ++posIndex, pos += posChange)
      {
        connection.positionsNormal[posIndex] = std::make_pair(pos, coeffANormal * pow(pos, 2) + coeffBNormal * pos);
        connection.positionsOver[posIndex] = std::make_pair(pos, coeffAOver * pow(pos, 2) + coeffBOver * pos);
      }
    }
  }
}

void ExpansionMap::getParabolaCoefficients(const Real &distance, const Real &heightEnd, Real stepHeight, Real &coeffA, Real &coeffB)
{
  if (heightEnd < 0)
    stepHeight = -heightEnd + stepHeight;

  coeffA = (-heightEnd - 2 * stepHeight - 2 * sqrt(pow(stepHeight, 2) + stepHeight * heightEnd)) / pow(distance, 2);
  coeffB = heightEnd / distance - coeffA * distance;
}

bool ExpansionMap::validateReachabilities()
{
  if (!initializeKDLSolvers())
  {
    ProgressPublisher::showMessage(progressIndex, "Could not validate KDL solvers!", 3.0);
    return false;
  }

  bool state = checkLeftExpansionNodes();

  deleteKDLSolvers();
  return state;
}

bool ExpansionMap::initializeKDLSolvers()
{
  KDL::Tree kdlTree;     ///< KDL tree, created from urdf given as an argument to the constructor.
  KDL::Chain chainRight, chainLeft;

  bool readFine = kdl_parser::treeFromParam("/robot_description", kdlTree);
  if (!readFine)
  {
    ROS_ERROR("Could not parse urdf.");
    return false;
  }

  if (!kdlTree.getChain("left_sole_link", "base_link", chainLeft))
  {
    ROS_ERROR("Could not find chain between base_link and left_sole_link");
    return false;
  }
  if (!kdlTree.getChain("right_sole_link", "base_link", chainRight))
  {
    ROS_ERROR("Could not find chain between base_link and right_sole_link");
    return false;
  }

  KDL::JntArray jointArrayMinLeft(6);
  KDL::JntArray jointArrayMaxLeft(6);
  KDL::JntArray jointArrayMinRight(6);
  KDL::JntArray jointArrayMaxRight(6);

  jointArrayMinLeft.data[0] = -0.523598775598;
  jointArrayMaxLeft.data[0] = 0.261799387799;
  jointArrayMinLeft.data[1] = -1.308996939;
  jointArrayMaxLeft.data[1] = 0.785398163397;
  jointArrayMinLeft.data[2] = 0.0;
  jointArrayMaxLeft.data[2] = 2.61799387799;
  jointArrayMinLeft.data[3] = -1.74532925199;
  jointArrayMaxLeft.data[3] = 0.785398163397;
  jointArrayMinLeft.data[4] = -0.261799387799;
  jointArrayMaxLeft.data[4] = 0.523598775598;
  jointArrayMinLeft.data[5] = -0.785398163397;
  jointArrayMaxLeft.data[5] = 0.523598775598;

  jointArrayMinRight.data[0] = -0.261799387799;
  jointArrayMaxRight.data[0] = 0.523598775598;
  jointArrayMinRight.data[1] = -1.308996939;
  jointArrayMaxRight.data[1] = 0.785398163397;
  jointArrayMinRight.data[2] = 0.0;
  jointArrayMaxRight.data[2] = 2.61799387799;
  jointArrayMinRight.data[3] = -1.74532925199;
  jointArrayMaxRight.data[3] = 0.785398163397;
  jointArrayMinRight.data[4] = -0.523598775598;
  jointArrayMaxRight.data[4] = 0.261799387799;
  jointArrayMinRight.data[5] = -0.523598775598;
  jointArrayMaxRight.data[5] = 0.785398163397;

  fkSolverPosLeft = new KDL::ChainFkSolverPos_recursive(chainLeft);
  fkSolverPosRight = new KDL::ChainFkSolverPos_recursive(chainRight);
  ikSolverVelLeft = new KDL::ChainIkSolverVel_wdls(chainLeft, 1e-5, 500);
  ikSolverVelRight = new KDL::ChainIkSolverVel_wdls(chainRight, 1e-5, 500);

  ikSolverPosJLLeft = new KDL::ChainIkSolverPos_NR_JL(chainLeft, jointArrayMinLeft, jointArrayMaxLeft, *fkSolverPosLeft, *ikSolverVelLeft, 500, 1e-5);
  ikSolverPosJLRight = new KDL::ChainIkSolverPos_NR_JL(chainRight, jointArrayMinRight, jointArrayMaxRight, *fkSolverPosRight, *ikSolverVelRight, 500, 1e-5);

  return true;
}

bool ExpansionMap::checkLeftExpansionNodes()
{
  Pose6D poseRightFoot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Pose6D poseLeftFoot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Pose6D poseBase(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  UInt errorCounter = 0;
  for (UInt i = 0; i < leftExpansionNodes.size(); ++i)
  {
    const ExpansionNode &node = leftExpansionNodes[i];
    poseBase.yaw = node.footAngle * 0.5;
    poseLeftFoot.x = node.position.C0;
    poseLeftFoot.y = node.position.C1;
    poseLeftFoot.yaw = node.footAngle;

    //case up
//    poseLeftFoot.z = node.heightUp;
//    poseBase.z = DynamicFootstepPlannerConfig::expansionBaseHeight;
//    if (!findIKSolution(poseLeftFoot, poseBase, ikSolverPosJLLeft) || !findIKSolution(poseRightFoot, poseBase, ikSolverPosJLRight))
//    {
//      ++errorCounter;
//      continue;
//    }
//
//    //case down
//    poseLeftFoot.z = node.heightDown;
//    poseBase.z = DynamicFootstepPlannerConfig::expansionBaseHeight + node.heightDown;
//    if (!findIKSolution(poseLeftFoot, poseBase, ikSolverPosJLLeft) || !findIKSolution(poseRightFoot, poseBase, ikSolverPosJLRight))
//    {
//      ++errorCounter;
//      continue;
//    }

    //case center
    poseLeftFoot.z = 0.0;
    poseBase.z = DynamicFootstepPlannerConfig::expansionBaseHeight;
    if (!findIKSolution(poseLeftFoot, poseBase, ikSolverPosJLLeft) || !findIKSolution(poseRightFoot, poseBase, ikSolverPosJLRight))
    {
      ++errorCounter;
      continue;
    }
  }

  if (errorCounter == 0)
  {
    return true;
  }
  else
  {
    std::string message;
    message = "Errors in " + std::to_string(errorCounter) + " out of " + std::to_string(leftExpansionNodes.size()) + " poses.";
    ProgressPublisher::showMessage(progressIndex, message, 3.0);
    return false;
  }

}

bool ExpansionMap::findIKSolution(const Pose6D &rootPose, const Pose6D &tipPose, KDL::ChainIkSolverPos_NR_JL* solver)
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

  KDL::JntArray jointsInitial(6), joints;
  for (UInt i = 0; i < 6; ++i)
    jointsInitial.data[i] = 0.0;

  Int errorCode = solver->CartToJnt(jointsInitial, frame, joints);

  if (errorCode == KDL::ChainFkSolverPos_recursive::E_NOERROR || errorCode == KDL::ChainIkSolverVel_wdls::E_CONVERGE_PINV_SINGULAR)
    return true;
  else
    return false;
}

void ExpansionMap::deleteKDLSolvers()
{
  if (ikSolverPosJLRight != nullptr)
  {
    delete ikSolverPosJLRight;
    ikSolverPosJLRight = nullptr;
  }

  if (ikSolverPosJLLeft != nullptr)
  {
    delete ikSolverPosJLLeft;
    ikSolverPosJLLeft = nullptr;
  }

  if (fkSolverPosLeft != nullptr)
  {
    delete fkSolverPosLeft;
    fkSolverPosLeft = nullptr;
  }

  if (fkSolverPosRight != nullptr)
  {
    delete fkSolverPosRight;
    fkSolverPosRight = nullptr;
  }

  if (ikSolverVelRight != nullptr)
  {
    delete ikSolverVelRight;
    ikSolverVelRight = nullptr;
  }

  if (ikSolverVelLeft != nullptr)
  {
    delete ikSolverVelLeft;
    ikSolverVelLeft = nullptr;
  }
}
