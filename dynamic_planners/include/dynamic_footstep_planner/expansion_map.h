#ifndef DYNAMIC_PLANNERS_EXPANSION_MAP_H_
#define DYNAMIC_PLANNERS_EXPANSION_MAP_H_

#include <vector>
#include <utility>

#include <math_std/math_std.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include "dynamic_footstep_planner/dynamic_footstep_planner_config.h"
#include "dynamic_footstep_planner/template_interval_vector.h"

struct ExpansionNode
{
  PV2D position;
  Real angle;

  PV2D footDirection;
  Real footAngle;

  Real heightUp;
  Real heightDown;

  UInt nextExpansionNodeIndex;
};

struct ExpansionConnection
{
  std::vector<std::pair<Real, Real> > positionsNormal;
  std::vector<std::pair<Real, Real> > positionsOver;
};

class ExpansionMap
{
public:
  ExpansionMap();

  bool isValid();

  const std::vector<ExpansionNode> &getLeftExpansionNodes() const;

  const std::vector<ExpansionNode> &getRightExpansionNodes() const;

  const ExpansionConnection &getExpansionConnection(const Real distance, const Real height) const;

  UInt size() const;

  void initializeExpansionMap(bool validate, Int progressIndex);

private:
  struct Pose6D
  {
    Real x, y, z, roll, pitch, yaw;

    Pose6D()
    {

    }

    Pose6D(const Real x, const Real y, const Real z, const Real roll, const Real pitch, const Real yaw) :
        x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw)
    {

    }
  };

  bool valid;
  int progressIndex;

  std::vector<ExpansionNode> leftExpansionNodes;
  std::vector<ExpansionNode> rightExpansionNodes;
  IntervalVector<IntervalVector<ExpansionConnection> > expansionConnections;

  KDL::ChainFkSolverPos_recursive* fkSolverPosLeft;
  KDL::ChainFkSolverPos_recursive* fkSolverPosRight;
  KDL::ChainIkSolverVel_wdls* ikSolverVelLeft;
  KDL::ChainIkSolverVel_wdls* ikSolverVelRight;
  KDL::ChainIkSolverPos_NR_JL* ikSolverPosJLLeft;
  KDL::ChainIkSolverPos_NR_JL* ikSolverPosJLRight;

  void generateExpansionMaps();

  bool isPointDistanceAboveThreshold(const PV2D &point, Int index, const std::vector<ExpansionNode> &nodes);

  void setHeightLimits(const Real angle, ExpansionNode &nodeLeft, ExpansionNode &nodeRight);

  void findNextExpansionIndices();

  void addZeroAndSideSteps();

  void generateExpansionConnections();

  void getParabolaCoefficients(const Real &distance, const Real &heightEnd, Real stepHeight, Real &coeffA, Real &coeffB);

  // compute rechabilities via inverse kinematics
  bool validateReachabilities();

  bool initializeKDLSolvers();

  bool checkLeftExpansionNodes();

  bool findIKSolution(const Pose6D &rootPose, const Pose6D &tipPose, KDL::ChainIkSolverPos_NR_JL* solver);

  void deleteKDLSolvers();

};

#endif // DYNAMIC_PLANNERS_EXPANSION_MAP_H_
