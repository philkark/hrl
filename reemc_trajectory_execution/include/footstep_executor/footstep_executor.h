#ifndef REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_EXECUTOR_
#define REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_EXECUTOR_

//std
#include <iostream>
#include <vector>

//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

//personal
#include <math_std/math_std.h>

//local
#include "footstep_executor/robot_trajectory_generator.h"
#include "reemc_trajectory_execution/single_footstep_execution.h"
#include "reemc_trajectory_execution/step_to_init_pose.h"

class FootstepExecutor
{
public:
  FootstepExecutor();

  void run();

private:
  RobotTrajectoryGenerator generator;

  ros::NodeHandle nh;
  ros::Subscriber subscriberJointStates;
  ros::Publisher publisherBusy;
  ros::ServiceServer serviceSingleFootstepExecution;
  ros::ServiceServer serviceStepToInitPose;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* trajectoryClient;

  bool firstJointsSet;
  std::vector<Real> jointStates;

  void initialize();

  void subscriberJointStatesHandler(const sensor_msgs::JointState &msg);

  bool serviceSingleFootstepExecutionHandler(reemc_trajectory_execution::single_footstep_executionRequest &req,
                                             reemc_trajectory_execution::single_footstep_executionResponse &res);

  bool serviceStepToInitPoseHandler(reemc_trajectory_execution::step_to_init_poseRequest &req, reemc_trajectory_execution::step_to_init_poseResponse &res);
};

#endif // REEMC_TRAJECTORY_EXECUTION_FOOTSTEP_EXECUTOR_
