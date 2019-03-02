//ros
#include <std_msgs/Bool.h>

//local
#include "footstep_executor/footstep_executor.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

FootstepExecutor::FootstepExecutor() :
    generator("base_link", "left_sole_link", "right_sole_link"), firstJointsSet(false)
{
  initialize();
}

void FootstepExecutor::run()
{
  ros::WallRate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    std_msgs::Bool msg;

    if (trajectoryClient->getState().isDone())
      msg.data = false;
    else
      msg.data = true;

    publisherBusy.publish(msg);
  }
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void FootstepExecutor::initialize()
{
  jointStates.resize(14);

  subscriberJointStates = nh.subscribe("/joint_states", 1, &FootstepExecutor::subscriberJointStatesHandler, this);
  publisherBusy = nh.advertise<std_msgs::Bool>("busy_executing_step", 0);

  serviceSingleFootstepExecution = nh.advertiseService("execute_single_footstep", &FootstepExecutor::serviceSingleFootstepExecutionHandler, this);
  serviceStepToInitPose = nh.advertiseService("step_to_init_pose", &FootstepExecutor::serviceStepToInitPoseHandler, this);
  trajectoryClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/lower_body_controller/follow_joint_trajectory");
}

void FootstepExecutor::subscriberJointStatesHandler(const sensor_msgs::JointState &msg)
{
  firstJointsSet = true;

  UInt index = 0;
  for (Int i = 22; i >= 16; --i, ++index)
    jointStates[index] = msg.position[i];
  for (Int i = 29; i >= 23; --i, ++index)
    jointStates[index] = msg.position[i];
}

bool FootstepExecutor::serviceSingleFootstepExecutionHandler(reemc_trajectory_execution::single_footstep_executionRequest &req,
                                                             reemc_trajectory_execution::single_footstep_executionResponse &res)
{
  control_msgs::FollowJointTrajectoryGoal goal;
  RobotTrajectoryGenerator::Foot supportFoot, swingFoot;

  if (req.foot == reemc_trajectory_execution::single_footstep_executionRequest::left)
  {
    supportFoot = RobotTrajectoryGenerator::Foot::right;
    swingFoot = RobotTrajectoryGenerator::Foot::left;
  }
  else
  {
    supportFoot = RobotTrajectoryGenerator::Foot::left;
    swingFoot = RobotTrajectoryGenerator::Foot::right;
  }

  Pose6D footstepPose(req.footstep_x, req.footstep_y, req.footstep_z, 0.0, 0.0, req.footstep_yaw);

  res.status = reemc_trajectory_execution::single_footstep_executionResponse::success;
  generator.setInitialJointStates(jointStates);
  if (!generator.addTrajectoryBaseAboveFoot(goal, supportFoot, req.base_height, req.footstep_yaw * 0.5, req.base_speed))
    res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;
  else
    generator.setInitialJointStates(goal.trajectory.points.back().positions);

  if (res.status != reemc_trajectory_execution::single_footstep_executionResponse::success
      || !generator.addTrajectoryFootstepRectangularRelativePose(goal, swingFoot, req.footstep_height, footstepPose, req.footstep_duration))
    res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;
  else
    generator.setInitialJointStates(goal.trajectory.points.back().positions);

  if (res.status != reemc_trajectory_execution::single_footstep_executionResponse::success
      || !generator.addTrajectoryBaseAboveFoot(goal, swingFoot, req.base_height, 0.0, req.base_speed))
    res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;

  if (res.status == reemc_trajectory_execution::single_footstep_executionResponse::success)
  {
    goal.trajectory.header.stamp = ros::Time::now();
    trajectoryClient->sendGoal(goal);
  }

  return true;
}

bool FootstepExecutor::serviceStepToInitPoseHandler(reemc_trajectory_execution::step_to_init_poseRequest &req,
                                                    reemc_trajectory_execution::step_to_init_poseResponse &res)
{
  control_msgs::FollowJointTrajectoryGoal goal;
    RobotTrajectoryGenerator::Foot supportFoot, swingFoot;

    if (req.support == reemc_trajectory_execution::single_footstep_executionRequest::left)
    {
      supportFoot = RobotTrajectoryGenerator::Foot::left;
      swingFoot = RobotTrajectoryGenerator::Foot::right;
    }
    else
    {
      supportFoot = RobotTrajectoryGenerator::Foot::right;
      swingFoot = RobotTrajectoryGenerator::Foot::left;
    }

    Pose6D goalPose(0.0, (supportFoot == RobotTrajectoryGenerator::Foot::right ? req.foot_separation : -req.foot_separation), 0.0, 0.0, 0.0, 0.0);

    res.status = reemc_trajectory_execution::single_footstep_executionResponse::success;
    generator.setInitialJointStates(jointStates);
    if (!generator.addTrajectoryBaseAboveFoot(goal, supportFoot, req.base_height, 0.0, req.base_speed))
      res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;
    else
      generator.setInitialJointStates(goal.trajectory.points.back().positions);

    if (res.status != reemc_trajectory_execution::single_footstep_executionResponse::success
        || !generator.addTrajectoryFootstepRelativePose(goal, swingFoot, 0.05, goalPose, req.footstep_duration))
      res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;
    else
      generator.setInitialJointStates(goal.trajectory.points.back().positions);

    if (res.status != reemc_trajectory_execution::single_footstep_executionResponse::success
        || !generator.addTrajectoryBaseBetweenFeet(goal, req.base_height, req.base_speed))
      res.status = reemc_trajectory_execution::single_footstep_executionResponse::fail;

    if (res.status == reemc_trajectory_execution::single_footstep_executionResponse::success)
    {
      goal.trajectory.header.stamp = ros::Time::now();
      trajectoryClient->sendGoal(goal);
    }

    return true;
}
