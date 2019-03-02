//std
#include <vector>
#include <string>

//ros
#include <tools_std/ros_param_loader.h>
#include <tools_std/dynamic_transform_setter.h>
#include <tools_std/timer.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib_msgs/GoalStatusArray.h>

//local
#include "controllers/dynamic_rviz_controller.h"
#include "dynamic_visualizer/dynamic_rviz_visualizer.h"
#include "dynamic_planners/get_height_map.h"
#include "common/progress_publisher.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dynamic_rviz_controller");
  ProgressPublisher::initialize();
  dynamic_planners::DynamicRvizController* controller = new dynamic_planners::DynamicRvizController;
  controller->run();

  return 0;
}

namespace dynamic_planners
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

DynamicRvizController::DynamicRvizController() :
    dynamicHeightMap(nullptr), dynamicAStarGridPlanner(nullptr), dynamicRvizVisualizer(nullptr), globalMap(nullptr), dynamicFootstepPlanner(nullptr)
{
  initialize();
}

void DynamicRvizController::run()
{
  ros::Rate rate(30);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();

    if (firstTransformSet && dynamicHeightMap != nullptr && runMapping)
      dynamicHeightMap->updateMap(transformBaseToCamera, robotPose);

    if (runMapping && runLocalGridMapping)
    {
      dynamicAStarGridPlanner->setRobotPose(PV3D(robotPose.x, robotPose.y, robotPose.yaw));
      dynamicAStarGridPlanner->findAStarPathLocal(1);
    }

    if (runMapping && runFootstepPlanning)
    {
      dynamicFootstepPlanner->setRobotPose(PV3D(robotPose.x, robotPose.y, robotPose.yaw));
      dynamicFootstepPlanner->setInitialFeet(PV4D(leftFootPose.x, leftFootPose.y, leftFootPose.z, leftFootPose.yaw),
                                             PV4D(rightFootPose.x, rightFootPose.y, rightFootPose.z, rightFootPose.yaw), supportFoot);
      dynamicFootstepPlanner->findPlan(0.3, 1e10);
    }

    dynamicRvizVisualizer->updateMap();
    dynamicRvizVisualizer->updateMapGlobal();
    dynamicRvizVisualizer->updateAStarPathGlobal();
    dynamicRvizVisualizer->updateAStarPathLocal();
    dynamicRvizVisualizer->updateFootstepPlan();
    ProgressPublisher::processDeleteMessageQueue();

    if (runMapping && runFootstepPlanning && runFootstepExecution && dynamicFootstepPlanner->isPlanValid())
      executeNextFootstep();
    else if (dynamicFootstepPlanner->isGoalReached())
      executeZeroFootstep();
  }
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void DynamicRvizController::initialize()
{
  initializeMessageHandling();
  waitForRviz(10.0);
  initializeParameters();
}

void DynamicRvizController::initializeParameters()
{
  runMapping = false;
  runLocalGridMapping = false;
  firstTransformSet = false;
  supportFoot = Foot::left;
  footstepFinished = true;

  dynamicAStarGridPlanner = new DynamicAStarGridPlanner;
  dynamicFootstepPlanner = new DynamicFootstepPlanner;
  dynamicRvizVisualizer = new DynamicRvizVisualizer;
  dynamicRvizVisualizer->setDynamicAStarGridPlanner(dynamicAStarGridPlanner);
  dynamicRvizVisualizer->setDynamicFootstepPlanner(dynamicFootstepPlanner);
  dynamicFootstepPlanner->setProgressMessageIndex(ProgressPublisher::addPublisher("info_footstep_planner"));

  subscriberDisplaySettingsChangedHandler(std_msgs::Empty());
}

void DynamicRvizController::initializeMessageHandling()
{
  subscriberCreateMapper = nh.subscribe("create_mapper", 0, &DynamicRvizController::subscriberCreateMapperHandler, this);
  subscriberRunMapping = nh.subscribe("is_mapping_running_changed", 0, &DynamicRvizController::subscriberRunMappingHandler, this);
  subscriberRobotPoses = nh.subscribe("reemc_base_camera_poses", 0, &DynamicRvizController::subscriberRobotPosesHandler, this);
  subscriberResetRobotPose = nh.subscribe("reset_robot_pose_signal", 0, &DynamicRvizController::subscriberResetRobotPoseHandler, this);
  subscriberSetGlobalMap = nh.subscribe("set_global_map", 0, &DynamicRvizController::subscriberSetGlobalMapHandler, this);
  subscriberFindGridPlanGlobal = nh.subscribe("clicked_point", 0, &DynamicRvizController::subscriberFindGridPlanGlobalHandler, this);
  subscriberFindGridPlanLocal = nh.subscribe("astar_local_is_running_changed", 0, &DynamicRvizController::subscriberFindGridPlanLocalHandler, this);
  subscriberInitializeFootstepPlanner = nh.subscribe("initialize_footstep_planner", 0, &DynamicRvizController::subscriberInitializeFootstepPlannerHandler,
                                                     this);
  subscriberRunFootstepPlanning = nh.subscribe("run_footstep_planning_changed", 0, &DynamicRvizController::subscriberRunFootstepPlanningHandler, this);
  subscriberDisplaySettingsChanged = nh.subscribe("display_update_settings", 0, &DynamicRvizController::subscriberDisplaySettingsChangedHandler, this);
  subscriberRunFootstepExecutionChanged = nh.subscribe("run_footstep_execution_changed", 0,
                                                       &DynamicRvizController::subscriberRunFootstepExecutionChangedHandler, this);
  subscriberFootstepExecutorBusy = nh.subscribe("busy_executing_step", 0, &DynamicRvizController::subscriberFootstepExecutorBusyHandler, this);
  publisherResetRobotPose = nh.advertise<geometry_msgs::Pose>("reset_robot_pose", 1);
  publisherJointTrajectory = nh.advertise<trajectory_msgs::JointTrajectory>("/lower_body_controller/command", 1);
  serviceClientGetHeightMap = nh.serviceClient<dynamic_planners::get_height_map>("get_height_map");
  serviceClientFootstepExecution = nh.serviceClient<reemc_trajectory_execution::single_footstep_execution>("execute_single_footstep");
  serviceClientStepToInitPose = nh.serviceClient<reemc_trajectory_execution::step_to_init_pose>("step_to_init_pose");
}

bool DynamicRvizController::waitForRviz(const Real duration)
{
  std::cout << "Waiting for rviz..." << std::endl;
  ros::WallTime timeStart = ros::WallTime::now();
  ros::WallDuration maxWaitingTime(duration);

  while (ros::ok() && (ros::WallTime::now() - timeStart) < maxWaitingTime)
  {
    if (subscriberFindGridPlanGlobal.getNumPublishers() > 0)
    {
      std::cout << "Found rviz!" << std::endl;
      return true;
    }

    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
  }

  std::cout << "Waiting for rviz timeout!" << std::endl;
  ros::shutdown();
  return false;
}

void DynamicRvizController::subscriberCreateMapperHandler(const std_msgs::Empty &msg)
{
  if (dynamicHeightMap != nullptr)
    delete dynamicHeightMap;

  dynamicHeightMap = new DynamicHeightMap;
  dynamicRvizVisualizer->setDynamicHeightMap(dynamicHeightMap);
  if (globalMap != nullptr)
    dynamicHeightMap->setGlobalMap(globalMap);
  dynamicAStarGridPlanner->setDynamicHeightMap(dynamicHeightMap);
  dynamicFootstepPlanner->setDynamicHeightMap(dynamicHeightMap);
}

void DynamicRvizController::subscriberRunMappingHandler(const std_msgs::Empty &msg)
{
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberRunMappingHandler", "is_mapping_running", runMapping, false);
  if (!runMapping)
  {
    dynamicAStarGridPlanner->invalidateLocalPath();
    dynamicFootstepPlanner->invalidatePlan();
  }
}

void DynamicRvizController::subscriberRobotPosesHandler(const geometry_msgs::PoseArray &msg)
{
  const geometry_msgs::Pose &poseBase = msg.poses[0];
  const geometry_msgs::Pose &poseCamera = msg.poses[1];
  const geometry_msgs::Pose &poseLeftFoot = msg.poses[2];
  const geometry_msgs::Pose &poseRightFoot = msg.poses[3];

  tf::StampedTransform transform;
  transform.stamp_ = ros::Time::now();
  transform.child_frame_id_ = "base_link";
  transform.frame_id_ = "map";
  transform.setOrigin(tf::Vector3(poseBase.position.x, poseBase.position.y, poseBase.position.z));
  transform.setRotation(tf::Quaternion(poseBase.orientation.x, poseBase.orientation.y, poseBase.orientation.z, poseBase.orientation.w));
  transformBroadcaster.sendTransform(transform);
  robotPose.x = transform.getOrigin()[0];
  robotPose.y = transform.getOrigin()[1];
  robotPose.z = transform.getOrigin()[2];
  transform.getBasis().getRPY(robotPose.roll, robotPose.pitch, robotPose.yaw);

  transform.setOrigin(tf::Vector3(poseLeftFoot.position.x, poseLeftFoot.position.y, poseLeftFoot.position.z));
  transform.setRotation(tf::Quaternion(poseRightFoot.orientation.x, poseRightFoot.orientation.y, poseRightFoot.orientation.z, poseRightFoot.orientation.w));
  leftFootPose.x = transform.getOrigin()[0];
  leftFootPose.y = transform.getOrigin()[1];
  leftFootPose.z = transform.getOrigin()[2];
  transform.getBasis().getRPY(leftFootPose.roll, leftFootPose.pitch, leftFootPose.yaw);

  transform.setOrigin(tf::Vector3(poseRightFoot.position.x, poseRightFoot.position.y, poseRightFoot.position.z));
  transform.setRotation(tf::Quaternion(poseRightFoot.orientation.x, poseRightFoot.orientation.y, poseRightFoot.orientation.z, poseRightFoot.orientation.w));
  rightFootPose.x = transform.getOrigin()[0];
  rightFootPose.y = transform.getOrigin()[1];
  rightFootPose.z = transform.getOrigin()[2];
  transform.getBasis().getRPY(rightFootPose.roll, rightFootPose.pitch, rightFootPose.yaw);

  transformBaseToCamera.C03 = poseCamera.position.x;
  transformBaseToCamera.C13 = poseCamera.position.y;
  transformBaseToCamera.C23 = poseCamera.position.z;

  tf::Matrix3x3 mat;
  mat.setRotation(tf::Quaternion(poseCamera.orientation.x, poseCamera.orientation.y, poseCamera.orientation.z, poseCamera.orientation.w));

  transformBaseToCamera.C00 = mat.getColumn(0).x();
  transformBaseToCamera.C10 = mat.getColumn(0).y();
  transformBaseToCamera.C20 = mat.getColumn(0).z();
  transformBaseToCamera.C01 = mat.getColumn(1).x();
  transformBaseToCamera.C11 = mat.getColumn(1).y();
  transformBaseToCamera.C21 = mat.getColumn(1).z();
  transformBaseToCamera.C02 = mat.getColumn(2).x();
  transformBaseToCamera.C12 = mat.getColumn(2).y();
  transformBaseToCamera.C22 = mat.getColumn(2).z();
  firstTransformSet = true;

  dynamicRvizVisualizer->setMsgPoses(msg.poses[0]);
}

void DynamicRvizController::subscriberResetRobotPoseHandler(const std_msgs::Empty &msg)
{
  dynamicAStarGridPlanner->invalidateGlobalPath();
  dynamicAStarGridPlanner->invalidateLocalPath();
  dynamicFootstepPlanner->invalidateGlobalPath();
  dynamicFootstepPlanner->invalidatePlan();

  publishZeroTrajectory();
  for (UInt i = 0; i < 100; ++i)
  {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }

  publishRobotPose();
}

void DynamicRvizController::subscriberSetGlobalMapHandler(const std_msgs::Empty &msg)
{
  dynamic_planners::get_height_mapRequest req;
  dynamic_planners::get_height_mapResponse res;

  std::string isGlobalMapAutoSize;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberGlobalMapSizingTypeHandler", "global_map_sizing_type", isGlobalMapAutoSize, "fixed");
  if (isGlobalMapAutoSize == "auto")
    req.auto_size = true;
  else if (isGlobalMapAutoSize == "fixed")
    req.auto_size = false;

  Real tmp;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_min_x", tmp, 0.0);
  req.min_x = tmp;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_max_x", tmp, 0.0);
  req.max_x = tmp;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_min_y", tmp, 0.0);
  req.min_y = tmp;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_max_y", tmp, 0.0);
  req.max_y = tmp;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_resolution", tmp, 0.0);
  req.resolution = tmp;

  Real safetyDistance;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberSetGlobalMapHandler", "global_map_safety_distance", safetyDistance, 0.0);

  serviceClientGetHeightMap.call(req, res);

  if (globalMap != nullptr)
    delete globalMap;

  globalMap = new GlobalMap;
  globalMap->setMap(req, res, safetyDistance);

  if (dynamicHeightMap != nullptr)
    dynamicHeightMap->setGlobalMap(globalMap);
  dynamicRvizVisualizer->setNewGlobalMap(globalMap, res.objects);
  dynamicAStarGridPlanner->setGlobalMap(globalMap);

  //dynamicAStarGridPlanner->invalidateGlobalPath();
  dynamicAStarGridPlanner->invalidateLocalPath();
  //dynamicFootstepPlanner->invalidateGlobalPath();
  dynamicFootstepPlanner->invalidatePlan();
}

void DynamicRvizController::subscriberFindGridPlanGlobalHandler(const geometry_msgs::PointStamped &msg)
{
  if (globalMap == nullptr)
    return;

  dynamicAStarGridPlanner->setPlanningParameters();

  if (dynamicAStarGridPlanner->findAStarPathGlobal(PV2D(robotPose.x, robotPose.y), PV2D(msg.point.x, msg.point.y)))
    dynamicFootstepPlanner->setGlobalPath(dynamicAStarGridPlanner->getAStarPathGlobal());
}

void DynamicRvizController::subscriberFindGridPlanLocalHandler(const std_msgs::Empty &msg)
{
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberFindGridPlanLocalHandler", "astar_local_is_running", runLocalGridMapping, "false");
  if (!runLocalGridMapping)
    dynamicAStarGridPlanner->invalidateLocalPath();
  else
    dynamicAStarGridPlanner->setPlanningParameters();
}

void DynamicRvizController::subscriberInitializeFootstepPlannerHandler(const std_msgs::Empty &msg)
{
  if (dynamicFootstepPlanner->initializeSettings())
    dynamicRvizVisualizer->setFootstepMarkerSizes();
}

void DynamicRvizController::subscriberRunFootstepPlanningHandler(const std_msgs::Empty &msg)
{
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberRunFootstepPlanningHandler", "run_footstep_planning", runFootstepPlanning, "false");
  if (!runFootstepPlanning)
    dynamicFootstepPlanner->invalidatePlan();
}

void DynamicRvizController::subscriberDisplaySettingsChangedHandler(const std_msgs::Empty &msg)
{
  dynamicRvizVisualizer->updateDisplaySettings();
}

void DynamicRvizController::subscriberRunFootstepExecutionChangedHandler(const std_msgs::Empty &msg)
{
  RosParamLoader::getRosParam(nh, "DynamicRvizController::subscriberRunFootstepExecutionChangedHandler", "run_footstep_execution", runFootstepExecution,
                              "false");
}

void DynamicRvizController::subscriberFootstepExecutorBusyHandler(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    footstepFinished = false;
  }
  else
  {
    footstepFinished = true;
  }
}

void DynamicRvizController::publishZeroTrajectory()
{
  trajectory_msgs::JointTrajectory msg;
  msg.header.stamp = ros::Time::now();
  msg.joint_names.push_back("leg_left_sole_joint");
  msg.joint_names.push_back("leg_left_6_joint");
  msg.joint_names.push_back("leg_left_5_joint");
  msg.joint_names.push_back("leg_left_4_joint");
  msg.joint_names.push_back("leg_left_3_joint");
  msg.joint_names.push_back("leg_left_2_joint");
  msg.joint_names.push_back("leg_left_1_joint");
  msg.joint_names.push_back("leg_right_sole_joint");
  msg.joint_names.push_back("leg_right_6_joint");
  msg.joint_names.push_back("leg_right_5_joint");
  msg.joint_names.push_back("leg_right_4_joint");
  msg.joint_names.push_back("leg_right_3_joint");
  msg.joint_names.push_back("leg_right_2_joint");
  msg.joint_names.push_back("leg_right_1_joint");
  msg.points.resize(1);
  msg.points[0].time_from_start.fromSec(1.0);
  msg.points[0].positions.resize(14, 0.0);

  publisherJointTrajectory.publish(msg);
}

void DynamicRvizController::publishRobotPose()
{
  geometry_msgs::Pose msg;

  double roll, pitch, yaw;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_x", msg.position.x, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_y", msg.position.y, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_z", msg.position.z, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_roll", roll, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_pitch", pitch, 0.0);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::publishRobotPose", "set_robot_yaw", yaw, 0.0);
  tf::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();

  publisherResetRobotPose.publish(msg);
}

void DynamicRvizController::executeNextFootstep()
{
  reemc_trajectory_execution::single_footstep_executionRequest req;
  reemc_trajectory_execution::single_footstep_executionResponse res;

  Real footstepDuration;
  Real waitAfterStep;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::executeNextFootstep", "robot_footstep_duration", footstepDuration);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::executeNextFootstep", "robot_wait_after_step", waitAfterStep);

  PV4D pose = dynamicFootstepPlanner->getFootDisplacement(2);
  const Footstep &footstep = dynamicFootstepPlanner->getFootstepPlan()[2];

  req.footstep_x = pose.C0;
  req.footstep_y = pose.C1;
  req.footstep_z = pose.C2;
  req.footstep_yaw = pose.C3;
  req.foot =
      footstep.foot == Foot::left ? reemc_trajectory_execution::single_footstep_executionRequest::left :
          reemc_trajectory_execution::single_footstep_executionRequest::right;
  req.footstep_duration = footstepDuration;
  req.footstep_height = footstep.isStepOver ? DynamicFootstepPlannerConfig::stepOverHeight : DynamicFootstepPlannerConfig::stepNormalHeight;
  req.base_speed = DynamicFootstepPlannerConfig::robotSpeed;
  req.base_height = DynamicFootstepPlannerConfig::expansionBaseHeight;

  serviceClientFootstepExecution.call(req, res);

  if (res.status == reemc_trajectory_execution::single_footstep_executionResponse::success)
  {
    waitFootstepStart();
    waitFootstepFinish();

    ros::WallTime tb = ros::WallTime::now();
    while ((ros::WallTime::now() - tb).toSec() < waitAfterStep)
    {
      ros::spinOnce();
      ros::WallDuration(0.03).sleep();
    }
    supportFoot = supportFoot == Foot::right ? Foot::left : Foot::right;
  }

}

void DynamicRvizController::executeZeroFootstep()
{
  reemc_trajectory_execution::step_to_init_poseRequest req;
  reemc_trajectory_execution::step_to_init_poseResponse res;

  Real footstepDuration;
  Real waitAfterStep;
  RosParamLoader::getRosParam(nh, "DynamicRvizController::executeNextFootstep", "robot_footstep_duration", footstepDuration);
  RosParamLoader::getRosParam(nh, "DynamicRvizController::executeNextFootstep", "robot_wait_after_step", waitAfterStep);

  req.footstep_duration = footstepDuration;
  req.foot_separation = DynamicFootstepPlannerConfig::footSeparation;
  req.base_speed = DynamicFootstepPlannerConfig::robotSpeed;
  req.base_height = DynamicFootstepPlannerConfig::expansionBaseHeight;
  req.support =
      supportFoot == Foot::right ? reemc_trajectory_execution::step_to_init_poseRequest::right : reemc_trajectory_execution::step_to_init_poseRequest::left;

  serviceClientStepToInitPose.call(req, res);

  if (res.status == reemc_trajectory_execution::step_to_init_poseResponse::success)
  {
    waitFootstepStart();
    waitFootstepFinish();

    ros::WallTime tb = ros::WallTime::now();
    while ((ros::WallTime::now() - tb).toSec() < waitAfterStep)
    {
      ros::spinOnce();
      ros::WallDuration(0.03).sleep();
    }
    supportFoot = supportFoot == Foot::right ? Foot::left : Foot::right;
    dynamicFootstepPlanner->invalidateGlobalPath();
    dynamicAStarGridPlanner->invalidateGlobalPath();
    dynamicAStarGridPlanner->invalidateLocalPath();
  }
}

void DynamicRvizController::waitFootstepStart()
{
  while (footstepFinished)
  {
    ros::spinOnce();
    ros::WallDuration(0.03).sleep();
  }
}

void DynamicRvizController::waitFootstepFinish()
{
  while (!footstepFinished)
  {
    ros::spinOnce();
    ros::WallDuration(0.03).sleep();
  }
}
}
