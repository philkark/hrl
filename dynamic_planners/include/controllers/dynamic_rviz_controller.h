#ifndef DYNAMIC_PLANNERS_DYNAMIC_RVIZ_CONTROLLER_H_
#define DYNAMIC_PLANNERS_DYNAMIC_RVIZ_CONTROLLER_H_

//std
#include <list>

//ros
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

//personal
#include <reemc_trajectory_execution/single_footstep_execution.h>
#include <reemc_trajectory_execution/step_to_init_pose.h>

//local
#include "dynamic_visualizer/dynamic_rviz_visualizer.h"
#include "dynamic_height_map/dynamic_height_map.h"
#include "dynamic_height_map/global_map.h"
#include "dynamic_astar_grid_planner/dynamic_astar_grid_planner.h"
#include "dynamic_footstep_planner/dynamic_footstep_planner.h"

namespace dynamic_planners
{

class DynamicRvizController
{
public:
  DynamicRvizController();

  void run();

private:
  ros::NodeHandle nh;
  ros::Subscriber subscriberCreateMapper;
  ros::Subscriber subscriberRunMapping;
  ros::Subscriber subscriberRobotPoses;
  ros::Subscriber subscriberResetRobotPose;
  ros::Subscriber subscriberSetGlobalMap;
  ros::Subscriber subscriberFindGridPlanLocal;
  ros::Subscriber subscriberInitializeFootstepPlanner;
  ros::Subscriber subscriberRunFootstepPlanning;
  ros::Subscriber subscriberFindGridPlanGlobal;
  ros::Subscriber subscriberDisplaySettingsChanged;
  ros::Subscriber subscriberRunFootstepExecutionChanged;
  ros::Subscriber subscriberFootstepExecutorBusy;
  ros::Publisher publisherResetRobotPose;
  ros::Publisher publisherJointTrajectory;
  ros::ServiceClient serviceClientGetHeightMap;
  ros::ServiceClient serviceClientFootstepExecution;
  ros::ServiceClient serviceClientStepToInitPose;

  tf::TransformListener transformListener;
  tf::TransformBroadcaster transformBroadcaster;

  std::list<std::pair<ros::WallTime, ros::Publisher*> > deleteMessageQueue;

  Matrix34 transformBaseToCamera;
  Pose6D robotPose;
  Pose6D leftFootPose;
  Pose6D rightFootPose;
  bool firstTransformSet;

  DynamicRvizVisualizer* dynamicRvizVisualizer;

  GlobalMap* globalMap;

  DynamicHeightMap* dynamicHeightMap;
  bool runMapping;

  DynamicAStarGridPlanner* dynamicAStarGridPlanner;
  bool runLocalGridMapping;

  DynamicFootstepPlanner* dynamicFootstepPlanner;
  bool runFootstepPlanning;

  Foot supportFoot;
  bool runFootstepExecution;
  bool footstepFinished;

  void initialize();

  void initializeParameters();

  void initializeMessageHandling();

  bool waitForRviz(const Real duration);

  void subscriberCreateMapperHandler(const std_msgs::Empty &msg);

  void subscriberRunMappingHandler(const std_msgs::Empty &msg);

  void subscriberRobotPosesHandler(const geometry_msgs::PoseArray &msg);

  void subscriberResetRobotPoseHandler(const std_msgs::Empty &msg);

  void subscriberSetGlobalMapHandler(const std_msgs::Empty &msg);

  void subscriberFindGridPlanGlobalHandler(const geometry_msgs::PointStamped &msg);

  void subscriberFindGridPlanLocalHandler(const std_msgs::Empty &msg);

  void subscriberInitializeFootstepPlannerHandler(const std_msgs::Empty &msg);

  void subscriberRunFootstepPlanningHandler(const std_msgs::Empty &msg);

  void subscriberDisplaySettingsChangedHandler(const std_msgs::Empty &msg);

  void subscriberRunFootstepExecutionChangedHandler(const std_msgs::Empty &msg);

  void subscriberFootstepExecutorBusyHandler(const std_msgs::Bool &msg);

  void publishZeroTrajectory();

  void publishRobotPose();

  void executeNextFootstep();

  void executeZeroFootstep();

  void waitFootstepStart();

  void waitFootstepFinish();
};

}

#endif //DYNAMIC_PLANNERS_DYNAMIC_RVIZ_CONTROLLER_H_
