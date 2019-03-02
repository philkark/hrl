#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace gazebo
{
class GazeboPluginReemcRvizSimulation : public ModelPlugin
{
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    this->model = model;
    if (!sdf->HasElement("updateRate"))
    {
      std::cerr << "GazeboPluginReemcRvizSimulation missing <updateRate> sdf element." << std::endl;
      return;
    }
    else
      duration = ros::Duration(1.0 / sdf->GetElement("updateRate")->Get<double>());

    cameraLink = model->GetLink("rgbd_camera_link");
    baseLink = model->GetLink("base_link");
    leftFootLink = model->GetLink("left_sole_link");
    rightFootLink = model->GetLink("right_sole_link");

    if (!sdf->HasElement("topicName"))
    {
      std::cerr << "GazeboPluginReemcRvizSimulation missing <topicName> sdf element." << std::endl;
      return;
    }
    else
      publisherPoses = nh.advertise<geometry_msgs::PoseArray>(sdf->GetElement("topicName")->Get<std::string>(), 1);

    lastPublishTime = ros::Time::now();
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboPluginReemcRvizSimulation::onUpdate, this));
    subscriberResetWorldPose = nh.subscribe("reset_robot_pose", 1, &GazeboPluginReemcRvizSimulation::subscriberResetWorldPoseHandler, this);
  }

  void onUpdate()
  {
    const ros::Time timeNow = ros::Time::now();

    if (timeNow - lastPublishTime >= duration)
    {
      lastPublishTime = timeNow;
      publishPoses();
    }
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr cameraLink;
  physics::LinkPtr baseLink;
  physics::LinkPtr leftFootLink;
  physics::LinkPtr rightFootLink;
  event::ConnectionPtr updateConnection;

  ros::Publisher publisherPoses;
  ros::Subscriber subscriberResetWorldPose;
  ros::NodeHandle nh;

  std::string linkName;
  ros::Duration duration;

  ros::Time lastPublishTime;

  void publishPoses() const
  {
    math::Pose poseCamera = cameraLink->GetWorldPose();
    math::Pose poseBase = baseLink->GetWorldPose();
    math::Pose poseLeftFoot = leftFootLink->GetWorldPose();
    math::Pose poseRightFoot = rightFootLink->GetWorldPose();
    math::Pose poseBaseToCamera = poseCamera - poseBase;
    math::Pose poseBaseToLeftFoot = poseLeftFoot - poseBase;
    math::Pose poseBaseToRightFoot = poseRightFoot - poseBase;

    geometry_msgs::PoseArray msg;
    msg.poses.resize(4);

    msg.poses[0].position.x = poseBase.pos.x;
    msg.poses[0].position.y = poseBase.pos.y;
    msg.poses[0].position.z = poseBase.pos.z;
    msg.poses[0].orientation.x = poseBase.rot.x;
    msg.poses[0].orientation.y = poseBase.rot.y;
    msg.poses[0].orientation.z = poseBase.rot.z;
    msg.poses[0].orientation.w = poseBase.rot.w;

    msg.poses[1].position.x = poseBaseToCamera.pos.x;
    msg.poses[1].position.y = poseBaseToCamera.pos.y;
    msg.poses[1].position.z = poseBaseToCamera.pos.z;
    msg.poses[1].orientation.x = poseBaseToCamera.rot.x;
    msg.poses[1].orientation.y = poseBaseToCamera.rot.y;
    msg.poses[1].orientation.z = poseBaseToCamera.rot.z;
    msg.poses[1].orientation.w = poseBaseToCamera.rot.w;

    msg.poses[2].position.x = poseBaseToLeftFoot.pos.x;
    msg.poses[2].position.y = poseBaseToLeftFoot.pos.y;
    msg.poses[2].position.z = poseBaseToLeftFoot.pos.z;
    msg.poses[2].orientation.x = poseBaseToLeftFoot.rot.x;
    msg.poses[2].orientation.y = poseBaseToLeftFoot.rot.y;
    msg.poses[2].orientation.z = poseBaseToLeftFoot.rot.z;
    msg.poses[2].orientation.w = poseBaseToLeftFoot.rot.w;

    msg.poses[3].position.x = poseBaseToRightFoot.pos.x;
    msg.poses[3].position.y = poseBaseToRightFoot.pos.y;
    msg.poses[3].position.z = poseBaseToRightFoot.pos.z;
    msg.poses[3].orientation.x = poseBaseToRightFoot.rot.x;
    msg.poses[3].orientation.y = poseBaseToRightFoot.rot.y;
    msg.poses[3].orientation.z = poseBaseToRightFoot.rot.z;
    msg.poses[3].orientation.w = poseBaseToRightFoot.rot.w;

    publisherPoses.publish(msg);
  }

  void subscriberResetWorldPoseHandler(const geometry_msgs::Pose &msg)
  {
    math::Pose pose;
    pose.pos.x = msg.position.x;
    pose.pos.y = msg.position.y;
    pose.pos.z = msg.position.z;
    pose.rot.x = msg.orientation.x;
    pose.rot.y = msg.orientation.y;
    pose.rot.z = msg.orientation.z;
    pose.rot.w = msg.orientation.w;

    physics::Joint_V joints = model->GetJoints();

    for (int i = 0; i < joints.size(); ++i)
    {
      int angleCount = joints[i]->GetAngleCount();
      for (int j = 0; j < angleCount; ++j)
        joints[i]->SetPosition(j, 0.0);
    }

    model->SetWorldPose(pose);
  }
};

GZ_REGISTER_MODEL_PLUGIN(GazeboPluginReemcRvizSimulation)
}
