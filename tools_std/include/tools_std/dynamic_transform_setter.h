#ifndef TOOLS_STD_DYNAMIC_TRANSFORM_SETTER_H_
#define TOOLS_STD_DYNAMIC_TRANSFORM_SETTER_H_

#include <math_std/matrix.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <tools_std/transform6dConfig.h>

class DynamicTransformSetter
{
public:
  DynamicTransformSetter();

  DynamicTransformSetter(const Real x, const Real y, const Real z, const Real roll, const Real pitch, const Real yaw);

  const Matrix34 &getMatrix() const;

  const tf::Transform &getTransform() const;

  const void setCurrentTransform(const std::string &frame, const std::string &frameChild);

private:
  ros::NodeHandle nh;
  tf::Transform transform;
  Matrix34 matrix;
  dynamic_reconfigure::Server<tools_std::transform6dConfig> DRServer;

  const Real xInit;
  const Real yInit;
  const Real zInit;
  const Real rollInit;
  const Real pitchInit;
  const Real yawInit;


  bool firstCallback;

  void transformCallback(tools_std::transform6dConfig &config, UInt level);
};

#endif // TOOLS_STD_DYNAMIC_TRANSFORM_SETTER_H_
