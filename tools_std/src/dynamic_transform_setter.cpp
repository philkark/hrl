#include <tools_std/dynamic_transform_setter.h>

DynamicTransformSetter::DynamicTransformSetter() :
  xInit(0.0), yInit(0.0), zInit(0.0), rollInit(0.0), pitchInit(0.0), yawInit(0.0), nh("transform_setter"), DRServer(nh)
{
  firstCallback = true;
  matrix.set(0.0);
  DRServer.setCallback(boost::bind(&DynamicTransformSetter::transformCallback, this, _1, _2));
}

DynamicTransformSetter::DynamicTransformSetter(const Real x, const Real y, const Real z, const Real roll, const Real pitch, const Real yaw) :
    xInit(x), yInit(y), zInit(z), rollInit(roll), pitchInit(pitch), yawInit(yaw), nh("transform_setter"), DRServer(nh)
{
  firstCallback = true;
  matrix.set(0.0);
  DRServer.setCallback(boost::bind(&DynamicTransformSetter::transformCallback, this, _1, _2));
}

const Matrix34 &DynamicTransformSetter::getMatrix() const
{
  return matrix;
}

const tf::Transform &DynamicTransformSetter::getTransform() const
{
  return transform;
}

const void DynamicTransformSetter::setCurrentTransform(const std::string &frame, const std::string &frameChild)
{
  static tf::TransformBroadcaster tfBroadcaster;
  tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame, frameChild));
}

void DynamicTransformSetter::transformCallback(tools_std::transform6dConfig &config, UInt level)
{
  if (firstCallback)
  {
    firstCallback = false;

    transform.setOrigin(tf::Vector3(xInit, yInit, zInit));

    tf::Matrix3x3 matrixTF;
    matrixTF.setRPY(rollInit, pitchInit, yawInit);
    transform.setBasis(matrixTF);

    matrix.C03 = xInit;
    matrix.C13 = yInit;
    matrix.C23 = zInit;

    matrix.C00 = matrixTF.getRow(0).x();
    matrix.C01 = matrixTF.getRow(0).y();
    matrix.C02 = matrixTF.getRow(0).z();

    matrix.C10 = matrixTF.getRow(1).x();
    matrix.C11 = matrixTF.getRow(1).y();
    matrix.C12 = matrixTF.getRow(1).z();

    matrix.C20 = matrixTF.getRow(2).x();
    matrix.C21 = matrixTF.getRow(2).y();
    matrix.C22 = matrixTF.getRow(2).z();
    config.x = xInit;
    config.y = yInit;
    config.z = zInit;
    config.roll = rollInit;
    config.pitch = pitchInit;
    config.yaw = yawInit;
    return;
  }

  transform.setOrigin(tf::Vector3(config.x, config.y, config.z));

  tf::Matrix3x3 matrixTF;
  matrixTF.setRPY(config.roll, config.pitch, config.yaw);
  transform.setBasis(matrixTF);

  matrix.C03 = config.x;
  matrix.C13 = config.y;
  matrix.C23 = config.z;

  matrix.C00 = matrixTF.getRow(0).x();
  matrix.C01 = matrixTF.getRow(0).y();
  matrix.C02 = matrixTF.getRow(0).z();

  matrix.C10 = matrixTF.getRow(1).x();
  matrix.C11 = matrixTF.getRow(1).y();
  matrix.C12 = matrixTF.getRow(1).z();

  matrix.C20 = matrixTF.getRow(2).x();
  matrix.C21 = matrixTF.getRow(2).y();
  matrix.C22 = matrixTF.getRow(2).z();
}
