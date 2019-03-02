#include <tools_std/ros_param_loader.h>

bool RosParamLoader::validParamLoads = true;

bool RosParamLoader::allParamLoadsValid()
{
  return validParamLoads;
}

bool RosParamLoader::checkParam(const ros::NodeHandle &nh, const std::string &name, const std::string &location)
{
  try
  {
    if (!nh.hasParam(name))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      return false;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to read ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    return false;
  }
  return true;
}

void RosParamLoader::getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, std::string &member, const std::string &memberDefault)
{
  member = memberDefault;
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      validParamLoads = false;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to find ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    validParamLoads = false;
  }
}

void RosParamLoader::getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, int &member, const int &memberDefault)
{
  member = memberDefault;
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      validParamLoads = false;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to find ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    validParamLoads = false;
  }
}

void RosParamLoader::getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, uint &member, const uint &memberDefault)
{
  member = memberDefault;
  try
  {
    int memberTmp;
    if (!nh.getParam(name, memberTmp))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      validParamLoads = false;
    }
    else
      member = (uint)memberTmp;
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to find ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    validParamLoads = false;
  }
}

void RosParamLoader::getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, double &member, const double &memberDefault)
{
  member = memberDefault;
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      validParamLoads = false;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to find ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    validParamLoads = false;
  }
}

void RosParamLoader::getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, bool &member, const bool &memberDefault)
{
  member = memberDefault;
  try
  {
    if (!nh.getParam(name, member))
    {
      ROS_ERROR("'%s' can't find ROS parameter '%s'.", location.c_str(), (nh.getNamespace() + "/" + name).c_str());
      validParamLoads = false;
    }
  }
  catch (ros::InvalidNameException &ex)
  {
    ROS_ERROR("'%s' tried to find ROS parameter '%s', but it has an invalid format.", location.c_str(),
              (nh.getNamespace() + "/" + name).c_str());
    validParamLoads = false;
  }
}
