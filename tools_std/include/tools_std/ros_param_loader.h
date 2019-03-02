#ifndef TOOLS_STD_ROS_PARAM_LOADER_H
#define TOOLS_STD_ROS_PARAM_LOADER_H

#include <ros/ros.h>
#include <ros/package.h>

class RosParamLoader
{
private:
  static bool validParamLoads;

public:
  static bool allParamLoadsValid();

  static bool checkParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name);

  static void getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, std::string &member, const std::string &memberDefault="");

  static void getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, int &member, const int &memberDefault=0);

  static void getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, uint &member, const uint &memberDefault=0);

  static void getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, double &member, const double &memberDefault=0.0);

  static void getRosParam(const ros::NodeHandle &nh, const std::string &location, const std::string &name, bool &member, const bool &memberDefault=false);
};

#endif //TOOLS_STD_ROS_PARAM_LOADER_H
