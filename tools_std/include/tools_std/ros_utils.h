#ifndef TOOLS_STD_ROS_UTILS_H_
#define TOOLS_STD_ROS_UTILS_H_

#include <ros/ros.h>
#include <ros/package.h>

class RosUtils
{
public:
  static void wait(double duration, bool spin);


};

#endif // TOOLS_STD_ROS_UTILS_H_
