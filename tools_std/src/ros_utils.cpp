#include <tools_std/ros_utils.h>

void RosUtils::wait(double duration, bool spin)
{
  ros::WallTime timeStart = ros::WallTime::now();
  ros::WallDuration dur(duration);

  while ((ros::WallTime::now() - timeStart) < dur)
  {
    if (spin)
      ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
}
