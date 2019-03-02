#ifndef DYNAMIC_PLANNERS_PROGRESS_PUBLISHER_H_
#define DYNAMIC_PLANNERS_PROGRESS_PUBLISHER_H_

//std
#include <string>
#include <utility>
#include <vector>
#include <list>

//ros
#include <ros/ros.h>
#include <std_msgs/String.h>


class ProgressPublisher
{
public:
  static void initialize();

  static int addPublisher(const std::string &topic);

  static void showMessage(const int index, const std::string &message, const double time = -1);

  static void processDeleteMessageQueue();

private:
  static ros::NodeHandle* nh;

  static std_msgs::String msg;

  static std::vector<ros::Publisher> publishers;

  static std::list<std::pair<ros::WallTime, int> > deleteMessageQueue;
};


#endif // DYNAMIC_PLANNERS_PROGRESS_PUBLISHER_H_
