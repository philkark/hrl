//local
#include "common/progress_publisher.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

void ProgressPublisher::initialize()
{
  nh = new ros::NodeHandle();
}

int ProgressPublisher::addPublisher(const std::string &topic)
{
  if (nh == nullptr)
    return -1;

  publishers.push_back(ros::Publisher());
  publishers.back() = nh->advertise<std_msgs::String>(topic, 20);

  ros::WallTime timeNow = ros::WallTime::now();

  while (publishers.back().getNumSubscribers() == 0 && (ros::WallTime::now() - timeNow).toSec() < 5.0)
  {
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }

  return publishers.size() - 1;
}

void ProgressPublisher::showMessage(const int index, const std::string &message, const double time)
{
  msg.data = message;
  publishers[index].publish(msg);
  if(time > 0.0)
    deleteMessageQueue.push_back(std::make_pair(ros::WallTime::now() + ros::WallDuration(time), index));
  ros::spinOnce();
}

void ProgressPublisher::processDeleteMessageQueue()
{
  const ros::WallTime timeNow = ros::WallTime::now();
  for (std::list<std::pair<ros::WallTime, int> >::iterator it = deleteMessageQueue.begin(); it != deleteMessageQueue.end();)
  {
    if (timeNow > it->first)
    {
      publishers[it->second].publish(std_msgs::String());
      it = deleteMessageQueue.erase(it);
    }
    else
      ++it;
  }

  ros::spinOnce();
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

ros::NodeHandle* ProgressPublisher::nh = nullptr;

std_msgs::String ProgressPublisher::msg = std_msgs::String();

std::vector<ros::Publisher> ProgressPublisher::publishers = std::vector<ros::Publisher>();

std::list<std::pair<ros::WallTime, int> > ProgressPublisher::deleteMessageQueue = std::list<std::pair<ros::WallTime, int> >();
