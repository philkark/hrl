#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_INFO_TEXT_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_INFO_TEXT_H_

//std
#include <string>

//plugin
#include <rviz/panel.h>
#include <QObject>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

//ros
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace rviz_ros_communication_plugin
{

class InfoText : public QObject
{
Q_OBJECT
public:
  InfoText(const std::string &topic);

  QLabel* getLabel();


private:
  QLabel* label;

  ros::NodeHandle nh;
  ros::Subscriber subscriber;

  void subscriberHandler(const std_msgs::String &msg);
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_INFO_TEXT_FIELD_H_
