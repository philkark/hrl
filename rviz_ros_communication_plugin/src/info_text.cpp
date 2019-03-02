//ros
#include <ros/ros.h>

//local
#include "rviz_ros_communication_plugin/info_text.h"

namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

InfoText::InfoText(const std::string &topic)
{
  label = new QLabel;
  label->setAlignment(Qt::AlignLeft);
  label->setMaximumWidth(390);
  label->setWordWrap(true);

  subscriber = nh.subscribe(topic, 0, &InfoText::subscriberHandler, this);
}

QLabel* InfoText::getLabel()
{
  return label;
}


//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void InfoText::subscriberHandler(const std_msgs::String &msg)
{
  label->setText(QString::fromStdString(msg.data));
}

} // namespace rviz_ros_communication_plugin
