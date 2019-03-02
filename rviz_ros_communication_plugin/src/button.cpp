//ros
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

//local
#include "rviz_ros_communication_plugin/button.h"

namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// ABSTRACT BUTTON /////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

QPushButton* AbstractButton::getButton() const
{
  return button;
}

//////////////////// PROTECTED ///////////////////////////////////

AbstractButton::AbstractButton(const std::string &content, const std::string &topic, rviz::Panel* panel) :
    panel(panel)
{
  button = new QPushButton;
  button->setText(QString::fromStdString(content));

  if (!topic.empty())
    publisher = nh.advertise<std_msgs::Empty>(topic, 1);
  connect(button, SIGNAL(clicked()), this, SLOT(onClicked()));
}

//////////////////////////////////////////////////////////////////

//////////////////// SIGNAL BUTTON ///////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

SignalButton::SignalButton(const std::string &content, const std::string &topic) :
    AbstractButton(content, topic, nullptr)
{
}

AbstractButton::Type SignalButton::getType() const
{
  return AbstractButton::Signal;
}

//////////////////// PRIVATE /////////////////////////////////////

void SignalButton::onClicked()
{
  publisher.publish(std_msgs::Empty());
}

//////////////////////////////////////////////////////////////////

//////////////////// TOGGLE BUTTON ///////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

ToggleButton::ToggleButton(const std::string &contentUntoggled, const std::string &contentToggled, const std::string &topic, const std::string &param) :
    AbstractButton(contentUntoggled, topic, nullptr)
{
  textUntoggled = QString::fromStdString(contentUntoggled);
  textToggled = QString::fromStdString(contentToggled);
  button->setCheckable(true);
  paramName = param;
  setRosParam();
}

AbstractButton::Type ToggleButton::getType() const
{
  return AbstractButton::Toggle;
}

bool ToggleButton::getState()
{
  return button->isChecked();
}

//////////////////// PRIVATE /////////////////////////////////////

void ToggleButton::onClicked()
{
  updateText();
  setRosParam();
}

void ToggleButton::setRosParam()
{
  if (button->isChecked())
    ros::param::set(paramName, true);
  else
    ros::param::set(paramName, false);

  if (!publisher.getTopic().empty())
    publisher.publish(std_msgs::Empty());
}

void ToggleButton::updateText()
{
  if (button->isChecked())
    button->setText(textToggled);
  else
    button->setText(textUntoggled);
}

//////////////////////////////////////////////////////////////////

//////////////////// MENU BUTTON /////////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////
MenuButton::MenuButton(const std::string &topic, const std::string &param, rviz::Panel* panel) :
    AbstractButton("", topic, panel)
{
  menu = new QMenu;

  connect(menu, SIGNAL(triggered(QAction*)), this, SLOT(onMenuSelect(QAction*)));
  button->setMenu(menu);
  paramName = param;
}

AbstractButton::Type MenuButton::getType() const
{
  return AbstractButton::Menu;
}

void MenuButton::addMenuItem(const std::string &name, const std::string &value)
{
  QAction* action;
  action = new QAction(QString::fromStdString(name), menu);
  action->setData(QVariant::fromValue(names.size()));
  menu->addAction(action);

  names.push_back(name);
  values.push_back(value);
}

void MenuButton::setIndex(int index)
{
  currentIndex = index;
  updateText();
  setRosParam();
}

int MenuButton::getIndex()
{
  return currentIndex;
}

//////////////////// PRIVATE /////////////////////////////////////

void MenuButton::onClicked()
{

}

void MenuButton::onMenuSelect(QAction* action)
{
  setIndex(action->data().toInt());
  Q_EMIT panel->configChanged();
}

void MenuButton::setRosParam()
{
  ros::param::set(paramName, values[currentIndex]);
  publisher.publish(std_msgs::Empty());
}

void MenuButton::updateText()
{
  button->setText(QString::fromStdString(names[currentIndex]));
}

} // namespace rviz_ros_communication_plugin
