#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_BUTTON_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_BUTTON_H_

//std
#include <string>
#include <vector>

//plugin
#include <QObject>
#include <QPushButton>
#include <QMenu>

//ros
#include <ros/ros.h>
#include <rviz/panel.h>

namespace rviz_ros_communication_plugin
{

class AbstractButton : public QObject
{
Q_OBJECT
public:
  enum Type
  {
    Signal, Toggle, Menu
  };

  QPushButton* getButton() const;

  virtual Type getType() const = 0;

protected:
  ros::NodeHandle nh;
  ros::Publisher publisher;

  rviz::Panel* panel;
  QPushButton* button;

  AbstractButton(const std::string &content, const std::string &topic, rviz::Panel* panel);

protected Q_SLOTS:

  virtual void onClicked() = 0;
};

class SignalButton : public AbstractButton
{
public:
  SignalButton(const std::string &content, const std::string &topic);

  Type getType() const;

private:
  void onClicked();
};

class ToggleButton : public AbstractButton
{
public:
  ToggleButton(const std::string &contentUntoggled, const std::string &contentToggled, const std::string &topic, const std::string &param);

  Type getType() const;

  bool getState();

private:
  std::string paramName;
  QString textUntoggled;
  QString textToggled;

  void onClicked();

  void setRosParam();

  void updateText();
};

class MenuButton : public AbstractButton
{
Q_OBJECT
public:
  MenuButton(const std::string &topic, const std::string &param, rviz::Panel* panel);

  Type getType() const;

  void addMenuItem(const std::string &name, const std::string &value);

  void setIndex(int index);

  int getIndex();

private:
  std::string paramName;

  int currentIndex;

  QMenu* menu;

  std::vector<std::string> names;
  std::vector<std::string> values;

  void onClicked();

  void setRosParam();

  void updateText();

private Q_SLOTS:

  void onMenuSelect(QAction *action);
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_BUTTON_H_
