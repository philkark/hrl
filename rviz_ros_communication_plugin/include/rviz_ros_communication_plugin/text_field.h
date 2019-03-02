#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_TEXT_FIELD_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_TEXT_FIELD_H_

//std
#include <string>

//plugin
#include <rviz/panel.h>
#include <QObject>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>

namespace rviz_ros_communication_plugin
{

class TextField : public QObject
{
Q_OBJECT
public:
  TextField(const std::string &labelString, const std::string &param, rviz::Panel* panel);

  QString getContent();

  void setContent(const QString &content);

  QHBoxLayout* getLayout();

  int getLabelWidth();

  void setLabelWidth(int width);

private:
  std::string paramName;
  rviz::Panel* panel;
  QHBoxLayout* layout;
  QLabel* label;
  QLineEdit* line;

  void setRosParam();

private Q_SLOTS:
  void onLineEditEditingFinished();
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_TEXT_FIELD_H_
