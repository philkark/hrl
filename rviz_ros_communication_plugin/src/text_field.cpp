//ros
#include <ros/ros.h>

//local
#include "rviz_ros_communication_plugin/text_field.h"


namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

TextField::TextField(const std::string &labelString, const std::string &param, rviz::Panel* panel) :
    panel(panel), paramName(param)
{
  layout = new QHBoxLayout;

  label = new QLabel;
  label->setText(QString::fromStdString(labelString + ": "));
  layout->addWidget(label);

  line = new QLineEdit;
  layout->addWidget(line);

  connect(line, SIGNAL(editingFinished()), this, SLOT(onLineEditEditingFinished()));
}

QString TextField::getContent()
{
  return line->text();
}

void TextField::setContent(const QString &content)
{
  line->setText(content);
  setRosParam();
}

QHBoxLayout* TextField::getLayout()
{
  return layout;
}

int TextField::getLabelWidth()
{
  return label->fontMetrics().width(label->text());
}

void TextField::setLabelWidth(int width)
{
  label->setFixedWidth(width);
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void TextField::setRosParam()
{
  ros::param::set(paramName, line->text().toStdString());
}

void TextField::onLineEditEditingFinished()
{
  setRosParam();
  Q_EMIT panel->configChanged();
}

} // namespace rviz_ros_communication_plugin
