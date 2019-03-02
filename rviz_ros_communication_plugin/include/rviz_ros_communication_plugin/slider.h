#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_SLIDER_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_SLIDER_H_

//std
#include <string>

//plugin
#include <rviz/panel.h>
#include <QObject>
#include <QLabel>
#include <QSlider>
#include <QLineEdit>
#include <QHBoxLayout>


namespace rviz_ros_communication_plugin
{

#define DOUBLE_SLIDER_FACTOR 0.000001

class AbstractSlider : public QObject
{
Q_OBJECT
public:
  QHBoxLayout* getLayout();

  int getLabelWidth();

  void setLabelWidth(int width);

protected:
  std::string paramName;
  rviz::Panel* panel;
  QHBoxLayout* layout;
  QLabel* label;
  QSlider* slider;
  QLineEdit* line;

  AbstractSlider(const std::string &labelString, const int minimum, const int maximum, const std::string &param, rviz::Panel* panel);

  virtual void setRosParam() = 0;

protected Q_SLOTS:
  virtual void onSliderValueChanged(int value) = 0;

  virtual void onSliderReleased() = 0;

  virtual void onLineEditEditingFinished() = 0;
};

class IntSlider : public AbstractSlider
{
public:
  IntSlider(const std::string &labelString, const int minimum, const int maximum, const std::string &param, rviz::Panel* panel);

  void setValue(int value);

  int getValue();

private:
  int currentValue;

  void onSliderValueChanged(int value);

  void onSliderReleased();

  void onLineEditEditingFinished();

  void setLineEdit(const int value);

  void setSlider(const int value);

  void setRosParam();
};

class DoubleSlider : public AbstractSlider
{
public:
  DoubleSlider(const std::string &labelString, const double minimum, const double maximum, const std::string &param, rviz::Panel* panel);

  void setValue(double value);

  double getValue();

private:
  double currentValue;

  void onSliderValueChanged(int value);

  void onSliderReleased();

  void onLineEditEditingFinished();

  void setLineEdit(const double value);

  void setSlider(const double value);

  void setRosParam();
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_SLIDER_H_
