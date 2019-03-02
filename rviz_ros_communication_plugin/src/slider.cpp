//plugin
#include <QIntValidator>
#include <QString>

//ros
#include <ros/ros.h>

//local
#include "rviz_ros_communication_plugin/slider.h"

namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// ABSTACT SLIDER //////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

int AbstractSlider::getLabelWidth()
{
  return label->fontMetrics().width(label->text());
}

void AbstractSlider::setLabelWidth(int width)
{
  label->setFixedWidth(width);
}

QHBoxLayout* AbstractSlider::getLayout()
{
  return layout;
}

//////////////////// PROTECTED ///////////////////////////////////

AbstractSlider::AbstractSlider(const std::string &labelString, const int minimum, const int maximum, const std::string &param, rviz::Panel* panel) :
    panel(panel), paramName(param)
{
  layout = new QHBoxLayout;

  label = new QLabel;
  label->setText(QString::fromStdString(labelString + ": "));
  layout->addWidget(label);

  line = new QLineEdit;
  line->setFixedWidth(70);
  layout->addWidget(line);

  slider = new QSlider;
  slider->setOrientation(Qt::Horizontal);
  slider->setRange(minimum, maximum);
  layout->addWidget(slider);

  connect(slider, SIGNAL(sliderMoved(int)), this, SLOT(onSliderValueChanged(int)));
  connect(slider, SIGNAL(sliderReleased()), this, SLOT(onSliderReleased()));
  connect(line, SIGNAL(editingFinished()), this, SLOT(onLineEditEditingFinished()));
}


//////////////////////////////////////////////////////////////////

//////////////////// INT SLIDER //////////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

IntSlider::IntSlider(const std::string &labelString, const int minimum, const int maximum, const std::string &param, rviz::Panel* panel) :
    AbstractSlider(labelString, minimum, maximum, param, panel)
{
  QIntValidator* validator = new QIntValidator(minimum, maximum);
  line->setValidator(validator);
  setLineEdit(slider->value());
}

void IntSlider::setValue(int value)
{
  currentValue = value;
  setLineEdit(currentValue);
  setSlider(currentValue);
  setRosParam();
}

int IntSlider::getValue()
{
  return currentValue;
}

//////////////////// PRIVATE /////////////////////////////////////

void IntSlider::onSliderValueChanged(int value)
{
  currentValue = value;
  setLineEdit(currentValue);
  Q_EMIT panel->configChanged();
}

void IntSlider::onSliderReleased()
{
  setRosParam();
}

void IntSlider::onLineEditEditingFinished()
{
  currentValue = line->text().toInt();
  setSlider(currentValue);
  setRosParam();
  Q_EMIT panel->configChanged();
}

void IntSlider::setLineEdit(const int value)
{
  QString str;
  str.setNum(value);

  line->blockSignals(true);
  line->setText(str);
  line->blockSignals(false);
}

void IntSlider::setSlider(const int value)
{
  slider->blockSignals(true);
  slider->setValue(value);
  slider->blockSignals(false);
}

void IntSlider::setRosParam()
{
  ros::param::set(paramName, getValue());
}


//////////////////////////////////////////////////////////////////

//////////////////// DOUBLE SLIDER ///////////////////////////////

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

DoubleSlider::DoubleSlider(const std::string &labelString, const double minimum, const double maximum, const std::string &param, rviz::Panel* panel) :
    AbstractSlider(labelString, minimum / DOUBLE_SLIDER_FACTOR, maximum / DOUBLE_SLIDER_FACTOR, param, panel)
{
  setLineEdit(slider->value() * DOUBLE_SLIDER_FACTOR);
}

void DoubleSlider::setValue(double value)
{
  currentValue = value;
  setLineEdit(currentValue);
  setSlider(currentValue);
  setRosParam();
}

double DoubleSlider::getValue()
{
  return currentValue;
}

//////////////////// PRIVATE /////////////////////////////////////

void DoubleSlider::onSliderValueChanged(int value)
{
  currentValue = value* DOUBLE_SLIDER_FACTOR;
  setLineEdit(currentValue);
  Q_EMIT panel->configChanged();
}

void DoubleSlider::onSliderReleased()
{
  setRosParam();
}

void DoubleSlider::onLineEditEditingFinished()
{
  QString str = line->text();
  bool state;
  double value = str.toDouble(&state);

  if(state)
  {
    if(value < slider->minimum() * DOUBLE_SLIDER_FACTOR)
      value = slider->minimum() * DOUBLE_SLIDER_FACTOR;
    else if(value > slider->maximum() * DOUBLE_SLIDER_FACTOR)
      value = slider->maximum() * DOUBLE_SLIDER_FACTOR;

    currentValue = value;
    setSlider(value);
  }
  else
    currentValue = slider->value() * DOUBLE_SLIDER_FACTOR;

  setLineEdit(currentValue);
  setRosParam();
  Q_EMIT panel->configChanged();
}

void DoubleSlider::setLineEdit(const double value)
{
  QString str;
  str.setNum(value, 'f', 4);

  line->blockSignals(true);
  line->setText(str);
  line->blockSignals(false);
}

void DoubleSlider::setSlider(const double value)
{
  slider->blockSignals(true);
  slider->setValue(value / DOUBLE_SLIDER_FACTOR);
  slider->blockSignals(false);
}

void DoubleSlider::setRosParam()
{
  ros::param::set(paramName, getValue());
}

} // namespace rviz_ros_communication_plugin

