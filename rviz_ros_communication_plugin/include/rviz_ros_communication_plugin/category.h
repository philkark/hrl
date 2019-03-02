#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_CATEGORY_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_CATEGORY_H_

//std
#include <string>
#include <vector>

//plugin
#include <QScrollArea>
#include <QVBoxLayout>
#include <QString>

//local
#include "rviz_ros_communication_plugin/slider.h"
#include "rviz_ros_communication_plugin/info_text.h"
#include "rviz_ros_communication_plugin/text_field.h"
#include "rviz_ros_communication_plugin/button.h"

namespace rviz_ros_communication_plugin
{

class Category
{
public:
  Category(const std::string &name);

  QString getName() const;

  QScrollArea* getWidget() const;

  void addHeadLine(const std::string &name, const int fontSize = 14);

  void addInfoText(const std::string &topic);

  void addSlider(DoubleSlider* slider);

  void addSlider(IntSlider* slider);

  void addTextField(TextField* textField);

  void addButtons(const std::vector<AbstractButton*> &buttons);

  void finalizeLayout();

  int getIntSliderSettingsCount() const;

  std::vector<int> getIntSliderSettings() const;

  void setIntSliderSettings(const std::vector<int> &settings);

  int getDoubleSliderSettingsCount() const;

  std::vector<double> getDoubleSliderSettings() const;

  void setDoubleSliderSettings(const std::vector<double> &settings);

  int getTextFieldSettingsCount() const;

  std::vector<QString> getTextFieldSettings() const;

  void setTextFieldSettings(const std::vector<QString> &settings);

  int getMenuButtonSettingsCount() const;

  std::vector<int> getMenuButtonSettings() const;

  void setMenuButtonSettings(const std::vector<int> &settings);

private:
  std::string name;
  QScrollArea* scrollArea;
  QVBoxLayout* layout;

  std::vector<IntSlider*> intSliders;
  std::vector<DoubleSlider*> doubleSliders;
  std::vector<TextField*> textFields;
  std::vector<ToggleButton*> toggleButtons;
  std::vector<MenuButton*> menuButtons;
  std::vector<InfoText*> infoTexts;

  void adjustLabelWidths();
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_CATEGORY_H_
