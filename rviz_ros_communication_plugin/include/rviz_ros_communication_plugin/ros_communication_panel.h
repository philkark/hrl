#ifndef RVIZ_ROS_COMMUNICATION_PLUGIN_ROS_COMMUNICATION_PANEL_H_
#define RVIZ_ROS_COMMUNICATION_PLUGIN_ROS_COMMUNICATION_PANEL_H_

//std
#include <vector>
#include <string>
#include <iostream>

//plugin
#include <rviz/panel.h>
#include <QWidget>
#include <QTabWidget>

//local
#include "rviz_ros_communication_plugin/category.h"

namespace rviz_ros_communication_plugin
{

class RosCommunicationPanel : public rviz::Panel
{
public:
  RosCommunicationPanel(QWidget* parent = nullptr);

  ~RosCommunicationPanel();

protected:
  QTabWidget* tabWidget;
  std::vector<Category> categories;

  void load(const rviz::Config& config);

  void save(rviz::Config config) const;

  Category* addCategory(const std::string &name);

  void constructCategories();

  void constructCategoryWorld();

  void constructCategoryMapping();

  void constructCategoryGridPlanning();

  void constructCategoryFootstepPlanning();

  void constructCategoryVisualization();

  void constructCategoryWalkingControl();

  void constructCategoryTesting();
};

} // namespace rviz_ros_communication_plugin

#endif // RVIZ_ROS_COMMUNICATION_PLUGIN_ROS_COMMUNICATION_PANEL_H_
