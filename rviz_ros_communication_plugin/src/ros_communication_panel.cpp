//std
#include <string>

//plugin
#include <QVBoxLayout>
#include <QLabel>
#include <QString>

//ros
#include <ros/ros.h>

//local
#include "rviz_ros_communication_plugin/ros_communication_panel.h"
#include "rviz_ros_communication_plugin/slider.h"
#include "rviz_ros_communication_plugin/button.h"

namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

RosCommunicationPanel::RosCommunicationPanel(QWidget* parent) :
    rviz::Panel(parent)
{
  tabWidget = new QTabWidget;

  constructCategories();

  tabWidget->setMinimumWidth(400);
  tabWidget->setMaximumWidth(400);
  setLayout(new QVBoxLayout);
  this->layout()->addWidget(tabWidget);
}

RosCommunicationPanel::~RosCommunicationPanel()
{
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void RosCommunicationPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  for (int i = 0; i < categories.size(); ++i)
  {
    const Category &category = categories[i];

    std::vector<int> intSettings = category.getIntSliderSettings();
    std::vector<double> doubleSettings = category.getDoubleSliderSettings();
    std::vector<QString> textFieldSettings = category.getTextFieldSettings();
    std::vector<int> menuButtonSettings = category.getMenuButtonSettings();

    int index = 0;
    for (int j = 0; j < intSettings.size(); ++j, ++index)
    {
      config.mapSetValue(category.getName() + QString::number(index), intSettings[j]);
    }
    for (int j = 0; j < doubleSettings.size(); ++j, ++index)
      config.mapSetValue(category.getName() + QString::number(index), doubleSettings[j]);
    for (int j = 0; j < textFieldSettings.size(); ++j, ++index)
      config.mapSetValue(category.getName() + QString::number(index), textFieldSettings[j]);
    for (int j = 0; j < menuButtonSettings.size(); ++j, ++index)
      config.mapSetValue(category.getName() + QString::number(index), menuButtonSettings[j]);
  }
}

void RosCommunicationPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);

  for (int i = 0; i < categories.size(); ++i)
  {
    Category &category = categories[i];

    std::vector<int> intSettings(category.getIntSliderSettingsCount());
    std::vector<double> doubleSettings(category.getDoubleSliderSettingsCount());
    std::vector<QString> textFieldSettings(category.getTextFieldSettingsCount());
    std::vector<int> menuButtonSettings(category.getMenuButtonSettingsCount());

    int index = 0;
    for (int j = 0; j < intSettings.size(); ++j, ++index)
    {
      config.mapGetInt(category.getName() + QString::number(index), &intSettings[j]);
    }

    for (int j = 0; j < doubleSettings.size(); ++j, ++index)
    {
      float value;
      config.mapGetFloat(category.getName() + QString::number(index), &value);
      doubleSettings[j] = (double)value;
    }
    for (int j = 0; j < textFieldSettings.size(); ++j, ++index)
    {
      config.mapGetString(category.getName() + QString::number(index), &textFieldSettings[j]);
    }
    for (int j = 0; j < menuButtonSettings.size(); ++j, ++index)
    {
      config.mapGetInt(category.getName() + QString::number(index), &menuButtonSettings[j]);
    }

    category.setIntSliderSettings(intSettings);
    category.setDoubleSliderSettings(doubleSettings);
    category.setTextFieldSettings(textFieldSettings);
    category.setMenuButtonSettings(menuButtonSettings);
  }
}

Category* RosCommunicationPanel::addCategory(const std::string &name)
{
  categories.push_back(Category(name));
  Category* category = &categories.back();
  tabWidget->addTab(category->getWidget(), category->getName());
  return category;
}

void RosCommunicationPanel::constructCategories()
{
  constructCategoryWorld();
  constructCategoryMapping();
  constructCategoryGridPlanning();
  constructCategoryFootstepPlanning();
  constructCategoryVisualization();
  constructCategoryWalkingControl();
}

void RosCommunicationPanel::constructCategoryWorld()
{
  Category* category = addCategory("World");

  category->addHeadLine("Robot");
  category->addSlider(new DoubleSlider("X", -100, 100, "set_robot_x", this));
  category->addSlider(new DoubleSlider("Y", -100, 100, "set_robot_y", this));
  category->addSlider(new DoubleSlider("Z", -100, 100, "set_robot_z", this));
  category->addSlider(new DoubleSlider("Roll", -M_PI, M_PI, "set_robot_roll", this));
  category->addSlider(new DoubleSlider("Pitch", -M_PI, M_PI, "set_robot_pitch", this));
  category->addSlider(new DoubleSlider("Yaw", -M_PI, M_PI, "set_robot_yaw", this));
  category->addSlider(new DoubleSlider("Movement Speed", 0.001, 100, "robot_movement_speed", this));

  std::vector<AbstractButton*> buttons;
  buttons.push_back(new SignalButton("Reset Robot Pose", "reset_robot_pose_signal"));
  category->addButtons(buttons);

  category->addHeadLine("Global Map");
  category->addSlider(new DoubleSlider("Min X", -100, 100, "global_map_min_x", this));
  category->addSlider(new DoubleSlider("Max X", -100, 100, "global_map_max_x", this));
  category->addSlider(new DoubleSlider("Min Y", -100, 100, "global_map_min_y", this));
  category->addSlider(new DoubleSlider("Max Y", -100, 100, "global_map_max_y", this));
  category->addSlider(new DoubleSlider("Resolution", 0.001, 1.0, "global_map_resolution", this));
  category->addSlider(new DoubleSlider("Safety Distance", 0.0, 3.0, "global_map_safety_distance", this));
  buttons.clear();

  MenuButton* menuButton = new MenuButton("global_map_sizing_type_changed", "global_map_sizing_type", this);
  menuButton->addMenuItem("Auto Size", "auto");
  menuButton->addMenuItem("Fixed Size", "fixed");
  buttons.push_back(menuButton);
  buttons.push_back(new SignalButton("Set Global Map", "set_global_map"));
  category->addButtons(buttons);

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryMapping()
{
  Category* category = addCategory("Mapping");
  std::vector<AbstractButton*> buttons;

  category->addHeadLine("Camera Settings");
  category->addSlider(new IntSlider("Width Resolution", 160, 640, "camera_width", this));
  category->addSlider(new IntSlider("Height Resolution", 120, 480, "camera_height", this));
  category->addSlider(new IntSlider("FPS", 15, 60, "camera_fps", this));
  category->addSlider(new DoubleSlider("Width Angle", 1.0, 100.0, "camera_opening_angle_width", this));
  category->addSlider(new DoubleSlider("Height Angle", 1.0, 100.0, "camera_opening_angle_height", this));
  category->addTextField(new TextField("Camera Topic", "camera_topic", this));
  buttons.clear();
  buttons.push_back(new ToggleButton("Use Only Global Map", "Use Camera Data", "", "only_global_map"));
  category->addButtons(buttons);

  category->addHeadLine("Map Settings");
  category->addSlider(new DoubleSlider("Min X", -10.0, 10.0, "min_x", this));
  category->addSlider(new DoubleSlider("Max X", -10.0, 10.0, "max_x", this));
  category->addSlider(new DoubleSlider("Min Y", -10.0, 10.0, "min_y", this));
  category->addSlider(new DoubleSlider("Max Y", -10.0, 10.0, "max_y", this));
  category->addSlider(new DoubleSlider("Spacial Resolution", 0.001, 1.0, "resolution_space", this));
  category->addSlider(new DoubleSlider("Min T", -10.0, 10.0, "min_t", this));
  category->addSlider(new DoubleSlider("Max T", -10.0, 10.0, "max_t", this));
  category->addSlider(new DoubleSlider("Time Resolution", 0.01, 5.0, "resolution_time", this));
  category->addSlider(new DoubleSlider("Safety Distance", 0.0, 2.0, "safety_distance", this));
  category->addSlider(new DoubleSlider("Safety Boundary", 0.0, 2.0, "safety_boundary", this));

  category->addHeadLine("Segmentation");
  category->addSlider(new DoubleSlider("Max Vert. Angle", 0.0, 180.0, "segmentation_max_angle_vertical", this));
  category->addSlider(new DoubleSlider("Max Dev. Angle", 0.0, 180.0, "segmentation_max_angle_deviation", this));
  category->addSlider(new DoubleSlider("Max Seg. Dist.", 0.0, 0.5, "segmentation_max_segment_distance", this));
  category->addSlider(new IntSlider("Max Seg. Count", 10, 1000, "max_segments_per_map", this));

  category->addHeadLine("Object Tracking");
  category->addSlider(new DoubleSlider("Min Obj. Size", 0.0, 2.0, "min_object_size", this));
  category->addSlider(new DoubleSlider("Max Obj. Size", 0.0, 5.0, "max_object_size", this));
  category->addSlider(new IntSlider("Histroy Count", 1, 300, "map_history_count", this));
  category->addSlider(new DoubleSlider("Speed Est. Time", 0.0, 2.0, "max_object_tracking_time", this));

  category->addHeadLine("", 8);
  buttons.clear();
  buttons.push_back(new SignalButton("Create Mapper", "create_mapper"));
  buttons.push_back(new ToggleButton("Run Mapping", "Stop Mapping", "is_mapping_running_changed", "is_mapping_running"));
  category->addButtons(buttons);
  category->addInfoText("info_height_map");

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryGridPlanning()
{
  Category* category = addCategory("GridPlanning");
  std::vector<AbstractButton*> buttons;

  category->addHeadLine("Global Planning");
  category->addSlider(new DoubleSlider("Smoothing Distance", 0.0, 1.0, "astar_global_smoothing_distance", this));
  category->addSlider(new DoubleSlider("Smoothing Factor", 0.0, 3.0, "astar_global_smoothing_factor", this));
  category->addSlider(new DoubleSlider("Point Distance", 0.001, 1.0, "astar_global_smoothed_point_distance", this));

  category->addHeadLine("Local Planning");
  category->addSlider(new DoubleSlider("Smoothing Distance", 0.0, 1.0, "astar_local_smoothing_distance", this));
  category->addSlider(new DoubleSlider("Smoothing Factor", 0.0, 3.0, "astar_local_smoothing_factor", this));
  category->addSlider(new DoubleSlider("Point Distance", 0.001, 1.0, "astar_local_smoothed_point_distance", this));

  buttons.clear();
  buttons.push_back(new ToggleButton("Run Local Planning", "Stop Local Planning", "astar_local_is_running_changed", "astar_local_is_running"));
  category->addButtons(buttons);

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryFootstepPlanning()
{
  Category* category = addCategory("FootstepPlanning");
  std::vector<AbstractButton*> buttons;

  category->addHeadLine("Foot Settings");
  category->addSlider(new DoubleSlider("Size Front", 0.0, 0.3, "foot_size_front", this));
  category->addSlider(new DoubleSlider("Size Back", 0.0, 0.3, "foot_size_back", this));
  category->addSlider(new DoubleSlider("Size Inner", 0.0, 0.3, "foot_size_inner", this));
  category->addSlider(new DoubleSlider("Size Outer", 0.0, 0.3, "foot_size_outer", this));
  category->addSlider(new DoubleSlider("Separation", 0.0, 0.5, "foot_separation", this));

  category->addHeadLine("Expansion Map Settings");
  category->addSlider(new DoubleSlider("Forward", 0.0, 1.0, "expansion_map_forward", this));
  category->addSlider(new DoubleSlider("Sideways", 0.0, 1.0, "expansion_map_sideways", this));
  category->addSlider(new DoubleSlider("Backward", 0.0, 1.0, "expansion_map_backward", this));
  category->addSlider(new DoubleSlider("Up", 0.0, 1.0, "expansion_map_up", this));
  category->addSlider(new DoubleSlider("Down", 0.0, 1.0, "expansion_map_down", this));
  category->addSlider(new DoubleSlider("Node Distance", 0.0, 0.1, "expansion_map_node_distance", this));
  category->addSlider(new DoubleSlider("Max Foot Rotation", 0.0, 180.0, "expansion_map_max_foot_rotation", this));
  category->addSlider(new DoubleSlider("Base Height", 0.5, 1.0, "expansion_base_height", this));
  buttons.push_back(new ToggleButton("Perform Reachability Checks", "Don't Perform Reachability Checks", "", "perform_reachability_checks"));
  category->addButtons(buttons);

  category->addHeadLine("Search Settings");
  category->addSlider(new DoubleSlider("Ang. Skip Dist.", 0.01, 30, "search_skip_distance", this));
  category->addSlider(new DoubleSlider("Skip Dist. Increase", 0.0, 30, "search_skip_increase_factor", this));
  category->addSlider(new DoubleSlider("Node Map Resolution", 0.01, 0.5, "node_map_resolution", this));
  category->addSlider(new IntSlider("Max Nodes Per Node Cell", 1, 500, "max_nodes_per_node_cell", this));

  category->addHeadLine("Trajectory Settings");
  category->addSlider(new DoubleSlider("Intermed. Pose Dists.", 0.01, 0.5, "trajectory_pose_distances", this));
  category->addSlider(new DoubleSlider("Step Normal Height", 0.0, 0.5, "trajectory_normal_height", this));
  category->addSlider(new DoubleSlider("Step Over Height", 0.0, 0.5, "trajectory_over_height", this));

  category->addHeadLine("", 8);

  buttons.clear();
  buttons.push_back(new SignalButton("Initialize Planner", "initialize_footstep_planner"));
  buttons.push_back(new ToggleButton("Run Footstep Planning", "Stop Footstep Planning", "run_footstep_planning_changed", "run_footstep_planning"));
  category->addButtons(buttons);
  category->addInfoText("info_footstep_planner");

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryVisualization()
{
  Category* category = addCategory("Visualization");
  std::vector<AbstractButton*> buttons;

  category->addHeadLine("Mapping");
  buttons.clear();
  MenuButton* menuButton = new MenuButton("displayed_map_type_changed", "displayed_map_type", this);
  menuButton->addMenuItem("Height Map", "height");
  menuButton->addMenuItem("Normal Map", "normal");
  menuButton->addMenuItem("Segment Type Map", "segment");
  menuButton->addMenuItem("Edge Map", "edge");
  buttons.push_back(menuButton);
  category->addButtons(buttons);

  category->addHeadLine("Global Path");
  category->addSlider(new DoubleSlider("Radius", 0.001, 0.1, "display_global_radius", this));
  category->addSlider(new DoubleSlider("R", 0, 1.0, "display_global_r", this));
  category->addSlider(new DoubleSlider("G", 0, 1.0, "display_global_g", this));
  category->addSlider(new DoubleSlider("B", 0, 1.0, "display_global_b", this));
  category->addSlider(new DoubleSlider("A", 0, 1.0, "display_global_a", this));

  category->addHeadLine("Local Path");
  category->addSlider(new DoubleSlider("Radius", 0.001, 0.1, "display_local_radius", this));
  category->addSlider(new DoubleSlider("R", 0, 1.0, "display_local_r", this));
  category->addSlider(new DoubleSlider("G", 0, 1.0, "display_local_g", this));
  category->addSlider(new DoubleSlider("B", 0, 1.0, "display_local_b", this));
  category->addSlider(new DoubleSlider("A", 0, 1.0, "display_local_a", this));

  category->addHeadLine("Footstep Plan");
  category->addSlider(new DoubleSlider("Right R", 0, 1.0, "display_foot_r_r", this));
  category->addSlider(new DoubleSlider("Right G", 0, 1.0, "display_foot_r_g", this));
  category->addSlider(new DoubleSlider("Right B", 0, 1.0, "display_foot_r_b", this));
  category->addSlider(new DoubleSlider("Right A", 0, 1.0, "display_foot_r_a", this));
  category->addSlider(new DoubleSlider("Left R", 0, 1.0, "display_foot_l_r", this));
  category->addSlider(new DoubleSlider("Left G", 0, 1.0, "display_foot_l_g", this));
  category->addSlider(new DoubleSlider("Left B", 0, 1.0, "display_foot_l_b", this));
  category->addSlider(new DoubleSlider("Left A", 0, 1.0, "display_foot_l_a", this));

  category->addHeadLine("", 8);
  buttons.clear();
  buttons.push_back(new SignalButton("Update Visual Settings", "display_update_settings"));
  category->addButtons(buttons);

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryWalkingControl()
{
  Category* category = addCategory("WalkingControl");
  std::vector<AbstractButton*> buttons;

  category->addSlider(new DoubleSlider("Footstep Duration", 0.5, 10.0, "robot_footstep_duration", this));
  category->addSlider(new DoubleSlider("Wait After Step", 0.0, 10.0, "robot_wait_after_step", this));

  buttons.push_back(new ToggleButton("Start Walking", "Stop Walking", "run_footstep_execution_changed", "run_footstep_execution"));
  category->addButtons(buttons);

  category->finalizeLayout();
}

void RosCommunicationPanel::constructCategoryTesting()
{
  Category* category = addCategory("Testing");

  category->addSlider(new DoubleSlider("Head Pitch", -90, 90, "head_pitch_set", this));

  category->finalizeLayout();
}

} // namespace rviz_ros_communication_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_ros_communication_plugin::RosCommunicationPanel, rviz::Panel)
