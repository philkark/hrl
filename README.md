# General Info

This repository contains several packages for segmenting depth images into planar and non-planar regions, 2D path planning algorithms and 3D footstep planning.

There are four catkin packages, which are all necessary to run the mapping and planning framework.

## rviz_ros_communiation_plugin

This package provides an rviz plugin that communicates with the main controller described above. The plugin is automatically added as an rviz panel, when launching the planning packages. It can also be added in rviz via the top menu *Panels -> Add New Panel* and needs to be selected from *rviz_ros_communiation_plugin -> ReemcPanel*.

In general, the plugin contains a set of sliders, buttons, and text fields that set different ros parameters in the parameter server and publish empty message to different ros topics as a way of signaling.

It contains the following categories:

#### World
