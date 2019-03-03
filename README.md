# General Info

This repository contains several packages for segmenting depth images into planar and non-planar regions, 2D path planning algorithms and 3D footstep planning.

There are four catkin packages, which are all necessary to run the mapping and planning framework.

## dynamic_planners

## rviz_ros_communiation_plugin

This package provides an rviz plugin that communicates with the main controller described above. The plugin is automatically added as an rviz panel, when launching the planning packages. It can also be added in rviz via the top menu *Panels -> Add New Panel* and needs to be selected from *rviz_ros_communiation_plugin -> ReemcPanel*.

In general, the plugin contains a set of sliders, buttons, and text fields that set different ros parameters in the parameter server and publish empty message to different ros topics as a way of signaling.

It contains the following categories:

#### World

* **Robot** Allows to reset the position of the robot to the given 6D pose on the click of the button. The movement speed is used for footstep executions and using the correct predicted map during the planning phase.

* **Global Map** Generates a global height map using the objects in gazebo. It allows for the settings *Fixed Size*, which takes the given settings into account and *Auto Size* which generates a height map around the bounding box of the objects found in the current gazebo simulation.

#### Mapping

* **Camera Settings** Settings for the depth camera, which are necessary to correctly compute the height map from the depth image. *Use Only Global Map* allows to skip the data from the camera and only fill the local map with data from the global map.

* **Map Settings** Settings for the local map, centered around the *base_link* of the robot. *Safety Distance* should be approximately set to the size of the foot in case of footstep planning.

* **Segmentation** Settings needed for correctly segmenting the height map into planar and non-planar regions. *Max Vert. Angle* is the maximum angle from the z-axis that still counts any plane as planar (to avoid steep slopes). *Max Dev. Angle* accounts for the angle between neighboring normal vectors that still belong to the same planar region. *Max Seg. Count* sets the maximum number of different segments for each mapping cycle, to avoid usage of too much memory.

* **Object Tracking** Allows adjusting settings to adjust which objects are tracked and what time is being used to estimate their future positions.

#### GridPlanning

* **Global Planning** Settings that account for 2D grid planning on the global map. *Smoothing Distance* and *Smothing Factor* are used when converting the 2D grid path into a set of connected line segments. *Point Distance* is the final maximum distance between two neighboring points along the smoothed path.

* **Local Planning** Same as for the global planning section, but for 2D grid planning in the local height map.

#### FootstepPlanning

* **Foot Settings** Settings for the foot sizes relative to the *sole link* and the foot separation when the robot is in its initial pose.

* **Expansion Map Settings** The expansion map defines the reachabilities of the swing foot relative to the support foot from the zero position of the swing foot. *Node Distance* defines the minimum distance between poses in the reachability map. *Max Foot Rotation* clips the rotation of the foot, which in general points in the same direction as the displacement from the zero position. *Base Height* is the height of the base above the support foot. You can also enable performing reachability checks using inverse kinematics to check if the extreme expansion map poses are in fact reachable.

* **Search Settings** Settings that define works of the footstep plan search.

* **Trajectory Settings** Settings that define how collision checks are performed for intermediate poses between footstep motions during the footstep planning.

#### Visualization

* **Mapping** Allows selecting which map layer is being shown.

* **Global Path** Color and size of the global path points.

* **Local Path** Color and size of the local path points.

* **Footstep Plan** Colors of the right and left feet of the footstep plan.

#### WalkingControl

*Footstep Duration* sets the time the execution node will plan for performing the stepping motion, *Wait After Step* is the time the execution node should wait after finishing the stepping and base motions.

## math_std

Contains a library with several math related types, such as points/vectors, matrices, lines, etc. and contains many functions that allow for performing fast algebraic computations on these types.

## tools_std

Contains a library with several different classes that help in loading ros parameters, timing experiments, etc. 
