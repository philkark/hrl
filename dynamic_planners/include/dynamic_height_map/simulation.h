#ifndef DYNAMIC_PLANNERS_SIMULATION_H_
#define DYNAMIC_PLANNERS_SIMULATION_H_

#include <dynamic_height_map/types.h>
#include <dynamic_height_map/cells.h>
#include <dynamic_height_map/dynamic_height_map_config.h>
#include <dynamic_height_map/objects.h>

#include <random>
#include <fstream>
#include <iostream>
#include <utility>

#include <math_std/common.h>
#include <math_std/pv.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>

class SimulationObject
{
public:
  struct SimulationObjectConfig
  {
    SimulationObjectType type;
    std_msgs::ColorRGBA color;
    bool displayFullHeight; ///only works for boxes and cylinders due to performance reasons

    PV2D positionStart;
    PV2D positionFinal;
    Real movementDuration;

    PV2D direction;
    Real factorSq;
    Real factorLin;

    Real radius;
    Real length;
    Real width;
    Real height;
    Real angle;

    void finalize();
  };

  SimulationObject(const SimulationObjectConfig &config);

  const SimulationObjectConfig &getConfig() const;

  PV2D getPosition(const Real time) const;

  Cell2D getCell(const Real time) const;

  void getCells(const Real time, std::vector<Cell2DHeight> &cells) const;

  Real getDistanceToGoal(const Real time) const;

private:
  SimulationObjectConfig config;

  Cell2DSigned getCellSigned(const Real time) const;

  void computeCellsSphere(const PV2D &position, std::vector<Cell2DHeight> &cells) const;

  void computeCellsCylinder(const PV2D &position, std::vector<Cell2DHeight> &cells) const;

  void computeCellsCuboid(const PV2D &position, std::vector<Cell2DHeight> &cells) const;
};

class HeightMapSimulation
{
public:
  void updateMap(const Real time, HeightMap &heightMap, HeightMap &heightMapInitial);

  void addObject(const SimulationObject::SimulationObjectConfig &objectConfig);

  void loadObjectsFromFile();

  void deleteAllObjects();

  void deleteLastObject();

  const std::vector<SimulationObject> &getObjects() const;

private:

  std::vector<SimulationObject> objects;
  std::default_random_engine randomEngine;

  void getKeyValuePair(const std::string &str, std::pair<std::string, std::string> &keyValuePair) const;

  void readConfig(std::ifstream &file, SimulationObject::SimulationObjectConfig &config);
};


#endif // DYNAMIC_PLANNERS_SIMULATION_H_
