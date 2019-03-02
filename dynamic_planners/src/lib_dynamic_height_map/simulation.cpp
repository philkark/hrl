#include <dynamic_height_map/simulation.h>

// *********************************************************************

// ************************** SimulationObject *************************

// *********************************************************************

// ************************** PUBLIC ***********************************

void SimulationObject::SimulationObjectConfig::finalize()
{
  if (positionFinal == positionStart)
    movementDuration = 0.0;
  else
  {
    direction = positionFinal - positionStart;
    factorSq = -direction.norm() / pow(movementDuration, 2);
    factorLin = 2 * direction.norm() / movementDuration;
    direction.normalize();
  }

  if (color.r == 0.0 && color.g == 0.0 && color.b == 0.0 && color.a == 0.0)
  {
    color.a = 1.0;
    color.r = color.g = color.b = 0.5;
  }
}

SimulationObject::SimulationObject(const SimulationObjectConfig &config) :
    config(config)
{
  this->config.finalize();
}

const SimulationObject::SimulationObjectConfig &SimulationObject::getConfig() const
{
  return config;
}

PV2D SimulationObject::getPosition(const Real time) const
{
  if (time < 0.0)
    return config.positionStart;
  else if (time < config.movementDuration)
    return config.positionStart + config.direction * (config.factorSq * pow(time, 2) + config.factorLin * time);
  else
    return config.positionFinal;
}

Cell2D SimulationObject::getCell(const Real time) const
{
  PV2D position = getPosition(time);
  return Cell2D((UInt)((position.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip),
                (UInt)((position.C1 - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip));
}

void SimulationObject::getCells(const Real time, std::vector<Cell2DHeight> &cells) const
{
  switch (config.type)
  {
    case SimulationObjectType::Cylinder:
      computeCellsCylinder(getPosition(time), cells);
      break;
    case SimulationObjectType::Sphere:
      computeCellsSphere(getPosition(time), cells);
      break;
    case SimulationObjectType::Cuboid:
      computeCellsCuboid(getPosition(time), cells);
      break;
  }
}

Real SimulationObject::getDistanceToGoal(const Real time) const
{
  return (getPosition(time) - config.positionFinal).norm();
}

// ************************** PRIVATE **********************************

Cell2DSigned SimulationObject::getCellSigned(const Real time) const
{
  PV2D position = getPosition(time);
  return Cell2DSigned((Int)((position.C0 - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip),
                      (Int)((position.C1 - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip));
}

void SimulationObject::computeCellsSphere(const PV2D &position, std::vector<Cell2DHeight> &cells) const
{
  Int indexXMin = (Int)((position.C0 - config.radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexXMax = (Int)((position.C0 + config.radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);
  Int indexYMin = (Int)((position.C1 - config.radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexYMax = (Int)((position.C1 + config.radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);

  if (indexXMin < 0)
    indexXMin = 0;
  if (indexXMax < 0)
    indexXMax = 0;
  if (indexYMin < 0)
    indexYMin = 0;
  if (indexYMax < 0)
    indexYMax = 0;
  if (indexXMin >= DynamicHeightMapConfig::sizeX)
    indexXMin = DynamicHeightMapConfig::sizeX - 1;
  if (indexXMax >= DynamicHeightMapConfig::sizeX)
    indexXMax = DynamicHeightMapConfig::sizeX - 1;
  if (indexYMin >= DynamicHeightMapConfig::sizeY)
    indexYMin = DynamicHeightMapConfig::sizeY - 1;
  if (indexYMax >= DynamicHeightMapConfig::sizeY)
    indexYMax = DynamicHeightMapConfig::sizeY - 1;

  if (indexXMax <= indexXMin || indexYMax <= indexYMin)
    return;

  cells.clear();
  cells.reserve((indexXMax - indexXMin) * (indexYMax - indexYMin));

  const Real radiusSq = pow(config.radius, 2);

  for (UInt xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
    for (UInt yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
    {
      const Real x = DynamicHeightMapConfig::minX + (xIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C0;
      const Real y = DynamicHeightMapConfig::minY + (yIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C1;
      const Real distSq = x * x + y * y;
      if (distSq <= radiusSq)
        cells.push_back(Cell2DHeight(xIndex, yIndex, config.radius + sqrt(pow(config.radius, 2) - distSq)));
    }
}

void SimulationObject::computeCellsCylinder(const PV2D &position, std::vector<Cell2DHeight> &cells) const
{
  Int indexXMin = (Int)((position.C0 - config.radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexXMax = (Int)((position.C0 + config.radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);
  Int indexYMin = (Int)((position.C1 - config.radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexYMax = (Int)((position.C1 + config.radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);

  if (indexXMin < 0)
    indexXMin = 0;
  if (indexXMax < 0)
    indexXMax = 0;
  if (indexYMin < 0)
    indexYMin = 0;
  if (indexYMax < 0)
    indexYMax = 0;
  if (indexXMin >= DynamicHeightMapConfig::sizeX)
    indexXMin = DynamicHeightMapConfig::sizeX - 1;
  if (indexXMax >= DynamicHeightMapConfig::sizeX)
    indexXMax = DynamicHeightMapConfig::sizeX - 1;
  if (indexYMin >= DynamicHeightMapConfig::sizeY)
    indexYMin = DynamicHeightMapConfig::sizeY - 1;
  if (indexYMax >= DynamicHeightMapConfig::sizeY)
    indexYMax = DynamicHeightMapConfig::sizeY - 1;

  if (indexXMax <= indexXMin || indexYMax <= indexYMin)
    return;

  cells.clear();
  cells.reserve((indexXMax - indexXMin) * (indexYMax - indexYMin));

  const Real radiusSq = pow(config.radius, 2);

  for (UInt xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
    for (UInt yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
    {
      const Real x = DynamicHeightMapConfig::minX + (xIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C0;
      const Real y = DynamicHeightMapConfig::minY + (yIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C1;
      const Real distSq = x * x + y * y;
      if (distSq <= radiusSq)
        cells.push_back(Cell2DHeight(xIndex, yIndex, config.height));
    }
}

void SimulationObject::computeCellsCuboid(const PV2D &position, std::vector<Cell2DHeight> &cells) const
{
  const Real radius = sqrt(pow(config.length * 0.5, 2) + pow(config.width * 0.5, 2));

  Int indexXMin = (Int)((position.C0 - radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexXMax = (Int)((position.C0 + radius - DynamicHeightMapConfig::minX) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);
  Int indexYMin = (Int)((position.C1 - radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip - 1);
  Int indexYMax = (Int)((position.C1 + radius - DynamicHeightMapConfig::minY) * DynamicHeightMapConfig::resolutionSpaceRecip + 1);

  if (indexXMin < 0)
    indexXMin = 0;
  if (indexXMax < 0)
    indexXMax = 0;
  if (indexYMin < 0)
    indexYMin = 0;
  if (indexYMax < 0)
    indexYMax = 0;
  if (indexXMin >= DynamicHeightMapConfig::sizeX)
    indexXMin = DynamicHeightMapConfig::sizeX - 1;
  if (indexXMax >= DynamicHeightMapConfig::sizeX)
    indexXMax = DynamicHeightMapConfig::sizeX - 1;
  if (indexYMin >= DynamicHeightMapConfig::sizeY)
    indexYMin = DynamicHeightMapConfig::sizeY - 1;
  if (indexYMax >= DynamicHeightMapConfig::sizeY)
    indexYMax = DynamicHeightMapConfig::sizeY - 1;

  if (indexXMax <= indexXMin || indexYMax <= indexYMin)
    return;

  cells.clear();
  cells.reserve((indexXMax - indexXMin) * (indexYMax - indexYMin));

  const Real cosAlpha = cos(config.angle);
  const Real sinAlpha = sin(config.angle);
  const Real lengthHalf = config.length * 0.5;
  const Real widthHalf = config.width * 0.5;

  for (UInt xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
    for (UInt yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
    {
      const Real x = DynamicHeightMapConfig::minX + (xIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C0;
      const Real y = DynamicHeightMapConfig::minY + (yIndex + 0.5) * DynamicHeightMapConfig::resolutionSpace - position.C1;
      const Real xRotated = x * cosAlpha + y * sinAlpha;
      const Real yRotated = -x * sinAlpha + y * cosAlpha;

      if (xRotated >= -lengthHalf && xRotated <= lengthHalf && yRotated >= -widthHalf && yRotated <= widthHalf)
        cells.push_back(Cell2DHeight(xIndex, yIndex, config.height));
    }

//    for (Int x = -indexLimit; x <= indexLimit; ++x)
//      for (Int y = -indexLimit; y <= indexLimit; ++y)
//      {
//        pointTmp.C0 = x * DynamicHeightMapConfig::resolutionSpace;
//        pointTmp.C1 = y * DynamicHeightMapConfig::resolutionSpace;
//        MathStd::rotateAroundPoint(pointTmp, PV2D(), -angle);
//
//        if (MathStd::isInside(pointTmp, PV2D(-length * 0.5, -width * 0.5), PV2D(length * 0.5, width * 0.5)))
//          cellsCenter.push_back(Cell2DSignedHeight(x, y, height));
//      }
}

// *********************************************************************

// ************************** HeightMapSimulation **********************

// *********************************************************************

// ************************** PUBLIC ***********************************

void HeightMapSimulation::loadObjectsFromFile()
{
  deleteAllObjects();

  std::ifstream file(SimulationConfig::simulationFilePath.c_str());
  if (!file.good())
  {
    std::cout << "Could not read simulation scenario file!" << std::endl;
    return;
  }

  std::string line;
  std::pair<std::string, std::string> keyValuePair;

  while (std::getline(file, line))
  {
    if (line.empty() || line[0] == '#')
      continue;

    getKeyValuePair(line, keyValuePair);

    SimulationObject::SimulationObjectConfig config;
    if (keyValuePair.first == "-Sphere" && keyValuePair.second.empty())
      config.type = SimulationObjectType::Sphere;
    else if (keyValuePair.first == "-Cylinder" && keyValuePair.second.empty())
      config.type = SimulationObjectType::Cylinder;
    else if (keyValuePair.first == "-Cuboid" && keyValuePair.second.empty())
      config.type = SimulationObjectType::Cuboid;
    else
      continue;

    readConfig(file, config);
    addObject(config);
  }

  file.close();
}

void HeightMapSimulation::updateMap(const Real time, HeightMap &heightMap, HeightMap &heightMapInitial)
{
  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
      heightMap[xIndex][yIndex] = 0.0;

  for (SimulationObjectIterator object = objects.begin(); object != objects.end(); ++object)
  {
    std::vector<Cell2DHeight> cells;
    object->getCells(time, cells);
    for (Cell2DHeightIt cell = cells.begin(); cell != cells.end(); ++cell)
      heightMap[cell->x][cell->y] = cell->height;
  }

  std::normal_distribution<Real> distribution(SimulationConfig::simulationNoiseMean, SimulationConfig::simulationNoiseStdDev);

  for (UInt xIndex = 0; xIndex < DynamicHeightMapConfig::sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < DynamicHeightMapConfig::sizeY; ++yIndex)
    {
      heightMap[xIndex][yIndex] += distribution(randomEngine);
      heightMapInitial[xIndex][yIndex] = heightMap[xIndex][yIndex];
    }
}

void HeightMapSimulation::addObject(const SimulationObject::SimulationObjectConfig &config)
{
  objects.push_back(SimulationObject(config));
}

void HeightMapSimulation::deleteAllObjects()
{
  objects.clear();
}

void HeightMapSimulation::deleteLastObject()
{
  if (objects.size() > 0)
    objects.pop_back();
}

const std::vector<SimulationObject> &HeightMapSimulation::getObjects() const
{
  return objects;
}

// ************************** PRIVATE **********************************

void HeightMapSimulation::getKeyValuePair(const std::string &str, std::pair<std::string, std::string> &keyValuePair) const
{
  keyValuePair.first.clear();
  keyValuePair.second.clear();

  bool readValue = false;
  for (std::string::const_iterator it = str.begin(); it != str.end(); ++it)
  {
    if (*it == ':')
    {
      readValue = true;
      continue;
    }

    if (!readValue)
      keyValuePair.first.push_back(*it);
    else
      keyValuePair.second.push_back(*it);
  }
}

void HeightMapSimulation::readConfig(std::ifstream &file, SimulationObject::SimulationObjectConfig &config)
{
  std::string line;
  std::pair<std::string, std::string> keyValuePair;

  bool gotName = false, gotMinimum = false, gotMaximum = false;

  while (std::getline(file, line))
  {
    if (line.empty())
      continue;

    getKeyValuePair(line, keyValuePair);

    if (keyValuePair.first == "position_start" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.positionStart.C0 >> config.positionStart.C1))
        continue;
    }
    else if (keyValuePair.first == "position_final" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.positionFinal.C0 >> config.positionFinal.C1))
        continue;
    }
    else if (keyValuePair.first == "movement_duration" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.movementDuration))
        continue;
    }
    else if (keyValuePair.first == "radius" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.radius))
        continue;
    }
    else if (keyValuePair.first == "length" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.length))
        continue;
    }
    else if (keyValuePair.first == "width" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.width))
        continue;
    }
    else if (keyValuePair.first == "height" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.height))
        continue;
    }
    else if (keyValuePair.first == "angle" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.angle))
        continue;
      config.angle *= TORAD;
    }
    else if (keyValuePair.first == "color" && !keyValuePair.second.empty())
    {
      std::istringstream iss(keyValuePair.second);
      if (!(iss >> config.color.r >> config.color.g >> config.color.b >> config.color.a))
        continue;
    }
    else if (keyValuePair.first == "full_height" && !keyValuePair.second.empty())
    {
      if (keyValuePair.second == "true")
        config.displayFullHeight = true;
      else
        config.displayFullHeight = false;
    }
    else if (keyValuePair.first == "-" && keyValuePair.second.empty())
      break;
    else
      continue;
  }
}

