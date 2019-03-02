#include "dynamic_height_map/global_map.h"

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

GlobalMap::GlobalMap()
{
}

void GlobalMap::setMap(const dynamic_planners::get_height_map::Request &req, const dynamic_planners::get_height_map::Response &res, const Real safetyDistance)
{
  minX = res.min_x;
  maxX = res.max_x;
  minY = res.min_y;
  maxY = res.max_y;
  resolution = req.resolution;
  resolutionRecip = 1 / req.resolution;

  sizeX = res.size_x;
  sizeY = res.size_y;

  pointMap.resize(sizeX, std::vector<PV3D>(sizeY));
  occupancyMap.resize(sizeX, std::vector<bool>(sizeY, false));
  std::vector<std::vector<Int> > distanceMap(sizeX, std::vector<Int>(sizeY, -1));
  std::vector<Cell2D> occupancyQueue;

  UInt index = 0;
  for (UInt xIndex = 0; xIndex < sizeX; ++xIndex)
    for (UInt yIndex = 0; yIndex < sizeY; ++yIndex)
    {
      pointMap[xIndex][yIndex].C0 = minX + (xIndex + 0.5) * resolution;
      pointMap[xIndex][yIndex].C1 = minY + (yIndex + 0.5) * resolution;
      pointMap[xIndex][yIndex].C2 = res.height_data[index];
      if (pointMap[xIndex][yIndex].C2 != 0.0)
      {
        distanceMap[xIndex][yIndex] = 0;
        occupancyMap[xIndex][yIndex] = true;
        occupancyQueue.push_back(Cell2D(xIndex, yIndex));
      }
      ++index;
    }

  generateOccupancyMap(distanceMap, occupancyQueue, (Int)(safetyDistance * 1000.0));
}

void GlobalMap::generateOccupancyMap(std::vector<std::vector<Int> > &distanceMap, std::vector<Cell2D> &occupancyQueue, const Int safetyDistanceMM)
{
  const Int resolutionMM = (Int)(resolution * 1000.0);
  const Int resolutionDiagMM = (Int)(resolution * 1000.0 * M_SQRT2);

  for (UInt cellIndex = 0; cellIndex < occupancyQueue.size(); ++cellIndex)
  {
    const Cell2D cellCenter = occupancyQueue[cellIndex];

    Cell2D cell(cellCenter.x - 1, cellCenter.y - 1);

    if (cell.x < sizeX && cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionDiagMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x + 1;
    if (cell.x < sizeX && cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionDiagMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < sizeX)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y + 1;
    if (cell.x < sizeX && cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionDiagMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x;
    if (cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.x = cellCenter.x - 1;
    if (cell.x < sizeX && cell.y < sizeY)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionDiagMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }

    cell.y = cellCenter.y;
    if (cell.x < sizeX)
    {
      UInt newDistance = distanceMap[cellCenter.x][cellCenter.y] + resolutionMM;
      if (newDistance < safetyDistanceMM && (distanceMap[cell.x][cell.y] < 0 || distanceMap[cell.x][cell.y] > newDistance))
      {
        distanceMap[cell.x][cell.y] = newDistance;
        occupancyMap[cell.x][cell.y] = true;
        occupancyQueue.push_back(Cell2D(cell.x, cell.y));
      }
    }
  }

  //add border
  for (UInt i = 0; i < sizeX; ++i)
  {
    occupancyMap[i][0] = true;
    occupancyMap[i][sizeY - 1] = true;
  }
  for (UInt i = 0; i < sizeY; ++i)
  {
    occupancyMap[0][i] = true;
    occupancyMap[sizeX - 1][i] = true;
  }
}

Real GlobalMap::getHeight(const Real x, const Real y) const
{
  const UInt xIndex = (UInt)((x - minX) * resolutionRecip);
  const UInt yIndex = (UInt)((y - minY) * resolutionRecip);
  if (xIndex >= sizeX || yIndex >= sizeY)
    return 0.0;
  else
    return pointMap[xIndex][yIndex].C2;
}

Cell2D GlobalMap::getCell(const PV2D &point) const
{
  return Cell2D((UInt)((point.C0 - minX) * resolutionRecip), (UInt)((point.C1 - minY) * resolutionRecip));
}

PV2D GlobalMap::getPoint(const Cell2D &cell) const
{
  return PV2D(minX + (cell.x + 0.5) * resolution, minY + (cell.y + 0.5) * resolution);
}
