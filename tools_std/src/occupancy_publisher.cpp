#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

class OccupancyGridPublisher
{
public:
  OccupancyGridPublisher(const std::string &topic)
  {
    initializeMarker();

    publisher = nh.advertise<visualization_msgs::Marker>(topic, 1);
  }

  void publish()
  {
    publisher.publish(marker);
  }

  visualization_msgs::Marker &getCurrentMarker()
  {
    return marker;
  }

  void resetMarkerToLastOccupancyGrid()
  {
    marker = markerOrig;
  }

  bool setMarkerFromOccpuancyGrid(const nav_msgs::OccupancyGrid &occupancyGrid, const std::vector<std_msgs::ColorRGBA> &colorMapping)
  {
    if (colorMapping.size() != 256)
    {
      std::cout << "Color mapping must have size of 256. Did not set marker." << std::endl;
      return false;
    }

    marker.points.resize(occupancyGrid.info.width * occupancyGrid.info.height);
    marker.colors.resize(marker.points.size());
    marker.scale.x = marker.scale.y = occupancyGrid.info.resolution;

    resolution = occupancyGrid.info.resolution;
    sizeX = occupancyGrid.info.height;
    sizeY = occupancyGrid.info.width;
    const double offset = resolution * 0.5;

    size_t index = 0;
    for (size_t x = 0; x < sizeX; ++x)
    {
      for (size_t y = 0; y < sizeY; ++y)
      {
        geometry_msgs::Point &point = marker.points[index];
        std_msgs::ColorRGBA &color = marker.colors[index];

        point.x = offset + x * resolution;
        point.y = offset + y * resolution;
        point.z = -0.001;
        color = colorMapping[occupancyGrid.data[index]];

        ++index;
      }
    }

    markerOrig = marker;
    return true;
  }

  void setColor(const size_t x, const size_t y, const std_msgs::ColorRGBA &color)
  {
    marker.colors[y + x * sizeY] = color;
  }

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  visualization_msgs::Marker marker;
  visualization_msgs::Marker markerOrig;

  double resolution;
  size_t sizeX;
  size_t sizeY;

  void initializeMarker()
  {
    marker.id = 13131;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.z = 0.001;
    marker.pose.orientation.w = 1.0;
  }
};
