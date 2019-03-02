#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "dynamic_planners/get_height_map.h"
#include "dynamic_planners/gazebo_object.h"

namespace gazebo
{

class GazeboPluginHeightMapPublisher : public WorldPlugin
{
public:

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    world = _parent;
    serviceGetHeightMap = nh.advertiseService("get_height_map", &GazeboPluginHeightMapPublisher::serviceGetHeightMapHandler, this);
  }

private:
  physics::WorldPtr world;
  ros::NodeHandle nh;
  ros::Publisher publisherMarkers;
  ros::ServiceServer serviceGetHeightMap;
  event::ConnectionPtr updateConnection;

  bool serviceGetHeightMapHandler(dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res)
  {
    if (req.auto_size)
    {
      double minX = 1e10, maxX = -1e10, minY = 1e10, maxY = -1e10;
      getGlobalBoundingBox(minX, maxX, minY, maxY);
      res.min_x = minX;
      res.max_x = maxX;
      res.min_y = minY;
      res.max_y = maxY;
    }
    else
    {
      res.min_x = req.min_x;
      res.max_x = req.max_x;
      res.min_y = req.min_y;
      res.max_y = req.max_y;
    }

    if (res.min_x >= res.max_x || res.min_y >= res.max_y)
    {
      res.size_x = res.size_y = 0;
      return true;
    }

    res.size_x = (res.max_x - res.min_x) / req.resolution;
    res.size_y = (res.max_y - res.min_y) / req.resolution;

    res.height_data.resize(res.size_x * res.size_y, 0.0);

    fillHeightMap(req, res);

    return true;
  }

  void getGlobalBoundingBox(double &minX, double &maxX, double &minY, double &maxY)
  {
    physics::Model_V models = world->GetModels();

    for (int i = 0; i < models.size(); ++i)
    {
      if (models[i]->GetName() == "ground_plane" || models[i]->GetName().find("reemc") != std::string::npos)
        continue;

      gazebo::math::Box box = models[i]->GetBoundingBox();

      if (box.min.x < minX)
        minX = box.min.x;
      if (box.max.x > maxX)
        maxX = box.max.x;
      if (box.min.y < minY)
        minY = box.min.y;
      if (box.max.y > maxY)
        maxY = box.max.y;
    }
  }

  void fillHeightMap(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res)
  {
    physics::Model_V models = world->GetModels();

    for (int i = 0; i < models.size(); ++i)
    {
      if (models[i]->GetName() == "ground_plane" || models[i]->GetName().find("reemc") != std::string::npos)
        continue;

      unsigned int type = models[i]->GetLinks()[0]->GetCollisions()[0]->GetShapeType();

      if (type & physics::Shape::SPHERE_SHAPE)
      {
        fillHeightMapSphere(req, res, models[i]);
        addObjectSphere(res, models[i]);
      }
      else if (type & physics::Shape::CYLINDER_SHAPE)
      {
        fillHeightMapCylinder(req, res, models[i]);
        addObjectCylinder(res, models[i]);
      }
      else if (type & physics::Shape::BOX_SHAPE)
      {
        fillHeightMapCuboid(req, res, models[i]);
        addObjectCuboid(res, models[i]);
      }
    }
  }

  void fillHeightMapSphere(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::SphereShapePtr shape = boost::dynamic_pointer_cast<physics::SphereShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    const gazebo::math::Pose pose = model->GetRelativePose();
    const double radius = shape->GetRadius() + 0.05;
    const double radiusSq = pow(radius, 2);

    int indexXMin = (int)((pose.pos.x - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.pos.x + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.pos.y - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.pos.y + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.pos.x;
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.pos.y;
        const double distSq = pow(x, 2) + pow(y, 2);

        if (distSq <= radiusSq)
          res.height_data[yIndex + res.size_y * xIndex] = sqrt(radiusSq - distSq) + pose.pos.z;
      }
  }

  void fillHeightMapCylinder(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res,
                             const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::CylinderShapePtr shape = boost::dynamic_pointer_cast<physics::CylinderShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    const gazebo::math::Pose pose = model->GetRelativePose();
    const double radius = shape->GetRadius() + 0.05;
    const double radiusSq = pow(radius, 2);

    int indexXMin = (int)((pose.pos.x - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.pos.x + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.pos.y - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.pos.y + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    const double height = 0.5 * shape->GetLength() + pose.pos.z;

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.pos.x;
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.pos.y;
        const double distSq = pow(x, 2) + pow(y, 2);
        if (distSq <= radiusSq)
          res.height_data[yIndex + res.size_y * xIndex] = height;
      }
  }

  void fillHeightMapCuboid(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::BoxShapePtr shape = boost::dynamic_pointer_cast<physics::BoxShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    double sizeX = shape->GetSize().x + 0.05;
    double sizeY = shape->GetSize().y + 0.05;
    double sizeZ = shape->GetSize().z;

    const gazebo::math::Pose pose = model->GetRelativePose();
    const double radius = sqrt(pow(sizeX * 0.5, 2) + pow(sizeY * 0.5, 2));

    int indexXMin = (int)((pose.pos.x - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.pos.x + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.pos.y - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.pos.y + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    gazebo::math::Quaternion quat = pose.rot;
    const double yaw = quat.GetYaw();
    const double cosAlpha = cos(yaw);
    const double sinAlpha = sin(yaw);
    const double lengthHalf = sizeX * 0.5;
    const double widthHalf = sizeY * 0.5;

    const double height = 0.5 * sizeZ + pose.pos.z;

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.pos.x;
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.pos.y;
        const double xRotated = x * cosAlpha + y * sinAlpha;
        const double yRotated = -x * sinAlpha + y * cosAlpha;

        if (xRotated >= -lengthHalf && xRotated <= lengthHalf && yRotated >= -widthHalf && yRotated <= widthHalf)
          res.height_data[yIndex + res.size_y * xIndex] = height;
      }
  }

  void addObjectSphere(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::SphereShapePtr shape = boost::dynamic_pointer_cast<physics::SphereShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const gazebo::math::Pose pose = model->GetRelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::sphere;
    obj.radius = shape->GetRadius();
    obj.pos_x = pose.pos.x;
    obj.pos_y = pose.pos.y;
    obj.pos_z = pose.pos.z;
    obj.rot_x = pose.rot.x;
    obj.rot_y = pose.rot.y;
    obj.rot_z = pose.rot.z;
    obj.rot_w = pose.rot.w;
  }

  void addObjectCylinder(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::CylinderShapePtr shape = boost::dynamic_pointer_cast<physics::CylinderShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const gazebo::math::Pose pose = model->GetRelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::cylinder;
    obj.radius = shape->GetRadius();
    obj.height = shape->GetLength();
    obj.pos_x = pose.pos.x;
    obj.pos_y = pose.pos.y;
    obj.pos_z = pose.pos.z;
    obj.rot_x = pose.rot.x;
    obj.rot_y = pose.rot.y;
    obj.rot_z = pose.rot.z;
    obj.rot_w = pose.rot.w;
  }

  void addObjectCuboid(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::BoxShapePtr shape = boost::dynamic_pointer_cast<physics::BoxShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const gazebo::math::Pose pose = model->GetRelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::cuboid;
    obj.length = shape->GetSize().x;
    obj.width = shape->GetSize().y;
    obj.height = shape->GetSize().z;
    obj.pos_x = pose.pos.x;
    obj.pos_y = pose.pos.y;
    obj.pos_z = pose.pos.z;
    obj.rot_x = pose.rot.x;
    obj.rot_y = pose.rot.y;
    obj.rot_z = pose.rot.z;
    obj.rot_w = pose.rot.w;
  }

  inline void clip(int &num, const int min, const int max)
  {
    if (num < min)
      num = min;
    else if (num > max)
      num = max;
  }
};

GZ_REGISTER_WORLD_PLUGIN(GazeboPluginHeightMapPublisher)
}
