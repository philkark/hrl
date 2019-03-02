//ros
#include <ros/ros.h>

//local
#include "footstep_executor/footstep_executor.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "footstep_execution_node");

  FootstepExecutor* executor = new FootstepExecutor;

  if (!ros::ok())
  {
    std::cout << "Could not initialize." << std::endl;
    return 0;
  }

  executor->run();

  return 0;
}
