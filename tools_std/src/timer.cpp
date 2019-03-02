#include <tools_std/timer.h>

std::map<std::string, std::string> Timer::descriptions;
std::map<std::string, double> Timer::timeStart;
std::map<std::string, double> Timer::timeStop;

void Timer::add(const std::string &name, const std::string &description)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it == descriptions.end())
  {
    descriptions[name] = description;
    timeStart[name] = 0;
    timeStop[name] = 0;
  }
}

void Timer::start(const std::string &name)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it != descriptions.end())
    timeStart[name] = ros::Time::now().toSec();
}

void Timer::stop(const std::string &name)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it != descriptions.end())
    timeStop[name] = ros::Time::now().toSec();
}

void Timer::show(const std::string &name, const uint32_t iterations)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it != descriptions.end())
  {
    std::cout << it->second << (timeStop[name] - timeStart[name]) * 1000.0 / (double)iterations << " ms" << std::endl;
  }
}

void Timer::stopAndShow(const std::string &name, const uint32_t iterations)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it != descriptions.end())
  {
    timeStop[name] = ros::Time::now().toSec();
    std::cout << it->second << (timeStop[name] - timeStart[name]) * 1000.0 / (double)iterations << " ms" << std::endl;
  }
}

double Timer::getTime(const std::string &name, const uint32_t iterations)
{
  std::map<std::string, std::string>::iterator it = descriptions.find(name);
  if (it != descriptions.end())
    return (timeStop[name] - timeStart[name]) * 1000 / iterations;
  else
    return -1.0;
}
