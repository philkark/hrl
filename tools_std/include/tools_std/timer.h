#ifndef TOOLS_STD_TIMER_H_
#define TOOLS_STD_TIMER_H_

#include <string>
#include <map>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

class Timer
{
private:
  static std::map<std::string, std::string> descriptions;
  static std::map<std::string, double> timeStart;
  static std::map<std::string, double> timeStop;

public:
  static void add(const std::string &name, const std::string &description);

  static void start(const std::string &name);

  static void stop(const std::string &name);

  static void show(const std::string &name, const uint32_t iterations);

  static void stopAndShow(const std::string &name, const uint32_t iterations);

  static double getTime(const std::string &name, const uint32_t iterations);
};

#endif // TOOLS_STD_TIMER_H_
