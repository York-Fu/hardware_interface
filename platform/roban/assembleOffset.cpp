
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ros/ros.h"
#include "dxlOperate.h"
#include <yaml-cpp/yaml.h>

int8_t assembleDirection[SERVO_NUMBER_MAX] = {
  1,-1,-1,-1,1,1,
  1,-1,1,1,-1,1,
  -1,-1,-1,
  1,-1,-1,
  -1,-1,
  1,1,
};

double_t assembleOffset[SERVO_NUMBER_MAX] = {0};


void LoadAssembleOffset()
{
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;
  YAML::Node offsetDoc;

  std::string yamlPath = __FILE__;
  yamlPath.erase(yamlPath.rfind("/"));
  yamlPath = yamlPath + "/config/offset.yaml";

  try
  {
    offsetDoc = YAML::LoadFile(yamlPath.c_str());
  }
  catch (const std::exception &e)
  {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0)
    return;

  for (YAML::const_iterator itItemNum = itemData.begin(); itItemNum != itemData.end(); itItemNum++)
  {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }
  for (uint8_t i = 0; i < SERVO_NUMBER_MAX; i++)
  {
    IDNameStr = stringID + std::to_string(i + 1);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("Without find offset of %s ", IDNameStr.c_str());
    else
      assembleOffset[i] = offsetMap[IDNameStr];
    ROS_INFO("assembleOffset[%d]: %f", i, assembleOffset[i - 1]);
  }
}
