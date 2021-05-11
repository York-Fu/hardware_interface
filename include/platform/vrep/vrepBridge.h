#ifndef _vrepBridge_h_
#define _vrepBridge_h_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "sensorDataStruct.h"

namespace vrepBridge
{
  bool simInit();
  void simStart();
  void simTrigNext();
  bool simStop();

  bool simSetJointPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  bool simSetJointVelocity(std::vector<uint16_t> &ids, std::vector<double_t> &velocity);
  bool simSetJointTorque(std::vector<uint16_t> &ids, std::vector<double_t> &torque);
  bool simGetJointPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position);
  bool simGeJointVelocity(std::vector<uint16_t> &ids, std::vector<double_t> &velocity);
  bool simGeJointTorque(std::vector<uint16_t> &ids, std::vector<double_t> &torque);
  bool simGetForce(std::vector<ForceParam_t> &data);
  bool simGetImu(std::vector<ImuParam_t> &data);
  bool simGetSensor(SensorParam &sensor);
} // namespace vrepBridge

#endif