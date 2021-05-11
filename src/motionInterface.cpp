#include "motionInterface.h"

#include "vrepBridge.h"
#include "dxlOperate.h"

motionPhy_t motionPhy;

bool motionInterfaceSetup(std::string platform)
{
  if (strcmp(platform.c_str(), "sim") == 0)
  {
    motionPhy.init = vrepBridge::simInit;
    motionPhy.exit = vrepBridge::simStop;
    motionPhy.ping = NULL;

    motionPhy.setJoint = NULL;
    motionPhy.setJointPosition = vrepBridge::simSetJointPosition;
    motionPhy.setJointVelocity = vrepBridge::simSetJointVelocity;
    motionPhy.setJointTorque = vrepBridge::simSetJointTorque;

    motionPhy.getJoint = NULL;
    motionPhy.getJointPosition = vrepBridge::simGetJointPosition;
    motionPhy.getJointVelocity = vrepBridge::simGeJointVelocity;
    motionPhy.getJointTorque = vrepBridge::simGeJointTorque;
    motionPhy.getForce = vrepBridge::simGetForce;
    motionPhy.getImu = vrepBridge::simGetImu;
    motionPhy.getSensor = vrepBridge::simGetSensor;
  }
  else if (strcmp(platform.c_str(), "roban") == 0)
  {
    motionPhy.init = &DxlDevice::init;
    motionPhy.exit = &DxlDevice::exit;
    motionPhy.ping = &DxlDevice::ping;

    motionPhy.setJoint = NULL;
    motionPhy.setJointPosition = &RobanServo::setPosition;
    motionPhy.setJointVelocity = &RobanServo::setVelocity;
    motionPhy.setJointTorque = &RobanServo::setTorque;

    motionPhy.getJoint = NULL;
    motionPhy.getJointPosition = &RobanServo::getPosition;
    motionPhy.getJointVelocity = &RobanServo::getVelocity;
    motionPhy.getJointTorque = &RobanServo::getTorque;
    motionPhy.getForce = &RobanFsr::getData;
    motionPhy.getImu = &RobanImu::getData;
    motionPhy.getSensor = &RobanSensor::getData;
  }
  return true;
}
