#include "../include/hardward_interface/robotInterface.h"

#include "ros/ros.h"
#include "../platform/emptyRobot/emptyRobot.h"
#include "../platform/roban/dxlOperate.h"

Robot robot;

void robotInterfaceAssert()
{
  if(robot.f_init == NULL)
    robot.f_init = &EmptyRobot::init;

  if(robot.c_actuator.f_ping == NULL)
    robot.c_actuator.f_ping = &EmptyRobot::ping;

  if(robot.c_actuator.c_joint.f_read == NULL)
    robot.c_actuator.c_joint.f_read = &EmptyJoint::read;

  if(robot.c_actuator.c_joint.f_write == NULL)
    robot.c_actuator.c_joint.f_write = &EmptyJoint::write;

  if(robot.c_actuator.c_joint.f_bulkRead == NULL)
    robot.c_actuator.c_joint.f_bulkRead = &EmptyJoint::bulkRead;

  if(robot.c_actuator.c_joint.f_syncWrite == NULL)
    robot.c_actuator.c_joint.f_syncWrite = &EmptyJoint::syncWrite;

  if(robot.c_sensor.f_ping == NULL)
    robot.c_sensor.f_ping = &EmptyRobot::ping;

  if(robot.c_sensor.c_force.f_read == NULL)
    robot.c_sensor.c_force.f_read = &EmptyForce::read;

  if(robot.c_sensor.c_force.f_bulkRead == NULL)
    robot.c_sensor.c_force.f_bulkRead = &EmptyForce::bulkRead;

  if(robot.c_sensor.c_imu.f_read == NULL)
    robot.c_sensor.c_imu.f_read = &EmptyImu::read;
}

bool robotInterfaceInit()
{
  robot.f_init = &DxlDevise::init;

  robot.c_actuator.f_ping = &DxlDevise::ping;
  robot.c_actuator.c_joint.f_read = &Servo::read;
  robot.c_actuator.c_joint.f_write = &Servo::write;
  robot.c_actuator.c_joint.f_bulkRead = &Servo::bulkRead;
  robot.c_actuator.c_joint.f_syncWrite = &Servo::syncWrite;

  robot.c_sensor.f_ping = &DxlDevise::ping;
  robot.c_sensor.c_force.f_read = &RobanFsr::read;
  robot.c_sensor.c_force.f_bulkRead = &RobanFsr::bulkRead;
  robot.c_sensor.c_imu.f_read = &RobanImu::read;

  if (robot.f_init() == false)
  {
    ROS_ERROR("robot init failed, program exit!");
    exit(1);
  }

  robotInterfaceAssert();
  return true;
}
