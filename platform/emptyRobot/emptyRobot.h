#ifndef _emptyRobot_h_
#define _emptyRobot_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "ros/ros.h"

#include "../../include/hardward_interface/robotDataStruct.h"

class EmptyJoint
{
public:
  static bool read(uint8_t id, jointParam_t* param) {ROS_ERROR("empty func!"); return false;}
  static bool write(uint8_t id, jointParam_t param) {ROS_ERROR("empty func!"); return false;}

  static bool bulkRead(uint8_t* idList, uint8_t number, jointParam_t* paramList) {ROS_ERROR("empty func!"); return false;}
  static bool syncWrite(uint8_t* idList, uint8_t number, jointParam_t* paramList) {ROS_ERROR("empty func!"); return false;}
};

class EmptyForce
{
public:
  static bool read(uint8_t id, forceParam_t* param) {ROS_ERROR("empty func!"); return false;}
  static bool bulkRead(uint8_t* idList, uint8_t number, forceParam_t* paramList) {ROS_ERROR("empty func!"); return false;}
};

class EmptyImu
{
public:
  static bool read(uint8_t id, imuParam_t* param) {ROS_ERROR("empty func!"); return false;}
};

class EmptyRobot
{
public:
  static bool init() {ROS_ERROR("empty func!"); return false;}
  static bool ping(uint8_t id) {ROS_ERROR("empty func!"); return false;}
};

#endif
