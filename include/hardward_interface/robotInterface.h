#ifndef _robotInterface_h_
#define _robotInterface_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "robotDataStruct.h"

class Joint
{
public:
    bool (*f_read)(uint8_t id, jointParam_t* Param);
    bool (*f_write)(uint8_t id, jointParam_t Param);

    bool (*f_bulkRead)(uint8_t* idList, uint8_t number, jointParam_t* ParamList);
    bool (*f_syncWrite)(uint8_t* idList, uint8_t number, jointParam_t* ParamList);
};

class Force
{
public:
    bool (*f_read)(uint8_t id, forceParam_t* Param);
    bool (*f_bulkRead)(uint8_t* idList, uint8_t number, forceParam_t* ParamList);
};

class Imu
{
public:
    bool (*f_read)(uint8_t id, imuParam_t* Param);
};

class Actuator
{
public:
  bool (*f_ping)(uint8_t id);

  Joint c_joint;
};

class Sensor
{
public:
  bool (*f_ping)(uint8_t id);

  Force c_force;
  Imu c_imu;
};

class Robot
{
public:
  bool (*f_init)();

  Actuator c_actuator;
  Sensor c_sensor;
};


extern Robot robot;

bool robotInterfaceInit();

#endif