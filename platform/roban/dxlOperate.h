#ifndef _dxlOperate_h_
#define _dxlOperate_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "../../include/hardward_interface/robotDataStruct.h"

#define SERVO_NUMBER_MAX 22

struct ServoInfo_s
{
  uint8_t number;
  uint8_t idList[SERVO_NUMBER_MAX];
};

class Servo
{
private:
  static bool initHandlers(void);
  static void initPrarm(uint8_t *idList, uint8_t number);

public:
  static bool init();
  static bool read(uint8_t id, jointParam_t* param);
  static bool write(uint8_t id, jointParam_t param);

  static bool bulkRead(uint8_t* idList, uint8_t number, jointParam_t* paramList);
  static bool syncWrite(uint8_t* idList, uint8_t number, jointParam_t* paramList);
};

class RobanFsr
{
public:
  static bool read(uint8_t id, forceParam_t* param);
  static bool bulkRead(uint8_t* idList, uint8_t number, forceParam_t* paramList);
};

class RobanImu
{
public:
  static bool read(uint8_t id, imuParam_t* param);
};

class DxlDevise
{
public:
  static bool init();
  static bool ping(uint8_t id);
};

#endif
