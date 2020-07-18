#ifndef _robotDataStruct_h_
#define _robotDataStruct_h_

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

typedef struct axis_s
{
  double_t x;
  double_t y;
  double_t z;
}axis_t;

typedef struct jointParam_s
{
  double_t angle;
  double_t torque; // to do
}jointParam_t;

typedef struct forceParam_s
{
  axis_t force[4];
  axis_t moment[4];
}forceParam_t;

typedef struct imuParam_s
{
  axis_t angularVelocity;
  axis_t linearAcceleration;
}imuParam_t;

#endif