#include "dxlOperate.h"

#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define convertValueTOAngle(value) ((double_t)((value - 2048.0) / 12.80)) // value_to_angle
#define convertAngleTOValue(value) ((int32_t)((value)*12.80 + 2048.0))    // angle_to_value

#define TO_INT16(a, b) ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))

#define SERVO_POSITION 0
#define SERVO__VELOCITY 1
#define SERVO_POSITION_ADDR 36
#define SERVO_POSITION_LEN 2

#define FSR_LEFT_ID 111
#define FSR_RIGHT_ID 112
#define FSR_ADDR 90
#define FSR_LEN 4

#define BASE_BOARD_ID 200
#define BASE_BOARD_ADDR 24
#define BASE_BOARD_LEN 56

extern double assembleOffset[SERVO_NUMBER_MAX];
extern int8_t assembleDirection[SERVO_NUMBER_MAX];

extern void LoadAssembleOffset();

DynamixelWorkbench dxlWb;
ServoInfo_s servoInfo;

bool Servo::initHandlers(void)
{
  bool result = false;
  const char *log = NULL;

  result = dxlWb.addSyncWriteHandler(servoInfo.idList[0], "Goal_Position", &log); // 至少存在一个舵机
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler failed, %s", log);
    return false;
  }

  result = dxlWb.addSyncWriteHandler(servoInfo.idList[0], "Moving_Speed", &log);
  if (result == false)
  {
    ROS_ERROR("addSyncWriteHandler failed, %s", log);
    return false;
  }

  result = dxlWb.initBulkRead(&log);
  if (result == false)
  {
    ROS_ERROR("initBulkRead failed, %s", log);
    return false;
  }

  result = dxlWb.initBulkWrite(&log);
  if (result == false)
  {
    ROS_ERROR("initBulkWrite failed, %s", log);
    return false;
  }

  return true;
}

void Servo::initPrarm(uint8_t *idList, uint8_t number)
{
  bool result = false;
  const char *log = NULL;
  int32_t kp = 30;

  for (uint8_t i = 0; i < number; i++)
  {
    result = dxlWb.writeRegister(idList[i], "P_gain", kp, &log);
    if (result == false)
    {
      ROS_WARN("param set failed of %d, %s", idList[i], log);
    }
  }
}

bool Servo::init(void)
{
  const char *log = NULL;
  bool result = false;

  dxlWb.scan(servoInfo.idList, &servoInfo.number, SERVO_NUMBER_MAX, &log);
  ROS_INFO("number of servo devices: %d", servoInfo.number);

  if (servoInfo.number > 0)
  {
    for (uint8_t i = 0; i < servoInfo.number; i++)
    {
      ROS_INFO("servoInfo.idList[%d]: %d", i, servoInfo.idList[i]);
      dxlWb.torqueOn(servoInfo.idList[i], &log);
    }

    result = initHandlers();
    if (result == false)
    {
      ROS_ERROR("initDxlHandlers failed!");
      return false;
    }

    initPrarm(servoInfo.idList, servoInfo.number);
  }
  else
  {
    ROS_ERROR("no dynamixel device found!");
    return false;
  }
  return true;
}

bool Servo::read(uint8_t id, jointParam_t *param)
{
  if ((id < 1) || (id > SERVO_NUMBER_MAX))
  {
    ROS_WARN("Servo::read, illegal parameter!");
    return false;
  }

  bool result = false;
  const char *log = NULL;
  int32_t getData;

  result = dxlWb.readRegister(id, "Present_Position", &getData, &log);
  if (result == false)
  {
    ROS_WARN("failed to read, ID is %d, %s", id, log);
    return false;
  }
  param->angle = convertValueTOAngle(getData - assembleOffset[id - 1]) * assembleDirection[id - 1];
  return true;
}

bool Servo::write(uint8_t id, jointParam_t param)
{
  if ((id < 1) || (id > SERVO_NUMBER_MAX))
  {
    ROS_WARN("Servo::write, illegal parameter!");
    return false;
  }

  bool result = false;
  const char *log = NULL;
  int32_t setData = convertAngleTOValue(param.angle * assembleDirection[id - 1]) + assembleOffset[id - 1];

  result = dxlWb.writeRegister(id, "Goal_Position", setData, &log);
  if (result == false)
  {
    ROS_WARN("failed to write, ID is %d, %s", id, log);
    return false;
  }
  return true;
}

bool Servo::bulkRead(uint8_t *idList, uint8_t number, jointParam_t *paramList)
{
  for (uint8_t i = 0; i < number; i++)
  {
    if ((idList[i] < 1) || (idList[i] > SERVO_NUMBER_MAX))
    {
      ROS_WARN("Servo::bulkRead, illegal parameter!");
      return false;
    }
  }

  bool result = false;
  const char *log = NULL;
  uint16_t addrList[number] = {0};
  uint16_t lenghtList[number] = {0};
  int32_t valueList[number * SERVO_POSITION_LEN] = {0};

  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < number; i++)
  {
    addrList[i] = SERVO_POSITION_ADDR;
    lenghtList[i] = SERVO_POSITION_LEN;

    result = dxlWb.addBulkReadParam(idList[i], addrList[i], lenghtList[i], &log);
    if (result == false)
    {
      ROS_WARN("addBulkReadParam failed: %s", log);
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_WARN("bulkRead failed: %s", log);
    return false;
  }
  result = dxlWb.getRawBulkReadData(idList, number, addrList, lenghtList, valueList, &log);
  if (result == false)
  {
    ROS_WARN("getRawBulkReadData failed: %s", log);
    return false;
  }

  uint16_t value = 0;
  for (uint8_t i = 0; i < number; i++)
  {
    value = DXL_MAKEWORD(valueList[i * 2], valueList[i * 2 + 1]);
    paramList[i].angle = convertValueTOAngle(value - assembleOffset[idList[i] - 1]) * assembleDirection[idList[i] - 1];
  }

  return true;
}

bool Servo::syncWrite(uint8_t *idList, uint8_t number, jointParam_t *paramList)
{
  for (uint8_t i = 0; i < number; i++)
  {
    if ((idList[i] < 1) || (idList[i] > SERVO_NUMBER_MAX))
    {
      ROS_WARN("Servo::syncWrite, illegal parameter!");
      return false;
    }
  }

  bool result = false;
  const char *log = NULL;
  int32_t valueList[number] = {0};

  for (uint8_t i = 0; i < number; i++)
  {
    valueList[i] = convertAngleTOValue(paramList[i].angle * assembleDirection[idList[i] - 1]) + assembleOffset[idList[i] - 1];
  }

  result = dxlWb.syncWrite(SERVO_POSITION, idList, number, valueList, 1, &log); //同步写指令
  if (result == false)
  {
    ROS_WARN("failed to syncWrite, %s", log);
    return false;
  }
  return true;
}

bool RobanFsr::read(uint8_t id, forceParam_t *param)
{
  if ((id < 111) || (id > 112))
  {
    ROS_WARN("Fsr::read, illegal parameter!");
    return false;
  }

  bool result = false;
  const char *log = NULL;
  uint32_t getData[FSR_LEN];

  result = dxlWb.readRegister(id, FSR_ADDR, FSR_LEN, getData, &log);
  if (result == false)
  {
    ROS_WARN("failed to read, ID is %d, %s", id, log);
    return false;
  }
  for (uint8_t i = 0; i < FSR_LEN; i++)
  {
    param->force[i].z = getData[i];
  }
  return true;
}

bool RobanFsr::bulkRead(uint8_t *idList, uint8_t number, forceParam_t *paramList)
{
  for (uint8_t i = 0; i < number; i++)
  {
    if ((idList[i] < 111) || (idList[i] > 112))
    {
      ROS_WARN("Fsr::bulkRead, illegal parameter!");
      return false;
    }
  }

  bool result = false;
  const char *log = NULL;
  uint16_t addrList[number] = {0};
  uint16_t lenghtList[number] = {0};
  int32_t valueList[number * FSR_LEN] = {0};

  dxlWb.clearBulkReadParam();
  for (uint8_t i = 0; i < number; i++)
  {
    addrList[i] = FSR_ADDR;
    lenghtList[i] = FSR_LEN;

    result = dxlWb.addBulkReadParam(idList[i], addrList[i], lenghtList[i], &log);
    if (result == false)
    {
      ROS_WARN("addBulkReadParam failed: %s", log);
      return false;
    }
  }
  result = dxlWb.bulkRead(&log);
  if (result == false)
  {
    ROS_WARN("bulkRead failed: %s", log);
    return false;
  }
  result = dxlWb.getRawBulkReadData(idList, number, addrList, lenghtList, valueList, &log);
  if (result == false)
  {
    ROS_WARN("getRawBulkReadData failed: %s", log);
    return false;
  }

  for (uint8_t i = 1; i <= number; i++)
  {
    for (uint8_t len = 1; len <= FSR_LEN; len++)
    {
      paramList[i - 1].force[len - 1].z = valueList[i * len - 1];
    }
  }

  return true;
}

bool RobanImu::read(uint8_t id, imuParam_t *param)
{
  if (id != BASE_BOARD_ID)
  {
    ROS_WARN("Imu::read, illegal parameter!");
    return false;
  }

  bool result = false;
  const char *log = NULL;
  uint32_t getData[BASE_BOARD_LEN];

  result = dxlWb.readRegister(id, BASE_BOARD_ADDR, BASE_BOARD_LEN, getData, &log);
  if (result == false)
  {
    ROS_WARN("failed to read, ID is %d, %s", id, log);
    return false;
  }

  uint8_t startAddr = BASE_BOARD_ADDR;
  double_t grayCoefficient = 2000.0 / 32768;
  double_t accCoefficient = 16.0 / 32768;
  param->angularVelocity.x = grayCoefficient * TO_INT16(getData[38 - startAddr], getData[39 - startAddr]);
  param->angularVelocity.y = grayCoefficient * TO_INT16(getData[40 - startAddr], getData[41 - startAddr]);
  param->angularVelocity.z = grayCoefficient * TO_INT16(getData[42 - startAddr], getData[43 - startAddr]);
  param->linearAcceleration.x = accCoefficient * TO_INT16(getData[44 - startAddr], getData[45 - startAddr]);
  param->linearAcceleration.y = accCoefficient * TO_INT16(getData[46 - startAddr], getData[47 - startAddr]);
  param->linearAcceleration.z = accCoefficient * TO_INT16(getData[48 - startAddr], getData[49 - startAddr]);

  return true;
}

bool DxlDevise::init()
{
  const char *portName = "/dev/ttyUSB0";
  int baudrate = 1000000;

  const char *log = NULL;
  bool result = false;

  result = dxlWb.init(portName, baudrate, &log); // initWorkbench
  if (result == false)
  {
    ROS_ERROR("open port failed, log: %s", log);
    return false;
  }
  else
    ROS_INFO("open port succeed, baudrate: %d", baudrate);

  if (Servo::init() == false)
  {
    return false;
  }
  
  LoadAssembleOffset();

  return true;
}

bool DxlDevise::ping(uint8_t id)
{
  bool result = false;
  const char *log = NULL;

  result = dxlWb.ping(id, &log);
  if (result == false)
  {
    ROS_WARN("failed to ping dxl: ID is %d, %s", id, log);
    return false;
  }
  return true;
}
