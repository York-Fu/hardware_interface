
#include "ros/ros.h"
#include "../include/hardward_interface/robotInterface.h"

#define JOINT_NUM_MAX 22

#define FORCE_LEFT_FOOT_ID 111
#define FORCE_RIGHT_FOOT_ID 112

#define IMU_ID 200

void jointDemo()
{
  uint8_t idList[JOINT_NUM_MAX] = {
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
      13, 14, 15, 16, 17, 18, 19, 20, 21, 22
  };
  jointParam_t valueList[JOINT_NUM_MAX] = {0};

  robot.c_actuator.c_joint.f_bulkRead(idList, JOINT_NUM_MAX, valueList);
  for (uint8_t i = 0; i < JOINT_NUM_MAX; i++)
    ROS_INFO("angle of ID %d = %f", idList[i], valueList[i].angle);

  valueList[21].angle -= 10;
  robot.c_actuator.c_joint.f_write(22, valueList[21]);
  usleep(1000 * 1000);
  valueList[21].angle += 10;
  robot.c_actuator.c_joint.f_write(22, valueList[21]);
  usleep(1000 * 1000);

  valueList[21].angle -= 10;
  robot.c_actuator.c_joint.f_syncWrite(idList, JOINT_NUM_MAX, valueList);
  usleep(1000 * 1000);
  valueList[21].angle += 10;
  robot.c_actuator.c_joint.f_syncWrite(idList, JOINT_NUM_MAX, valueList);
}

void getForceDemo()
{
  uint8_t number = 2;
  uint8_t idList[number] = {FORCE_LEFT_FOOT_ID, FORCE_RIGHT_FOOT_ID};
  forceParam_t valueList[number] = {0};
  robot.c_sensor.c_force.f_bulkRead(idList, number, valueList);

  for (uint8_t i = 1; i <= number; i++)
  {
    ROS_INFO("force of ID %d: ", idList[i - 1]);
    for (uint8_t len = 1; len <= 4; len++)
    {
      ROS_INFO("force[%d] = %f", len - 1, valueList[i - 1].force[i * len - 1].z);
    }
  }
}

void getImuDemo()
{
  imuParam_t imuValue;
  robot.c_sensor.c_imu.f_read(IMU_ID, &imuValue);

  ROS_INFO("angular_velocity.x = %f", imuValue.angularVelocity.x);
  ROS_INFO("angular_velocity.y = %f", imuValue.angularVelocity.y);
  ROS_INFO("angular_velocity.z = %f", imuValue.angularVelocity.z);
  ROS_INFO("linear_acceleration.x = %f", imuValue.linearAcceleration.x);
  ROS_INFO("linear_acceleration.y = %f", imuValue.linearAcceleration.y);
  ROS_INFO("linear_acceleration.z = %f", imuValue.linearAcceleration.z);
}

void MainFunc()
{
  jointDemo();
  getForceDemo();
  getImuDemo();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hardInterface_node");
  ros::NodeHandle nodeHandle;

  robotInterfaceInit();
  MainFunc();

  return 0;
}