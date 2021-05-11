#include "vrepBridge.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

namespace vrepBridge
{
#define SIM_JOINT_NUM 22
#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

  ros::Subscriber simStepDone_sub;
  ros::Subscriber simState_sub;
  ros::Publisher simStart_pub;
  ros::Publisher simStop_pub;
  ros::Publisher simPause_pub;
  ros::Publisher simEnSync_pub;
  ros::Publisher simTrigNext_pub;

  ros::Subscriber jointPos_sub;
  ros::Subscriber jointVel_sub;
  ros::Subscriber jointTorque_sub;
  ros::Subscriber lFootFT_sub;
  ros::Subscriber rFootFT_sub;
  ros::Subscriber imu_sub;

  ros::Subscriber basePos_sub;
  ros::Subscriber com_sub;
  ros::Subscriber comv_sub;
  ros::Subscriber cop_sub;

  ros::Publisher jointPos_pub;
  ros::Publisher jointVel_pub;
  ros::Publisher jointTorque_pub;
  ros::Publisher cop_pub;

  bool stepDone; //一步仿真完成标志

  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointPosition;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointVelocity;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointTorque;
  Eigen::Matrix<double, 6, 1> lFootFT; // torque[x y z] force[x y z]
  Eigen::Matrix<double, 6, 1> rFootFT;
  Eigen::Vector3d imuGyro;
  Eigen::Vector3d imuAcc;
  Eigen::Vector3d imuEuler;

  Eigen::Matrix<double, 7, 1> basePos; // quaternion[w x y z]  translation[x y z]
  Eigen::Vector3d baseEuler;
  Eigen::Vector3d com;
  Eigen::Vector3d comv;
  Eigen::Vector3d cop;

  bool mtxJointPosI;
  bool mtxJointVelI;
  bool mtxJointTorqueI;

  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointPosCmd;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointVelCmd;
  Eigen::Matrix<double_t, SIM_JOINT_NUM, 1> jointTorqueCmd;

  void simStateCallback(const std_msgs::Int32::ConstPtr &state)
  {
    // std_msgs::Bool start;
    // std::cout<<"here:" << std::endl;
    // if(state->data == 0)
    // {
    // 	start.data=1;
    // 	start_pub.publish(start);
    // }
  }

  void simStepDoneCallback(const std_msgs::Bool::ConstPtr &done)
  {
    stepDone = true;
  }

  void jointPosCallback(const std_msgs::Float64MultiArray::ConstPtr &pos)
  {
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
      jointPosition[i] = pos->data[i] * TO_DEGREE;
    mtxJointPosI = true;
  }

  void jointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &vel)
  {
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
      jointVelocity[i] = vel->data[i] * TO_DEGREE;
    mtxJointVelI = true;
  }

  void jointTorqueCallback(const std_msgs::Float64MultiArray::ConstPtr &torque)
  {
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
      jointTorque[i] = torque->data[i];
    mtxJointTorqueI = true;
  }

  void lFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      lFootFT[i] = ft->data[i];
  }

  void rFootForceCallback(const std_msgs::Float64MultiArray::ConstPtr &ft)
  {
    for (int i = 0; i < 6; ++i)
      rFootFT[i] = ft->data[i];
  }

  void imuCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    imuGyro << msg->data[0], msg->data[1], msg->data[2];
    imuAcc << msg->data[3], msg->data[4], msg->data[5];
    imuEuler << msg->data[6], msg->data[7], msg->data[8];
  }

  void basePosCallback(const std_msgs::Float64MultiArray::ConstPtr &pos)
  {
    for (int i = 0; i < 7; ++i)
      basePos[i] = pos->data[i];

    Eigen::Quaterniond Q(basePos[0], basePos[1], basePos[2], basePos[3]);
    baseEuler = Q.toRotationMatrix().eulerAngles(2, 1, 0);
    baseEuler(2) = baseEuler(2) > M_PI / 2 ? baseEuler(2) - M_PI : (baseEuler(2) < -M_PI / 2 ? baseEuler(2) + M_PI : baseEuler(2));
    baseEuler(1) = baseEuler(1) > M_PI / 2 ? M_PI - baseEuler(1) : (baseEuler(1) < -M_PI / 2 ? -M_PI - baseEuler(1) : baseEuler(1));
    // baseEuler(0) = 0.001 * (M_PI / 180.0); //set Yaw to 0.001 for stable reason
  }

  void comCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    for (int i = 0; i < 3; ++i)
      com[i] = msg->data[i];
  }

  void comvCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    for (int i = 0; i < 3; ++i)
      comv[i] = msg->data[i];
  }

  void copCallback(const std_msgs::Float64MultiArray::ConstPtr &fcop)
  {
    for (int i = 0; i < 3; ++i)
      cop[i] = fcop->data[i];
  }

  bool simInit()
  {
    ros::NodeHandle nh;
    simStepDone_sub = nh.subscribe<std_msgs::Bool>("simulationStepDone", 1, &simStepDoneCallback);
    simState_sub = nh.subscribe<std_msgs::Int32>("simulationState", 1, &simStateCallback);
    simStart_pub = nh.advertise<std_msgs::Bool>("startSimulation", 1);
    simStop_pub = nh.advertise<std_msgs::Bool>("stopSimulation", 1);
    simPause_pub = nh.advertise<std_msgs::Bool>("pauseSimulation", 1);
    simEnSync_pub = nh.advertise<std_msgs::Bool>("enableSyncMode", 1);
    simTrigNext_pub = nh.advertise<std_msgs::Bool>("triggerNextStep", 1);

    jointPos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/joint/position", 10, &jointPosCallback);
    jointVel_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/joint/velocity", 10, &jointVelCallback);
    jointTorque_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/joint/torque", 10, &jointTorqueCallback);
    lFootFT_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/force/lFoot", 10, &lFootForceCallback);
    rFootFT_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/force/rFoot", 10, &rFootForceCallback);
    imu_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/sensor/imu", 10, &imuCallback);

    basePos_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/base", 10, &basePosCallback);
    com_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/com", 10, &comCallback);
    comv_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/comv", 10, &comvCallback);
    cop_sub = nh.subscribe<std_msgs::Float64MultiArray>("/sim/state/cop", 10, &copCallback);

    jointPos_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/position", 1);
    jointVel_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/velocity", 1);
    jointTorque_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/joint/torque", 1);
    cop_pub = nh.advertise<std_msgs::Float64MultiArray>("/sim/param/cop", 1);

    ROS_INFO("waiting for connect to vrep...");
    while (ros::ok() &&
           (simStart_pub.getNumSubscribers() <= 0 ||
            simEnSync_pub.getNumSubscribers() <= 0 ||
            simTrigNext_pub.getNumSubscribers() <= 0))
      ; //等待发布者与接收者建立连接
    ROS_INFO("vrep connected.");

    simStart();
    return true;
  }

  void simTrigNext()
  {
    std_msgs::Bool msg;
    msg.data = 1;
    stepDone = false;
    simTrigNext_pub.publish(msg); //vrep仿真一步
    while (stepDone == false && ros::ok())
    {
      // ros::spinOnce();
      usleep(1);
    }
  }

  void simStart()
  {
    std_msgs::Bool msg;
    msg.data = 1;
    simEnSync_pub.publish(msg); //开启vrep同步模式
    simStart_pub.publish(msg);  //开始vrep仿真
    usleep(1000 * 200);

    for (uint16_t i = 0; i < 2; i++)
    {
      simTrigNext();
    }
  }

  bool simStop()
  {
    std_msgs::Bool msg;
    msg.data = 0;
    simEnSync_pub.publish(msg);
    msg.data = 1;
    simStop_pub.publish(msg); //停止vrep仿真
    usleep(1000 * 200);
    ROS_INFO("simulation in vrep stopped!");
    return true;
  }

  void simSendJointPos(Eigen::VectorXd position)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = position[i] * TO_RADIAN;
    }
    jointPos_pub.publish(msg);

    // while (jointPos_sub.getNumPublishers() <= 0)
    //   ;
    // while (lFootFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (rFootFT_sub.getNumPublishers() <= 0)
    //   ;
    // while (imu_sub.getNumPublishers() <= 0)
    //   ;
    // while (jointPos_pub.getNumSubscribers() <= 0)
    //   ;
  }

  void simSendJointVel(Eigen::VectorXd velocity)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = velocity[i] * TO_RADIAN;
    }
    jointVel_pub.publish(msg);
  }

  void simSendJointTorque(Eigen::VectorXd torque)
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(SIM_JOINT_NUM);
    for (int i = 0; i < SIM_JOINT_NUM; ++i)
    {
      msg.data[i] = torque[i];
    }
    jointTorque_pub.publish(msg);
  }

  bool simSetJointPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointPosCmd[ids[i] - 1] = position[i];
    }
    simSendJointPos(jointPosCmd);
    simTrigNext();
    return true;
  }

  bool simSetJointVelocity(std::vector<uint16_t> &ids, std::vector<double_t> &velocity)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointVelCmd[ids[i] - 1] = velocity[i];
    }
    simSendJointVel(jointVelCmd);
    simTrigNext();
    return true;
  }

  bool simSetJointTorque(std::vector<uint16_t> &ids, std::vector<double_t> &torque)
  {
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      jointTorqueCmd[ids[i] - 1] = torque[i];
    }
    simSendJointTorque(jointTorqueCmd);
    simTrigNext();
    return true;
  }

  bool simGetJointPosition(std::vector<uint16_t> &ids, std::vector<double_t> &position)
  {
    while (!mtxJointPosI && ros::ok())
    {
      usleep(1);
    }
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      position[i] = jointPosition[ids[i] - 1];
    }
    mtxJointPosI = false;
    return true;
  }

  bool simGeJointVelocity(std::vector<uint16_t> &ids, std::vector<double_t> &velocity)
  {
    while (!mtxJointVelI && ros::ok())
    {
      usleep(1);
    }
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      velocity[i] = jointVelocity[ids[i] - 1];
    }
    mtxJointVelI = false;
    return true;
  }

  bool simGeJointTorque(std::vector<uint16_t> &ids, std::vector<double_t> &torque)
  {
    while (!mtxJointTorqueI && ros::ok())
    {
      usleep(1);
    }
    for (uint16_t i = 0; i < ids.size(); i++)
    {
      torque[i] = jointTorque[ids[i] - 1];
    }
    mtxJointTorqueI = false;
    return true;
  }

  bool simGetForce(std::vector<ForceParam_t> &data)
  {
    data[0].force = {lFootFT[0], lFootFT[1], lFootFT[2]};
    data[0].moment = {lFootFT[3], lFootFT[4], lFootFT[5]};
    data[1].force = {rFootFT[0], rFootFT[1], rFootFT[2]};
    data[1].moment = {rFootFT[3], rFootFT[4], rFootFT[5]};
    return true;
  }

  bool simGetImu(std::vector<ImuParam_t> &data)
  {
    data[0].angularVel = {imuGyro[0], imuGyro[1], imuGyro[2]};
    data[0].linearAcc = {imuAcc[0], imuAcc[1], imuAcc[2]};
    data[0].eulerAngle = {imuEuler[0], imuEuler[1], imuEuler[2]};
    return true;
  }

  bool simGetSensor(SensorParam &sensor)
  {
    sensor.joint.position.assign(jointPosition.data(), jointPosition.data() + jointPosition.rows() + jointPosition.cols());
    sensor.joint.velocity.assign(jointVelocity.data(), jointVelocity.data() + jointVelocity.rows() + jointVelocity.cols());
    sensor.joint.torque.assign(jointTorque.data(), jointTorque.data() + jointTorque.rows() + jointTorque.cols());
    simGetForce(sensor.force);
    simGetImu(sensor.imu);
  }
} // namespace vrepBridge
