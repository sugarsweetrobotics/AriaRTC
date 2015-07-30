// -*- C++ -*-
/*!
 * @file  AriaRTC.h
 * @brief Mobile Robots ARIA library avialble robot RT-component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef ARIARTC_H
#define ARIARTC_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;
#include "MobileRobot.h"
/*!
 * @class AriaRTC
 * @brief Mobile Robots ARIA library avialble robot RT-component
 *
 * MobileRobot ARIA library RT-component. Basically, Mobile Robot
 * controller, but the library also available to control laser
 * sensors.
 *
 */
class AriaRTC
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  AriaRTC(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~AriaRTC();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 0
   */
  int m_debug;
  /*!
   * Serial Port of Robot.
   * - Name: robotPort robotPort
   * - DefaultValue: COM1
   */
  std::string m_robotPort;
  /*!
   * Type of Robot Connection. tcp or serial is available
   * - Name: robotPortType robotPortType
   * - DefaultValue: serial
   */
  std::string m_robotPortType;
  /*!
   * TCP Port number of Robot Connection. If robotPortType is
   * tcp, this value will be used.
   * - Name: robotTcpPort robotTcpPort
   * - DefaultValue: 8101
   */
  std::string m_robotTcpPort;
  /*!
   * Tcp address of robot to be connected. If robotPortType is
   * tcp, this will be used.
   * - Name: robotTcpAddress robotTcpAddress
   * - DefaultValue: localhost
   */
  std::string m_robotTcpAddress;
  /*!
   * Serial port number of laser sensor. If laserPortType is
   * serial, this will be used.
   * - Name: laserPort laserPort
   * - DefaultValue: COM2
   */
  std::string m_laserPort;
  /*!
   * Connection type of laser sensor. tcp or serial is available.
   * - Name: laserPortType laserPortType
   * - DefaultValue: serial
   */
  std::string m_laserPortType;
  /*!
   * Laser sensor type. Available value is none (for no sensor),
   * urg2.0 (Hokuyo URG 04LX SCIP2.0 protocol), lms2xx (SICK
   * LMS200 series), lms1xx (SICK LMS100 series).
   * - Name: laserType laserType
   * - DefaultValue: none
   */
  std::string m_laserType;
  /*!
   * If laserPortType is tcp, robot and laser is remotely
   * connected. This value defines the tcp port of remote laser.
   * - Name: laserTcpPort laserTcpPort
   * - DefaultValue: 8102
   */
  std::string m_laserTcpPort;
  /*!
   * Bumper data send policy. If event, the data is sent when the
   * bumper state is changed. If continuous, every on_execute
   * time the data is sent.
   * - Name: bumperSendingPolicy bumperSendingPolicy
   * - DefaultValue: event
   */
  std::string m_bumperSendingPolicy;
  /*!
   * The interval of timeout. If the targetVelocity is not sent
   * for this interval, the robot rtc will go to error state and
   * stop maneuvour.
   * - Name: commandTimeout commandTimeout
   * - DefaultValue: 3
   */
  int m_commandTimeout;
  /*!
  * This value indicates the maximum interval of odometry (currentPose)
  * output. This RTC measures the current position of this robot, but if
  * no difference to the previous position is seen, this robot do not 
  * output the value. If this value is set, RTC output the position if 
  * the output is resumed until this interval. If this value is negative,
  * RTC do not output until the difference is seen.
  * - Name: odometryUpdateInterval odometryUpdateInterval
  * - DefaultValue: 0.5
  */
  double m_odometryUpdateInterval;
  /*!
  * Initial Pose (z)
  * - Name: initial_poes_x initial_pose_x
  * - DefaultValue: 0.0
  */
  double m_initial_pose_x;
  /*!
  * Initial Pose (z)
  * - Name: initial_poes_y initial_pose_y
  * - DefaultValue: 0.0
  */
  double m_initial_pose_y;
  /*!
  * Initial Pose (z)
  * - Name: initial_poes_z initial_pose_z
  * - DefaultValue: 0.0
  */
  double m_initial_pose_z;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedVelocity2D m_targetVelocity;
  /*!
   * Target Velocity of Mobile Robot (vx, vy, vz) in robot
   * coordinate
   * - Unit: meter / sec, radian / sec
   */
  InPort<RTC::TimedVelocity2D> m_targetVelocityIn;
  RTC::TimedPose2D m_poseUpdate;
  /*!
   * If you want to update the internal value of current pose of
   * mobile robot (in global coordinates), put data to this port.
   * - Unit: meter, radian
   */
  InPort<RTC::TimedPose2D> m_poseUpdateIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_currentVelocity;
  /*!
   * Current Velocity of Mobile Robot (In robot coordinate)
   */
  OutPort<RTC::TimedVelocity2D> m_currentVelocityOut;
  RTC::TimedPose2D m_currentPose;
  /*!
   * Current Pose estimated with Robot (usually odometry) in
   * global coordinates.
   * - Unit: meter, radian
   */
  OutPort<RTC::TimedPose2D> m_currentPoseOut;
  RTC::RangeData m_range;
  /*!
   * Ranger Sensor if robot is implemented
   */
  OutPort<RTC::RangeData> m_rangeOut;
  RTC::TimedBooleanSeq m_bumper;
  /*!
   * Bump Sensors if implemented. The order of sensor is same as
   * the robot itself. Check the order with demo.exe program. If
   * true, collide. If false, not collide. With the configuration
   * bump policy will change the data sending policy. Please see
   * it.
   */
  OutPort<RTC::TimedBooleanSeq> m_bumperOut;
  RTC::TimedDoubleSeq m_sonar;
  /*!
  * Sonar Output (distance of each sensor)
  */
  OutPort<RTC::TimedDoubleSeq> m_sonarOut;
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

	 RTC::TimedPose2D m_oldPose;
	 RTC::TimedVelocity2D m_oldTargetVelocity;
  ssr::MobileRobot* m_pMobileRobot;
};


extern "C"
{
  DLL_EXPORT void AriaRTCInit(RTC::Manager* manager);
};

#endif // ARIARTC_H
