// -*- C++ -*-
/*!
 * @file  AriaRTC.cpp
 * @brief Mobile Robots ARIA library avialble robot RT-component
 * @date $Date$
 *
 * $Id$
 */

#include "AriaRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* ariartc_spec[] =
  {
    "implementation_id", "AriaRTC",
    "type_name",         "AriaRTC",
    "description",       "Mobile Robots ARIA library avialble robot RT-component",
    "version",           "0.0.1",
    "vendor",            "SUGAR SWEET ROBOTICS",
    "category",          "MobileRobot",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.robotPort", "COM1",
    "conf.default.robotPortType", "serial",
    "conf.default.robotTcpPort", "8101",
    "conf.default.robotTcpAddress", "localhost",
    "conf.default.laserPort", "COM2",
    "conf.default.laserPortType", "serial",
    "conf.default.laserType", "none",
    "conf.default.laserTcpPort", "8102",
    "conf.default.bumperSendingPolicy", "event",
    "conf.default.commandTimeout", "3",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.robotPort", "text",
    "conf.__widget__.robotPortType", "spin.serial,tcp",
    "conf.__widget__.robotTcpPort", "text",
    "conf.__widget__.robotTcpAddress", "text",
    "conf.__widget__.laserPort", "text",
    "conf.__widget__.laserPortType", "spin.tcp,serial",
    "conf.__widget__.laserType", "spin.none,urg2.0,lms2xx,lms1xx",
    "conf.__widget__.laserTcpPort", "text",
    "conf.__widget__.bumperSendingPolicy", "spin.event,continuous",
    "conf.__widget__.commandTimeout", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
AriaRTC::AriaRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_targetVelocityIn("targetVelocity", m_targetVelocity),
    m_poseUpdateIn("poseUpdate", m_poseUpdate),
    m_currentVelocityOut("currentVelocity", m_currentVelocity),
    m_currentPoseOut("currentPose", m_currentPose),
    m_rangeOut("range", m_range),
    m_bumperOut("bumper", m_bumper)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
AriaRTC::~AriaRTC()
{
}



RTC::ReturnCode_t AriaRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetVelocity", m_targetVelocityIn);
  addInPort("poseUpdate", m_poseUpdateIn);
  
  // Set OutPort buffer
  addOutPort("currentVelocity", m_currentVelocityOut);
  addOutPort("currentPose", m_currentPoseOut);
  addOutPort("range", m_rangeOut);
  addOutPort("bumper", m_bumperOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("robotPort", m_robotPort, "COM1");
  bindParameter("robotPortType", m_robotPortType, "serial");
  bindParameter("robotTcpPort", m_robotTcpPort, "8101");
  bindParameter("robotTcpAddress", m_robotTcpAddress, "localhost");
  bindParameter("laserPort", m_laserPort, "COM2");
  bindParameter("laserPortType", m_laserPortType, "serial");
  bindParameter("laserType", m_laserType, "none");
  bindParameter("laserTcpPort", m_laserTcpPort, "8102");
  bindParameter("bumperSendingPolicy", m_bumperSendingPolicy, "event");
  bindParameter("commandTimeout", m_commandTimeout, "3");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AriaRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t AriaRTC::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AriaRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AriaRTC::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AriaRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t AriaRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void AriaRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(ariartc_spec);
    manager->registerFactory(profile,
                             RTC::Create<AriaRTC>,
                             RTC::Delete<AriaRTC>);
  }
  
};


