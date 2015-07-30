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
	"conf.default.odometryOutputInterval", "0.5",
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
	"conf.__widget__.odometryOutputInterval", "text",
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
    m_bumperOut("bumper", m_bumper),
	m_sonarOut("sonar", m_sonar)

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
  addOutPort("sonar", m_sonarOut);
  
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
  bindParameter("odometryUpdateInterval", m_odometryUpdateInterval, "0.5");
  // </rtc-template>
  

  ssr::MobileRobot::init();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t AriaRTC::onFinalize()
{
	ssr::MobileRobot::exit();
  return RTC::RTC_OK;
}

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
	try {
		ssr::RobotArgumentPtr pRobotArgument;
		if (this->m_robotPortType == "serial") {
			pRobotArgument = ssr::robotSerialConnection(m_robotPort);
		}
		else if (this->m_robotPortType == "tcp") {
			pRobotArgument = ssr::robotTcpConnection(m_robotTcpAddress, m_robotTcpPort);
		}

		ssr::LaserArgumentPtr pLaserArgument;

		m_pMobileRobot = new ssr::MobileRobot(pRobotArgument, pLaserArgument);

		m_pMobileRobot->getCurrentPosition(&(m_currentPose.data.position.x), &(m_currentPose.data.position.y), &(m_currentPose.data.heading));
		setTimestamp<RTC::TimedPose2D>(m_currentPose);
		m_currentPoseOut.write();
		m_oldPose = m_currentPose;

		m_oldTargetVelocity.tm.sec = 0;
		m_oldTargetVelocity.tm.nsec = 0;

		// バンパーの状態を最初に出力してバッファに保存しておく
		int bump[10] = { 0, };
		int numBump = m_pMobileRobot->getBumper(bump, 10);
		m_bumper.data.length(numBump);
		for (int i = 0; i < numBump; i++) {
			m_bumper.data[i] = bump[i];
		}
		setTimestamp(m_bumper);
		m_bumperOut.write();

		// ソナーの状態を最初に出力してバッファに保存しておく
		double sonar[12] = { 0, };
		int numSonar = m_pMobileRobot->getSonarNum();
		m_sonar.data.length(numSonar);
		for (int i = 0; i < numSonar; i++) {
			m_sonar.data[i] = m_pMobileRobot->getSonarOutput(i);
		}
		setTimestamp(m_sonar);
		m_sonarOut.write();

	}
	catch (ssr::MobileRobotException &ex) {
		std::cout << "[AriaRTC] MobileRobotException: " << ex.what() << std::endl;
		return RTC::RTC_ERROR;
	}

  return RTC::RTC_OK;
}


RTC::ReturnCode_t AriaRTC::onDeactivated(RTC::UniqueId ec_id)
{
	delete m_pMobileRobot;
	m_pMobileRobot = nullptr;

  return RTC::RTC_OK;
}


static double operator-(const RTC::Time& t1, const RTC::Time& t2) {
	return (t1.sec + t1.nsec / 1000000000.0) - (t2.sec + t2.nsec / 1000000000.0);
}

RTC::ReturnCode_t AriaRTC::onExecute(RTC::UniqueId ec_id)
{
	if (m_pMobileRobot->isConnectionLost()) {
		// ロボットが切断されたらERROR状態になる．周期処理は停止
		return RTC::RTC_ERROR;
	}

	// 現在位置のアップデート指令が来たら対応
	if (m_poseUpdateIn.isNew()) {
		m_poseUpdateIn.read();
		m_pMobileRobot->updateCurrentPosition(
			m_poseUpdate.data.position.x,
			m_poseUpdate.data.position.y,
			m_poseUpdate.data.heading);
	}

	// 現在のオドメトリ（速度・位置）を送る
	// まずロボットからオドメトリ情報を取得する
	m_pMobileRobot->getCurrentVelocity(&(m_currentVelocity.data.vx), &(m_currentVelocity.data.vy), &(m_currentVelocity.data.va));
	m_pMobileRobot->getCurrentPosition(&(m_currentPose.data.position.x), &(m_currentPose.data.position.y), &(m_currentPose.data.heading));

	setTimestamp(m_currentVelocity);
	setTimestamp(m_currentPose);
	//double dt = (m_currentPose.tm.sec + m_currentPose.tm.nsec / 1000000000.0) - (m_oldPose.tm.sec + m_oldPose.tm.nsec / 1000000000.0);

	// 前回の位置と変更があった場合は送信する．
	if (m_oldPose.data.position.x != m_currentPose.data.position.x ||
		m_oldPose.data.position.y != m_currentPose.data.position.y ||
		m_oldPose.data.heading != m_currentPose.data.heading) {

		m_currentVelocityOut.write();
		m_currentPoseOut.write();
		m_oldPose = m_currentPose; // 前回の位置を保存
		//m_OdometrySentTime = currentTime;
	}
	else {
		// 位置に変更がなくても，時間が経過していれば送信する．
		double duration = m_currentPose.tm - m_oldPose.tm;
		// 最後に送った時間からodometryUpdateIntervalより大きな時間が過ぎれば送信
		// もしodometryUpdateIntervalが負の値であれば，変更がない場合は送信しない設定となる
		if (duration > m_odometryUpdateInterval || m_odometryUpdateInterval >= 0) {
			m_currentVelocityOut.write();
			m_currentPoseOut.write();
			m_oldPose = m_currentPose;
		}
	}

	// 速度指令を受け取る
	if (this->m_targetVelocityIn.isNew()) {
		m_targetVelocityIn.read();
		m_pMobileRobot->setTargetVelocity(m_targetVelocity.data.vx, m_targetVelocity.data.vy, m_targetVelocity.data.va);
	}
	else {
		// 速度指令が来なかった場合は，最後の速度指令値からの時間を見て，インターバルより長ければロボットを停止させる．
		if (m_oldTargetVelocity.tm.sec == 0 && m_oldTargetVelocity.tm.nsec == 0) {

		}
		double duration = m_targetVelocity.tm - m_oldTargetVelocity.tm;
		if (duration > m_commandTimeout && m_commandTimeout > 0) {
			std::cout << "[RTC::AriaRTC] WARNING : No target Velocity is received for " << m_commandTimeout << " seconds." << std::endl;
			m_pMobileRobot->setTargetVelocity(0, 0, 0);
			return RTC::RTC_ERROR;
		}
	}


	// Bumperの状態を送信
	int bump[10] = { 0, };
	int numBump = m_pMobileRobot->getBumper(bump, 10);
	if (m_bumper.data.length() != numBump) {
		m_bumper.data.length(numBump);
	}

	// Bumperの状態を変数に代入しつつ，状態が変化したか否かを確認
	bool updateFlag = false;
	for (int i = 0; i < numBump; i++) {
		if (m_bumper.data[i] != bump[i]) {
			m_bumper.data[i] = bump[i];
			updateFlag = true;
		}
	}

	if (this->m_bumperSendingPolicy == "continuous") {
		setTimestamp(m_bumper);
		m_bumperOut.write();
	}
	else if (this->m_bumperSendingPolicy == "event" && updateFlag) {
		setTimestamp(m_bumper);
		m_bumperOut.write();
	}
	else {
		std::cout << "[AriaRTC] Invalid Configuration BumperSendingPolicy" << std::endl;
		return RTC::RTC_ERROR;
	}

	// Sonarの状態を送信
	double sonar[32];
	int numSonar = m_pMobileRobot->getSonarNum();
	if (m_sonar.data.length() != numSonar) {
		m_sonar.data.length(numSonar);
	}

	// Sonarの状態を変数に代入しつつ，状態が変化したか否かを確認
	updateFlag = false;
	for (int i = 0; i < numSonar; i++) {
		double distance = m_pMobileRobot->getSonarOutput(i);
		if (m_sonar.data[i] != distance) {
			m_bumper.data[i] = distance;
			updateFlag = true;
		}
	}
	
	if(updateFlag) {
		setTimestamp(m_bumper);
		m_bumperOut.write();
	}

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


