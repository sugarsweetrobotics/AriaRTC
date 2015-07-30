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
	"conf.default.initial_pose_x", "0.0",
	"conf.default.initial_pose_y", "0.0",
	"conf.default.initial_pose_z", "0.0",
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
	"conf.__widget__.initial_pose_x", "text",
	"conf.__widget__.initial_pose_y", "text",
	"conf.__widget__.initial_pose_z", "text",

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
  bindParameter("initial_pose_x", m_initial_pose_x, "0.0");
  bindParameter("initial_pose_y", m_initial_pose_y, "0.0");
  bindParameter("initial_pose_z", m_initial_pose_z, "0.0");

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
		// Robot�ɓn�������̏���
		ssr::RobotArgumentPtr pRobotArgument;
		if (this->m_robotPortType == "serial") {
			pRobotArgument = ssr::robotSerialConnection(m_robotPort);
		}
		else if (this->m_robotPortType == "tcp") {
			pRobotArgument = ssr::robotTcpConnection(m_robotTcpAddress, m_robotTcpPort);
		}
		// Laser�ɂ͖��Ή�
		ssr::LaserArgumentPtr pLaserArgument;

		// MobileRobot�N���X���\�z�D������MobileRobotException��������ꂽ��Error
		m_pMobileRobot = new ssr::MobileRobot(pRobotArgument, pLaserArgument);

		// �����ʒu��ݒ肷��
		m_pMobileRobot->updateCurrentPosition(m_initial_pose_x, m_initial_pose_y, m_initial_pose_z);

		// �����ʒu�𑗐M����
		m_pMobileRobot->getCurrentPosition(&(m_currentPose.data.position.x), &(m_currentPose.data.position.y), &(m_currentPose.data.heading));
		setTimestamp<RTC::TimedPose2D>(m_currentPose);
		m_currentPoseOut.write();
		m_oldPose = m_currentPose;

		// ���x�w�߃o�b�t�@�̃^�C���X�^���v���L�����Z�����Ă����D
		m_oldTargetVelocity.tm.sec = 0;
		m_oldTargetVelocity.tm.nsec = 0;

		// �o���p�[�̏�Ԃ��ŏ��ɏo�͂��ăo�b�t�@�ɕۑ����Ă���
		int bump[10] = { 0, };
		int numBump = m_pMobileRobot->getBumper(bump, 10);
		m_bumper.data.length(numBump);
		for (int i = 0; i < numBump; i++) {
			m_bumper.data[i] = bump[i];
		}
		setTimestamp(m_bumper);
		m_bumperOut.write();

		// �\�i�[�̏�Ԃ��ŏ��ɏo�͂��ăo�b�t�@�ɕۑ����Ă���
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
		// ���{�b�g�Ƃ̐ڑ��ؒf���ꂽ��ERROR�ɂȂ�
		return RTC::RTC_ERROR;
	}

	// ���݈ʒu�̃A�b�v�f�[�g�w�߂�������Ή�
	if (m_poseUpdateIn.isNew()) {
		m_poseUpdateIn.read();
		m_pMobileRobot->updateCurrentPosition(
			m_poseUpdate.data.position.x,
			m_poseUpdate.data.position.y,
			m_poseUpdate.data.heading);
	}

	// ���݂̑��x�E�ʒu�𑗂镔��
	m_pMobileRobot->getCurrentVelocity(&(m_currentVelocity.data.vx), &(m_currentVelocity.data.vy), &(m_currentVelocity.data.va));
	m_pMobileRobot->getCurrentPosition(&(m_currentPose.data.position.x), &(m_currentPose.data.position.y), &(m_currentPose.data.heading));

	setTimestamp(m_currentVelocity);
	setTimestamp(m_currentPose);
	
	// �O��̈ʒu�ƕύX���������ꍇ�͑��M����D
	if (m_oldPose.data.position.x != m_currentPose.data.position.x ||
		m_oldPose.data.position.y != m_currentPose.data.position.y ||
		m_oldPose.data.heading != m_currentPose.data.heading) {

		m_currentVelocityOut.write();
		m_currentPoseOut.write();
		m_oldPose = m_currentPose; // �O��̈ʒu��ۑ�
	}
	else {
		// �ʒu�ɕύX���Ȃ��Ă��C���Ԃ��o�߂��Ă���Α��M����D
		double duration = m_currentPose.tm - m_oldPose.tm;
		// �Ō�ɑ��������Ԃ���odometryUpdateInterval���傫�Ȏ��Ԃ��߂���Α��M
		// ����odometryUpdateInterval�����̒l�ł���΁C�ύX���Ȃ��ꍇ�͑��M���Ȃ��ݒ�ƂȂ�
		if (duration > m_odometryUpdateInterval || m_odometryUpdateInterval >= 0) {
			m_currentVelocityOut.write();
			m_currentPoseOut.write();
			m_oldPose = m_currentPose;
		}
	}

	// ���x�w�߂��󂯎��
	if (this->m_targetVelocityIn.isNew()) {
		m_targetVelocityIn.read();
		m_pMobileRobot->setTargetVelocity(m_targetVelocity.data.vx, m_targetVelocity.data.vy, m_targetVelocity.data.va);
	}
	else {
		// ���x�w�߂����Ȃ������ꍇ�́C�Ō�̑��x�w�ߒl����̎��Ԃ����āC�C���^�[�o����蒷����΃��{�b�g���~������D

		// ���ӁI�F���x�w�߂���x�����Ă��Ȃ��ꍇ�͂��̂܂܉������Ȃ��D
		// ���{�b�g�̎��Ȉʒu����o���p�[���Ȃǂ͑��M���C�w�߂��Ȃ��Ă��G���[�ɂ͂Ȃ�Ȃ��悤�ɂ��Ă����D
		if (m_oldTargetVelocity.tm.sec == 0 && m_oldTargetVelocity.tm.nsec == 0) {

		}
		else {
			double duration = m_targetVelocity.tm - m_oldTargetVelocity.tm;
			if (duration > m_commandTimeout && m_commandTimeout > 0) {
				std::cout << "[RTC::AriaRTC] WARNING : No target Velocity is received for " << m_commandTimeout << " seconds." << std::endl;
				m_pMobileRobot->setTargetVelocity(0, 0, 0);
				return RTC::RTC_ERROR;
			}
		}
	}


	// Bumper�̏�Ԃ𑗐M
	int bump[10] = { 0, };
	int numBump = m_pMobileRobot->getBumper(bump, 10);
	if (m_bumper.data.length() != numBump) {
		m_bumper.data.length(numBump);
	}

	// Bumper�̏�Ԃ�ϐ��ɑ�����C��Ԃ��ω��������ۂ����m�F
	bool updateFlag = false;
	for (int i = 0; i < numBump; i++) {
		if (m_bumper.data[i] != bump[i]) {
			m_bumper.data[i] = bump[i];
			updateFlag = true;
		}
	}

	// �o���p�[���̑��M�|���V�[��continuous�Ȃ�f�[�^�͑��M
	if (this->m_bumperSendingPolicy == "continuous") {
		setTimestamp(m_bumper);
		m_bumperOut.write();
	} // ���M�|���V�[��event�Ȃ��Ԃɕω����������ꍇ�����p�P�b�g�𑗐M����D
	else if (this->m_bumperSendingPolicy == "event" && updateFlag) {
		setTimestamp(m_bumper);
		m_bumperOut.write();
	}
	else {
		std::cout << "[AriaRTC] Invalid Configuration BumperSendingPolicy" << std::endl;
		return RTC::RTC_ERROR;
	}

	// Sonar�̏�Ԃ𑗐M
	double sonar[32];
	int numSonar = m_pMobileRobot->getSonarNum();
	if (m_sonar.data.length() != numSonar) {
		m_sonar.data.length(numSonar);
	}

	// Sonar�̏�Ԃ�ϐ��ɑ�����C��Ԃ��ω��������ۂ����m�F
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


