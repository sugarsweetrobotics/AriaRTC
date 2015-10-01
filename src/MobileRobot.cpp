/**
 * @file MobileRobot.cpp
 * @author Yuki Suga (ysuga@ysuga.net)
 * @copyright
 * Copyright (c) 2015, Sugar Sweet Robotics Co., Ltd. All rights reserved.
 * This program is made available under the terms of the GNU General Public License (GPL) version 3 
 * which accompanies this distribution, and is available at http://www.gnu.org/licenses/gpl-3.0.html
 */

#include <iostream>

#include <Aria.h>

#include <coil/Task.h>
#include <coil/TimeValue.h>
#include <coil/Time.h>

#include "MobileRobot.h"


using namespace ssr;


static bool initialize_flag = false;

void MobileRobot::init(void) {
	if (!initialize_flag) {
	    Aria::init();
		initialize_flag = true;
    }
}

void MobileRobot::exit(void) {
	if (initialize_flag){
		Aria::exit(0);
		initialize_flag = false;
	}
}


MobileRobot::MobileRobot(RobotArgumentPtr pRobotArgument, LaserArgumentPtr pLaserArgument) :
m_pLaser(nullptr), m_pRobot(nullptr),
connectionLostFlag(false) 
{
	if (pRobotArgument.get() == NULL) {
		throw new InvalidArgumentException();
	}

	std::shared_ptr<ArArgumentParser> pArgumentParser = initArgument(pRobotArgument, pLaserArgument);

	m_pRobot = new ArRobot();
	m_pConnector = new ArRobotConnector(pArgumentParser.get(), m_pRobot);
	if (!m_pConnector->connectRobot()) {
		m_pRobot->stop();
		throw new ConnectionException();
	}

	if (!m_pRobot->isConnected()) {
		m_pRobot->stop();
		throw new ConnectionException();
	}

	m_pBumpers = new ArBumpers();
	m_pRobot->addRangeDevice(m_pBumpers);

	m_pSonarDevice = new ArSonarDevice();
	m_pRobot->addRangeDevice(m_pSonarDevice);

	m_pRobot->runAsync(TRUE);
	ArUtil::sleep(1000);

	if (pLaserArgument.get() != nullptr) {
		connectLaser(pArgumentParser);
	}

	m_pRobot->lock();
	m_pRobot->enableMotors();
	m_pRobot->comInt(ArCommands::ENABLE, 1);
	m_pRobot->unlock();

	m_pConnectionLostHandler = new ArFunctorC<MobileRobot>(this, &MobileRobot::onConnectionLost);
	m_pRobot->addDisconnectOnErrorCB(m_pConnectionLostHandler, ArListPos::FIRST);

	// Update Small Configuration Value (on the fly update)
	// You can get and update the default value of these using ARCOS
	// http://robots.mobilerobots.com/wiki/ARCOS
	// To know the meanings of the values, see Robot's document.
	// m_pRobot->comInt(93, 128); // For TickMM (default 128 for 3DX)
	// m_pRobot->comInt(89, -50); // For DriftFactor (default 0 for P3DX)
	// m_pRobot->comInt(88, 16470); // For RevCount (default 16570 for P3DX)
	
}

MobileRobot::~MobileRobot() {
	if (m_pRobot) {
		m_pRobot->stopRunning(TRUE);
		delete m_pRobot;
		m_pRobot = nullptr;
	}
}




void MobileRobot::setTargetVelocity(double t, double l, double r){
	m_pRobot->setVel(t * 1000); // for Robot, Unit is mm/sec
	m_pRobot->setRotVel(r * 180 / M_PI);
	if (m_pRobot->hasLatVel()) {
		m_pRobot->setLatVel(l * 1000);
	}
}

void MobileRobot::setTargetAccel(double t, double l, double r){
	m_pRobot->setTransAccel(t);
	m_pRobot->setRotAccel(r);
	if (m_pRobot->hasLatVel()) {
		m_pRobot->setLatAccel(l);
	}
}

void MobileRobot::setTargetDecel(double t, double l, double r){
	m_pRobot->setTransDecel(t*1000);
	m_pRobot->setRotDecel(r * 180 / M_PI);
	if (m_pRobot->hasLatVel()) {
		m_pRobot->setLatDecel(l * 1000);
	}
}

void MobileRobot::setMaxVelocity(double t, double l, double r){
	m_pRobot->setTransVelMax(t * 1000);
	m_pRobot->setRotVelMax(r * 180 / M_PI);
	if (m_pRobot->hasLatVel()) {
		m_pRobot->setLatVelMax(l * 1000);
	}
}

void MobileRobot::setMaxAccel(double t, double l, double r){
	m_pRobot->setTransAccel(t);
	m_pRobot->setTransDecel(t);
	m_pRobot->setRotAccel(r);
	m_pRobot->setRotDecel(r);
	if (m_pRobot->hasLatVel()) {
		m_pRobot->setLatAccel(l);
		m_pRobot->setLatDecel(l);
	}
}

void MobileRobot::setGain(int transkp, int transkv, int transki, int rotkp, int rotkv, int rotki)
{
	m_pRobot->comInt(82, rotkp);
	m_pRobot->comInt(83, rotkv);
	m_pRobot->comInt(84, rotki);
	m_pRobot->comInt(85, transkp);
	m_pRobot->comInt(86, transkv);
	m_pRobot->comInt(87, transki);
}


void MobileRobot::getCurrentVelocity(double *t, double *l, double *r){
	*t = m_pRobot->getVel() / 1000.0;
	*l = m_pRobot->getLatVel() / 1000.0;
	*r = m_pRobot->getRotVel() / 180.0 * M_PI;
}

void MobileRobot::getCurrentAccel(double* t, double* l, double* r){
	*t = m_pRobot->getTransAccel() / 1000.0;
	*r = m_pRobot->getRotAccel() / 180.0 * M_PI;
	*l = m_pRobot->getLatAccel() / 1000.0;
}

void MobileRobot::getCurrentPosition(double *x, double *y, double *th) {
	ArPose pose = m_pRobot->getPose();
	*x = pose.getX() / 1000.0;
	*y = pose.getY() / 1000.0;
	*th = pose.getTh() / 180.0 * M_PI;
}

void MobileRobot::updateCurrentPosition(double x, double y, double th) {
	ArPose pose; pose.setX(x * 1000), pose.setY(y * 1000), pose.setThRad(th);
	m_pRobot->moveTo(pose);
}

void MobileRobot::getBatteryVoltage(double *v){
	*v = m_pRobot->getBatteryVoltage();
}

int MobileRobot::getBumper(int* bump, int numBump) {
	int counter = 0;
	int val = ((m_pRobot->getStallValue() & 0xff00) >> 8);
	int bit;
	for (int i = 0, bit = 2; i < m_pRobot->getNumFrontBumpers(); i++, bit *= 2)
	{
		if (val & bit)
			bump[counter] = 1;
		else
			bump[counter] = 0;
		counter++;
		if (counter >= numBump) {
			return counter;
		}
	}

	val = ((m_pRobot->getStallValue() & 0xff));
	for (int i = 0, bit = 2; i < m_pRobot->getNumRearBumpers(); i++, bit *= 2)
	{
		if (val & bit)
			bump[counter] = 1;
		else
			bump[counter] = 0;
		counter++;
		if (counter >= numBump) {
			return counter;
		}
	}

	return counter;
}

void MobileRobot::getTemperature(double *t){
	*t = m_pRobot->getTemperature();
}

int MobileRobot::getSonarNum(){
	return m_pRobot->getNumSonar();
}

double MobileRobot::getSonarOutput(int num){
	return m_pRobot->getSonarRange(num) / 1000.0;
}



//// private functions

std::shared_ptr<ArArgumentParser> MobileRobot::initArgument(RobotArgumentPtr pRobotArgument, LaserArgumentPtr pLaserArgument) {
	ArArgumentParser *parser;

	char buffer[255];
	strcpy(buffer, pRobotArgument->portName.c_str());
	
	std::vector<std::string> arguments;
	arguments.push_back("AriaRTC");

	if (pRobotArgument->protocol == "serial") {
		arguments.push_back("-rp");
		arguments.push_back(pRobotArgument->portName);
	}
	else if (pRobotArgument->protocol == "tcp") {
		arguments.push_back("-rh");
		arguments.push_back(pRobotArgument->ipAddress);
		arguments.push_back("-rrtp");
		arguments.push_back(pRobotArgument->tcpPort);
	}
	else {
		throw new InvalidArgumentException();
	}

	if (pLaserArgument.get() != nullptr) {
		std::cout << "[AriaRTC:MobileRobot::initializer] Currently AriaRTC do not support Laser Devices" << std::endl;
		//static char *argvLaser[] = { "AriaRTC", "-rp", buffer, "-lt", "lms2xx", "-lpt", "tcp", "-lp", "localhost" };

	}


	static ArArgumentBuilder builder;
	for (int i = 0; i < arguments.size(); i++) {
		builder.add(arguments[i].c_str());
	}
	parser = new ArArgumentParser(&builder);
	//parser->loadDefaultArguments();
	return std::shared_ptr<ArArgumentParser>(parser);
}

void MobileRobot::connectLaser(std::shared_ptr<ArArgumentParser> argument)
{
	/*
		m_pLaserConnector = new ArLaserConnector(parser, m_pRobot, m_pConnector);
		Aria::parseArgs();
		if (!m_pLaserConnector->connectLasers(false, false, true)) {
			delete parser; parser = NULL;
			m_pRobot->unlock();
			m_pRobot->stop();
			m_pRobot = NULL;
			failed = TRUE;
			return;
		}
		m_pRobot->lock();

		for (int i = 1; i <= 10; i++) {
			m_pLaser = m_pRobot->findLaser(i);
			if (m_pLaser != NULL) {
				break;
			}
		}
		m_pRobot->unlock();


		m_pLaser->lockDevice();
		if (!m_pLaser->isTryingToConnect()) {
			m_pLaser->asyncConnect();
		}
		m_pLaser->unlockDevice();
	*/
}
