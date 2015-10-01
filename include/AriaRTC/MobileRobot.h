/**
 * File : MobileRobot.h
 * Date : 2015/07/29
 * Author : Yuki Suga
 * E-mail : ysuga@ysuga.net
 */

#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <exception>

class ArRobot;
class ArBumpers;
class ArLaser;
class ArSonarDevice;
class ArRobotConnector;
class ArFunctor;
class ArLaserConnector;
class LaserParam;
class ArArgumentParser;

namespace ssr {

	class MobileRobotException : public std::exception {
	public:
		MobileRobotException() throw() {}
		virtual ~MobileRobotException() {}
	};

#define DEFINE_EXCEPTION(name, msg) \
	class name : public MobileRobotException { \
	public: \
	     name  () throw() {} \
		 virtual ~  name  () {} \
		 const char* what() const { return msg; } \
	}


	DEFINE_EXCEPTION(InvalidArgumentException, "Invalid Argument for MobileRobot");
	DEFINE_EXCEPTION(ConnectionException, "Connection Error fro MobileRobot");

	class RobotArgument;

	class LaserArgument;

	typedef std::shared_ptr<RobotArgument> RobotArgumentPtr;

	typedef std::shared_ptr<LaserArgument> LaserArgumentPtr;

	class MobileRobot {
	public:
		static void init();
		static void exit();

	private:

		// Objects for ARIA
		// This header can not include ARIA.h so the objects must be declared as pointer.


		ArRobot* m_pRobot;
		ArLaser *m_pLaser;
		ArBumpers* m_pBumpers;
		ArSonarDevice* m_pSonarDevice;
		ArRobotConnector *m_pConnector;
		ArLaserConnector *m_pLaserConnector;
		ArFunctor* m_pConnectionLostHandler;
	public:
		bool connectionLostFlag;

		bool isConnectionLost() { return connectionLostFlag; }

	public:
		MobileRobot(RobotArgumentPtr pRobotArgument,
					LaserArgumentPtr pLaserArgument);
		~MobileRobot();

	private:
		std::shared_ptr<ArArgumentParser> initArgument(RobotArgumentPtr pRobotArgument, LaserArgumentPtr pLaserArgument);
		void connectLaser(std::shared_ptr<ArArgumentParser> argument);
		void initMotionTask();
	public:

		void onConnectionLost() {
			connectionLostFlag = true;
		}

	public:
		void setTargetVelocity(double t, double l, double r);
		void setTargetAccel(double t, double l, double r);
		void setTargetDecel(double t, double l, double r);
		void setMaxVelocity(double t, double l, double r);
		void setMaxAccel(double t, double l, double r);

		void getCurrentVelocity(double *t, double *l, double *r);
		void getCurrentAccel(double* t, double* l, double* r);
		void getBatteryVoltage(double *v);
		void setGain(int transkp, int transkv, int transki, int rotkp, int rotkv, int rotki);
		int  getBumper(int *bump, int numBump);
		void getTemperature(double *t);
		void getCurrentPosition(double *x, double *y, double *th);
		void updateCurrentPosition(double x, double y, double th);
		int  getSonarNum();
		double getSonarOutput(int index);
	};

	/**
	 * Robot configuration for ARIA library
	 */
	class RobotArgument {
	public:
		std::string portName;
		std::string tcpPort;
		std::string ipAddress;
		std::string protocol;
	public:
		RobotArgument() {}
	};

	static RobotArgumentPtr robotSerialConnection(const std::string portName) {
		RobotArgument* pArg = new RobotArgument();
		pArg->protocol = "serial";
		pArg->portName = portName;
		return RobotArgumentPtr(pArg);
	}

	static RobotArgumentPtr robotTcpConnection(std::string ipAddress, std::string port) {
		RobotArgument* pArg = new RobotArgument();
		pArg->tcpPort = port;
		pArg->ipAddress = ipAddress;
		pArg->protocol = "tcp";
		return RobotArgumentPtr(pArg);
	}

	/**
	 * Laser configuration for ARIA library.
	 * 
	 * ARIA library controls not only mobile robots but also many types of sensors.
	 */
	class LaserArgument {
	public:
		std::string type;
		std::string protocol;
		std::string portName;
		std::string tcpPort;
	public:
		LaserArgument(){};
		virtual ~LaserArgument() {};
	};

	static LaserArgumentPtr urg20(std::string portName) {
		LaserArgument* pArg = new LaserArgument();
		pArg->type = "urg2.0";
		pArg->protocol = "serial";
		pArg->portName = portName;
		return LaserArgumentPtr(pArg);
	}

	static LaserArgumentPtr laserTcpConnection(const std::string type, int port) {
		LaserArgument* pArg = new LaserArgument();
		std::ostringstream oss;
		oss << port;
		pArg->tcpPort = oss.str();
		pArg->type = type;
		pArg->protocol = "tcp";
		return LaserArgumentPtr(pArg);
	}
};
