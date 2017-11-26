/// FileName : 		CommandSender.cpp
/// 				implementation file
/// Description :	It supports command send interface for ZJUNlict,
///	Keywords :		fitting, send, interface
/// Organization : 	ZJUNlict@Small Size League
/// Author : 		cliffyin
/// E-mail : 		cliffyin@zju.edu.cn
///					cliffyin007@gmail.com
/// Create Date : 	2011-07-25
/// Modified Date :	2011-07-25 
/// History :

#include "commandsender.h"
#include <utils.h>
#include <PlayInterface.h>
#include <ConfigReader.h>

namespace {
	const double pi = Param::Math::PI;

	/// 平射分档参数
	float A_KICK[12];
	float B_KICK[12];   
	float C_KICK[12];
	int MIN_FLAT_KICK_POWER[12] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
	int MAX_FLAT_KICK_POWER[12] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127};

	/// 挑射分档参数
	float A_CHIP_KICK[12];
	float B_CHIP_KICK[12];
	float C_CHIP_KICK[12];
	const int MIN_CHIP_POWER = 25;
	int MIN_CHIP_KICK_POWER[12] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
	int MAX_CHIP_KICK_POWER[12] = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127};

	/// 平/挑射参数外部文件读入
	CR_DECLARE(ROBOT_ID)
	CR_DECLARE(APARAM_FLAT_KICK)
	CR_DECLARE(BPARAM_FLAT_KICK)
	CR_DECLARE(CPARAM_FLAT_KICK)
	CR_DECLARE(MINPARAM_FLAT_KICK_POWER)
	CR_DECLARE(MAXPARAM_FLAT_KICK_POWER)
	CR_DECLARE(APARAM_CHIP_KICK)
	CR_DECLARE(BPARAM_CHIP_KICK)
	CR_DECLARE(CPARAM_CHIP_KICK)
	CR_DECLARE(MINPARAM_CHIP_KICK_POWER)
	CR_DECLARE(MAXPARAM_CHIP_KICK_POWER)
}

CCommandSender::CCommandSender(void)
{
	_wireless = new CWirelessModule;
	CR_SETUP(KickParam, ROBOT_ID, CR_INT)
	CR_SETUP(KickParam, APARAM_FLAT_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, BPARAM_FLAT_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, CPARAM_FLAT_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, MINPARAM_FLAT_KICK_POWER, CR_INT)
	CR_SETUP(KickParam, MAXPARAM_FLAT_KICK_POWER, CR_INT)
	CR_SETUP(KickParam, APARAM_CHIP_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, BPARAM_CHIP_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, CPARAM_CHIP_KICK, CR_DOUBLE)
	CR_SETUP(KickParam, MINPARAM_CHIP_KICK_POWER, CR_INT)
	CR_SETUP(KickParam, MAXPARAM_CHIP_KICK_POWER, CR_INT)

	// ROBOT_ID 中保存了当前场上存在的车的车号
	int robots = VARSIZE(ROBOT_ID);
	int param_pairs = VARSIZE(BPARAM_FLAT_KICK);
	for (int i = 0 ; i < robots; ++ i) {
		int robot_num = VIVAR(ROBOT_ID)[i];
		A_KICK[i] = VDVAR(APARAM_FLAT_KICK)[robot_num-1];
		B_KICK[i] = VDVAR(BPARAM_FLAT_KICK)[robot_num-1];
		C_KICK[i] = VDVAR(CPARAM_FLAT_KICK)[robot_num-1];
		MIN_FLAT_KICK_POWER[i] = VIVAR(MINPARAM_FLAT_KICK_POWER)[robot_num-1];
		MAX_FLAT_KICK_POWER[i] = VIVAR(MAXPARAM_FLAT_KICK_POWER)[robot_num-1];
		A_CHIP_KICK[i] = VDVAR(APARAM_CHIP_KICK)[robot_num-1];
		B_CHIP_KICK[i] = VDVAR(BPARAM_CHIP_KICK)[robot_num-1];
		C_CHIP_KICK[i] = VDVAR(CPARAM_CHIP_KICK)[robot_num-1];
		MIN_CHIP_KICK_POWER[i] = VIVAR(MINPARAM_CHIP_KICK_POWER)[robot_num-1];
		MAX_CHIP_KICK_POWER[i] = VIVAR(MAXPARAM_CHIP_KICK_POWER)[robot_num-1];
	}

	// 默认机器人模式
	memset(_robotMode, MODE_NORMAL_ROBOT, sizeof(_robotMode));
	memset(_modeSetting, false, sizeof(_modeSetting));
}

CCommandSender::~CCommandSender(void)
{
	delete _wireless;
}

void CCommandSender::setMode(int robot_mode, int robot_num) 
{
	if (robot_num > 0) {
		// 前面已经设置过了，就不能再设置
		if (! _modeSetting[robot_num-1]) {
			_robotMode[robot_num-1] = robot_mode;
			_modeSetting[robot_num-1] = true;
		}
	} else {
		for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
			if (! _modeSetting[i]) {
				_robotMode[i] = robot_mode;
				_modeSetting[i] = true;
			}
		}
	}

	return ;
}

void CCommandSender::sendCommand()
{
	ROBOTCOMMAND robotCmd;
	for (int i = 1; i <= Param::Field::MAX_PLAYER; ++ i) {
		robotCmd.mode = _robotMode[i-1];
		SpeedTran(_currentCommand[i]._xSpeed, _currentCommand[i]._ySpeed, _currentCommand[i]._rSpeed, 0, false);
		command2RobotCmd(_currentCommand[i], robotCmd);

		// TODO 为何这里没有设置为真实的车号
		_wireless->doWirelessModule(i-1, robotCmd);
		_modeSetting[i-1] = false; // 设置复位
	}

	return ;
}

void CCommandSender::sendCommand(unsigned char robotIndex[])
{
	ROBOTCOMMAND robotCmd;
	for (int i = 1; i <= Param::Field::MAX_PLAYER; ++ i) {
		// 进行速度分解
		SpeedTran(_currentCommand[i]._xSpeed, _currentCommand[i]._ySpeed, _currentCommand[i]._rSpeed, 0, false);

		// 上层ActionModule模块里设置的_currentCommand值在这里被转换成无线模块能接受的robotCmd形式
		// 将实际世界中的变量值意义按与底层机器人硬件指定协议转换到指定范围内,发送出去
		command2RobotCmd(_currentCommand[i], robotCmd);
		
		// 进行模式设定
		robotCmd.mode = _robotMode[i-1];
		if (robotCmd.mode == MODE_DEBUG_PID_WRITE) {
			for (int j = 0; j < 4; j ++) {
				robotCmd.driver_p[j] = _currentParam[i-1]._p[j];
				robotCmd.driver_i[j] = _currentParam[i-1]._i[j];
				robotCmd.driver_d[j] = _currentParam[i-1]._d[j];
			}
		}

		// 对应的真实车号
		_realRobotIndex[i-1] = robotIndex[i-1]-1;
		// 串口发送
		_wireless->doWirelessModule(_realRobotIndex[i-1], robotCmd);

		// 设置复位
		_modeSetting[i-1] = false; 
	}

	return ;
}

void CCommandSender::command2RobotCmd(const CCommandSender::ActionCommand& cmd, ROBOTCOMMAND& robotCmd)
{
	double xtemp = cmd._xSpeed;
	double ytemp = cmd._ySpeed;

	bool Using2011proto = true;
	if (Using2011proto) {
		// 以下部分用于新协议[6/11/2011 cliffyin]
		// vx : 1cm/s		限速 400cm/s
		// vy : 1cm/s		限速 400cm/s
		// w  : 40rad/s		限速 12.75rad/s
		robotCmd.x = (abs(xtemp)>400) ? Utils::Sign(xtemp)*400 : xtemp;
		robotCmd.y = (abs(ytemp)>400) ? Utils::Sign(ytemp)*400 : ytemp;
		robotCmd.rotate = (abs(cmd._rSpeed*40) > 500 ) ? Utils::Sign(cmd._rSpeed)*500 : cmd._rSpeed*40;
		/*robotCmd.x = robotCmd.x/0.94;
		robotCmd.y = robotCmd.y/0.85;*/
	} else {
		// 以下部分用于老协议 [6/11/2011 cliffyin]
		// vx : cm / s	限速 4m/s
		// vy : cm / s  限速 4m/s
		// w  : 40 * rad / s 
		robotCmd.x = (abs(xtemp)/2/2 > 126) ? Utils::Sign(xtemp)*126 : xtemp/2/2;
		robotCmd.y = (abs(ytemp)/2/2 > 126) ? Utils::Sign(ytemp)*126 : ytemp/2/2;
		robotCmd.rotate = (abs(cmd._rSpeed)*10 > 126) ? Utils::Sign(cmd._rSpeed)*126 : cmd._rSpeed*10;
	}

	robotCmd.cb = cmd._dribble;
	//cout<<"Dribble:  "<<int(cmd._dribble)<<endl;
	for (int i = 0; i < 4; ++ i) {
		robotCmd.speed[i] = wheel_speed[i];
	}
	
	robotCmd.shoot = cmd._kick;

	bool isTestInfrared = false;
	if (isTestInfrared) {
		robotCmd.x = 0;
		robotCmd.y = 0;
		robotCmd.rotate = 0;
		robotCmd.shoot = 20;
		robotCmd.cb = 0;
	}

	robotCmd.radius = cmd._radius;
	robotCmd.angle = cmd._angle;
	robotCmd.gyro = cmd._gyro;
    robotCmd.stop=cmd._stop;
	return ;
}

void CCommandSender::setSpeed(int num, double xSpeed, double ySpeed, double rSpeed)
{
	if (num < 1 || num > Param::Field::MAX_PLAYER) {
		cout<<"CommandSender : setSpeed invalid vehicle!"<<endl;
		return;
	}

	_currentCommand[num]._xSpeed = xSpeed;
	_currentCommand[num]._ySpeed = ySpeed;
	_currentCommand[num]._rSpeed = rSpeed;


	return ;
}

void CCommandSender::setGyro(int num, unsigned char dribble, double angle, unsigned int radius, double rspeed)
{
	if (num < 1 || num > Param::Field::MAX_PLAYER) {
		cout<<"CommandSender : setDribble invalid vehicle!"<<endl;
		return;
	}
	
	if(angle != 0){
		_currentCommand[num]._gyro = true;
	} else{
		_currentCommand[num]._gyro = false;
	}
	_currentCommand[num]._dribble = dribble;
	_currentCommand[num]._angle = angle;
	_currentCommand[num]._radius = radius;
	_currentCommand[num]._rSpeed = rspeed;
}
void CCommandSender::setDribble(int num, unsigned char dribble)
{
	if (num < 1 || num > Param::Field::MAX_PLAYER) {
		cout<<"CommandSender : setDribble invalid vehicle!"<<endl;
		return;
	}
	//cout<<"Num:  "<<num<<"  dribble:  "<<int(dribble)<<endl;
	_currentCommand[num]._dribble = dribble;

	return ;
}

void CCommandSender::setKick(int num, double kick, double chip, double pass)
{
	if (num < 1 || num > Param::Field::MAX_PLAYER) {
		cout<<"CommandSender : setKick invalid vehicle!"<<endl;
		return;
	}

	// 平/挑射分档
	if (fabs(kick) < 0.01) {
		if(fabs(chip)>0) {
			unsigned int ckickpower = kickPower2Mode(num,chip,true);
			_currentCommand[num]._kick =  ckickpower| SHOOT_CHIP;
		} else {
			_currentCommand[num]._kick = 0;
		}
	} else {
		_currentCommand[num]._kick = kickPower2Mode(num, kick, false);
	}


	return ;
}

void CCommandSender::setPidParam(int num, unsigned int* proportion, unsigned int* intergrate, unsigned int* differential)
{
	for (int i = 0; i < 4; ++ i) {
		_currentParam[num-1]._p[i] = proportion[i];
		_currentParam[num-1]._i[i] = intergrate[i];
		_currentParam[num-1]._d[i] = differential[i];
	}

	return ;
}

unsigned int CCommandSender::kickPower2Mode(int num, double kick, bool chipkick)
{
	int shootpower = 0;
	int realNum = _realRobotIndex[num-1]; // 数组返回值0-5
	
	if (chipkick) {
		//shootpower = (int)(K_CHIP_KICK[realNum]* kick + B_CHIP_KICK[realNum]);
		shootpower = (int)(A_CHIP_KICK[realNum]*kick*kick + B_CHIP_KICK[realNum]*kick + C_CHIP_KICK[realNum]);
		if (shootpower > MAX_CHIP_KICK_POWER[realNum] || kick>400) {
			shootpower = MAX_CHIP_KICK_POWER[realNum];
		} else if (shootpower < MIN_CHIP_KICK_POWER[realNum]) {
			shootpower = MIN_CHIP_KICK_POWER[realNum];
		}

		//cout<<"chipShootPower:  "<<shootpower<<endl;
	} else {
		// TODO 二次函数拟合，A是二次项系数，B是一次项系数，C是常数项
		shootpower = (int)(0.00001*A_KICK[realNum]*kick*kick + B_KICK[realNum]*kick + C_KICK[realNum]);
		if (kick >= 9999) {
			shootpower = 127;
		}

		if (fabs(kick - 8888) < 0.5) {
			shootpower = 90;
			return shootpower;
		}
		if (shootpower > MAX_FLAT_KICK_POWER[realNum]) {
			shootpower = MAX_FLAT_KICK_POWER[realNum];
		} else if (shootpower < MIN_FLAT_KICK_POWER[realNum]) {
			shootpower = MIN_FLAT_KICK_POWER[realNum];
		}
	}

	return shootpower;
}

void CCommandSender::SpeedTran(double m_fX,double m_fY,double m_fZ,double m_fOrg,bool isNewRobot)
{
	//新车参数
	if (true == isNewRobot) {
		// 08年国内赛作出的新车
		t1 = 60;
		t2 = 300;
		t3 = 229;
		t4 = 131;
		l_radius=75.55/10;
	} else {
		// 07年国际赛做出的新车
		t1=57;
		t2=303;
		t3=225;
		t4=135;
		l_radius=75.55/10;
	}

	double g1,g2,g3,g4;

	m_fY=m_fY;
	m_fZ=m_fZ;
	m_fX=-m_fX;
	g1=pi*(90.0 + m_fOrg + t1)/180.0;
	g2=pi*(90.0 + m_fOrg + t2)/180.0;
	g3=pi*(90.0 + m_fOrg + t3)/180.0;
	g4=pi*(90.0 + m_fOrg + t4)/180.0;

	double a00 = cos(g1);	double a01 = sin(g1);  double a02 = l_radius;
	double a10 = cos(g2);	double a11 = sin(g2);  double a12 = l_radius;
	double a20 = cos(g3);	double a21 = sin(g3);  double a22 = l_radius;
	double a30 = cos(g4);	double a31 = sin(g4);  double a32 = l_radius;  
	
	wheel_speed[0] = (a00*m_fX + a01*m_fY + a02*m_fZ);
	wheel_speed[1] = (a10*m_fX + a11*m_fY + a12*m_fZ);
	wheel_speed[2] = (a20*m_fX + a21*m_fY + a22*m_fZ);
	wheel_speed[3] = (a30*m_fX + a31*m_fY + a32*m_fZ);

	return ;
}

//////////////////////////////////////////////////////////////////////////
// receive function
//////////////////////////////////////////////////////////////////////////
int CCommandSender::getRobotInfo(int num, ROBOTINFO* info)
{
	int realnum = PlayInterface::Instance()->getRealIndexByNum(num);
	CCommControl *pCommControl = _wireless->getControlComm();
	if (NULL != pCommControl) {
		return pCommControl->RequestSpeedInfo(realnum, info);
	} else {
		return -1;
	}

	return -1;	
}

void CCommandSender::setstop(int num, bool torf){
	if (num < 1 || num > Param::Field::MAX_PLAYER) {
		cout<<"CommandSender : setDribble invalid vehicle!"<<endl;
		return;
	}
	_currentCommand[num]._stop=torf;
}