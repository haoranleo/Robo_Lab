#include "WorldModel.h"
#include "VisionModule.h"
#include "RobotCapability.h"
#include "ControlModel.h"
#include "BallStatus.h"
#include "RobotSensor.h"
#include "KickStatus.h"
#include "DribbleStatus.h"
#include "BestPlayer.h"
#include "GDebugEngine.h"
#include "PlayInterface.h"
#include "BufferCounter.h"

// 默认参数初始化
const int CWorldModel::myDefaultNum = 0;
const int CWorldModel::enemyDefaultNum = 0;

// 自己到球的矢量
const CVector CWorldModel::self2ball(int current_cycle,  int myNum,  int enemyNum) {
	static int last_cycle[Param::Field::MAX_PLAYER] = {-1,-1,-1,-1,-1,-1};
	static CVector _self2ball[Param::Field::MAX_PLAYER];

	if (last_cycle[myNum-1] < current_cycle) {
		_self2ball[myNum-1] = _pVision->Ball().Pos() - _pVision->OurPlayer(myNum).Pos();
		last_cycle[myNum-1] = current_cycle;
	}

	return _self2ball[myNum-1];
}

// 自己到球的距离
const double CWorldModel::self2ballDist(int current_cycle,  int myNum,  int enemyNum) {
	return self2ball(current_cycle, myNum, enemyNum).mod();	// 可以减少计算量
}

// 自己到球的角度
const double CWorldModel::self2ballDir(int current_cycle,  int myNum,  int enemyNum) {
	return self2ball(current_cycle, myNum, enemyNum).dir();	// 可以减少计算量
}

const string CWorldModel::CurrentRefereeMsg()
{
	return vision()->GetCurrentRefereeMsg();
}

bool CWorldModel::isPointInArea(const CGeoPoint p,const CGeoPoint leftUp,const CGeoPoint rightDown)
{
	return p.x() <= leftUp.x() && p.x() >= rightDown.x() && p.y() <= rightDown.y() && p.y() >= leftUp.y();
}

bool CWorldModel::isInImmortalArea(const CGeoPoint p)
{
	CGeoPoint ballPos = this->vision()->Ball().Pos();
	bool isBallInLeft = ballPos.y() < 0;
	double leftUpX = ballPos.x() - 15;
	double rightDownX = ballPos.x() - 160;
	double leftUpY = isBallInLeft ? (-Param::Field::PITCH_WIDTH/2+5) : (Param::Field::PITCH_WIDTH/2-80);
	double rightDownY = isBallInLeft ? (-Param::Field::PITCH_WIDTH/2+80) : (Param::Field::PITCH_WIDTH/2-5);
	CGeoPoint leftUp = CGeoPoint(leftUpX,leftUpY);
	CGeoPoint rightDown = CGeoPoint(rightDownX,rightDownY);
	return this->isPointInArea(p,leftUp,rightDown);
}

bool CWorldModel::isBallInLeftField()
{
	return this->vision()->Ball().Y() < 0;
}

bool CWorldModel::isInCorRushArea(const CGeoPoint p)
{
	CGeoPoint leftUp = CGeoPoint(180,-Param::Field::PITCH_WIDTH/2.0);
	CGeoPoint rightDown = CGeoPoint(-100,Param::Field::PITCH_WIDTH/2.0);
	return this->isPointInArea(p,leftUp,rightDown);
}

bool CWorldModel::isInMidRushArea(const CGeoPoint p)
{
	const MobileVisionT& ball = this->vision()->Ball();
	double leftUpX = 0,rightDownX = 0,leftUpY = 0,rightDownY = 0;
	if (this->isBallInLeftField())
	{
		leftUpX = ball.X()+100;
		rightDownX = ball.X()-30;
		leftUpY = ball.Y();
		rightDownY = ball.Y()+400;
	} else {
		leftUpX = ball.X()+100;
		rightDownX = ball.X()-30;
		leftUpY = ball.Y()-400;
		rightDownY = ball.Y();
	}
	CGeoPoint leftUp = CGeoPoint(leftUpX,leftUpY);
	CGeoPoint rightDown = CGeoPoint(rightDownX,rightDownY);
	return this->isPointInArea(p,leftUp,rightDown);
}