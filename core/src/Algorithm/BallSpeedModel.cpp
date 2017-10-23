#include "BallSpeedModel.h"
#include <param.h>
#include <math.h>
namespace{
	//double cal_acc = 31.68;
	double vel_factor = 0.9;
	double acc_factor = 1;
	double time_factor = 1;
	double posfortime_factor;
	int MAXERRORCOUNT = 15;
	CGeoPoint lastBallPos = CGeoPoint(0,0);
}


CBallSpeedModel::CBallSpeedModel()
{
	_ballVel = CVector(0,0);
	_lastBallVel = CVector(0,0);
	_ballPos = CGeoPoint(0,0);
	cal_acc = 10;
	_last_dist = 0;
}

CVector CBallSpeedModel::speedForTime(int frame, const CVisionModule* pVision )
{
	update(pVision);
	double my_time = frame / Param::Vision::FRAME_RATE;
	double vel_mod = _ballVel.mod() - cal_acc * my_time;
	int max_time = _ballVel.mod() / cal_acc ;
	if(max_time > my_time)
		return _ballVel * vel_mod / (vel_factor * _ballVel.mod());
	else
		return CVector(0,0);
}

CVector CBallSpeedModel::speedForDist(double dist, const CVisionModule* pVision )
{
	update(pVision);
	double vel_mod = _ballVel.mod() * _ballVel.mod() - 2 * cal_acc * dist;
	return _ballVel * vel_mod  / (vel_factor * _ballVel.mod());
}

double CBallSpeedModel::timeForDist(double dist, const CVisionModule* pVision )
{
	update(pVision);
	double final_dist = _ballVel.mod() * _ballVel.mod() / (2 * cal_acc);
	if(final_dist > dist){
		return (_ballVel.mod() - sqrt(_ballVel.mod() * _ballVel.mod() - 2 * cal_acc * dist)) / cal_acc;
	}
	else
		return -1;
}

double CBallSpeedModel::CalKickInitSpeed(const double dist)
{
	double vt = 100;
	if (dist < 100) {
		vt = 120;
	} else if (dist >= 100 && dist < 200) {
		vt = 200;
	} else if (dist >= 200 && dist < 300) {
		vt = 350;
	} else if (dist >= 300 && dist < 400) {
		vt = 350;
	} else if (dist >= 400) {
		vt = 350;
	}
	vt = vt+100;//让传球速度变小一点
	double ballVO = sqrt(vt * vt + 2 * 20 * dist);
	//迭代法计算初速度
	for (int i = 1;i<=3;i++){
		if(ballVO > 240){
			cal_acc = 10;
		} else if(ballVO > 190 && ballVO <= 240){
			cal_acc = ( 0.0025 * ballVO + 0.18 ) * 60;
		} else if(ballVO > 140 && ballVO <= 190){
			cal_acc = ( (0.0025 + ( 190 - ballVO) *0.00001)  * ballVO + 0.18 ) * 60;
		} else if (ballVO > 10 && ballVO <= 140){
			cal_acc = ( 0.003  * ballVO + 0.18 ) * 60;
		}
		ballVO = sqrt(vt * vt + 2 * cal_acc * dist);
	}
	return ballVO;
}

CGeoPoint CBallSpeedModel::posForTime(int frame, const CVisionModule* pVision )
{
	update(pVision);
	if(frame<0){
		return pVision->Ball().Pos();
	}
	double my_time = frame / Param::Vision::FRAME_RATE;
	double dist =  _ballVel.mod() * my_time - 0.5 * cal_acc * my_time * my_time;
	dist = (dist + _last_dist) / 2;
	_last_dist = dist;
	
	CGeoPoint final_pos;
	if( _ballVel.mod() < 0.5){
		final_pos = _ballPos;
	} else{
		final_pos = _ballPos + _ballVel * dist / _ballVel.mod();
	}
	return final_pos;
}

void CBallSpeedModel::update( const CVisionModule* pVision )
{
	_ballPos = pVision->Ball().Pos();
	lastBallPos = _ballPos;

	if(pVision->Ball().Vel().mod() > 240){
		cal_acc = 10;
	} else {
		double cal_norm = 0.34 * 60;
		//double cal_cali = 0.0025 * pVision->Ball().Vel().mod() + 0.18;
		/*if(cal_norm){
			
		}*/
		cal_acc = cal_norm;
	}
	_ballVel = pVision->Ball().Vel() * vel_factor;
}