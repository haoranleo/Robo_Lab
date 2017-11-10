#include "GDebugEngine.h"
#include "skill/Factory.h"
#include <utils.h>
#include "param.h"
#include <cmath>
#include <utils.h>
#include <VisionModule.h>
#include "BallStatus.h"
#include "ShootBall.h"
#include "TouchKickPos.h"
#include "KickDirection.h"
#include "WorldModel.h"
#include <KickStatus.h>
#include "RobotSensor.h"
#include "DribbleStatus.h"

namespace
{
	enum ShootBall_State{
		TOUCH_KICK = 1,
		CHASE_KICK,
		INTERCEPT_KICK,
		INTER_TO_TOUCH
	};
	bool verBos = false;
	int Touch2ChaseCnt = 0;
	int Touch2InterCnt = 0;
	int Chase2TouchCnt = 0;
	int Chase2InterCnt = 0;
	int MaxOutInterDribbleCycle = 60;
};

CShootBall::CShootBall()
{
	_lastCycle = 0;
	_out_inter_cnt = 0;
	_out_inter = false;
}

// 传进来的朝向是一个比较稳定的朝向，适用于TouchKick，但是对
// 于Chase和Intercept来说需要时时计算朝向，Touch是静态动作，而
// Chase和Intercept是动态动作！！！！
void CShootBall::plan(const CVisionModule* pVision)
{
	const int runner = task().executor;
	int flags = task().player.flag;
	double finalDir = task().player.angle;
	const PlayerVisionT& self = pVision->OurPlayer(runner);
	const MobileVisionT& ball = pVision->Ball();
	const CGeoPoint rawBallPos=pVision->RawBall().Pos();
	CVector self2ball = rawBallPos - self.Pos();
	const double ballVelDir = ball.Vel().dir();
	double antiBallVelDir = Utils::Normalize(ballVelDir + Param::Math::PI);
	double ballSpeed = ball.Vel().mod();
	CGeoSegment shootSeg(self.Pos(), self.Pos() + Utils::Polar2Vector(1500, finalDir));
	CGeoPoint ballShootSegInterP = shootSeg.projection(rawBallPos);
	CGeoLine ballMovingLine = CGeoLine(rawBallPos,rawBallPos+Utils::Polar2Vector(1000,ball.Vel().dir()));
	CGeoPoint projMePos = ballMovingLine.projection(self.Pos());
	double me2ballMovingLineDist = projMePos.dist(self.Pos());


	const CGeoSegment ballMovingSeg = CGeoSegment(rawBallPos,rawBallPos+Utils::Polar2Vector(800,Utils::Normalize(ballVelDir)));	
	const CGeoPoint projMe = ballMovingSeg.projection(self.Pos());					//小车在球移动线上的投影点
	double projDist = projMe.dist(self.Pos());										//小车到投影点的距离
	const double ball2projDist = projMe.dist(rawBallPos);							//投影点到球的距离
	const bool meOnBallMovingSeg = ballMovingSeg.IsPointOnLineOnSegment(projMe);	//投影点是否在球速线段上面

	bool  isBallOnShootSeg = shootSeg.IsPointOnLineOnSegment(ballShootSegInterP);
	double a_m2t_2_m2b = fabs(Utils::Normalize(self2ball.dir() - finalDir)) * 180 / Param::Math::PI;
	double a_m2t_2_bsr = fabs(Utils::Normalize(antiBallVelDir - finalDir)) * 180 / Param::Math::PI;
	double a_m2t_2_bsl = fabs(Utils::Normalize(ballVelDir - finalDir)) * 180 / Param::Math::PI; 
	double a_m2b_2_bsr = fabs(Utils::Normalize(self2ball.dir() - antiBallVelDir)) * 180 / Param::Math::PI;
	double meBall_ballVelDiff=fabs(Utils::Normalize(self2ball.dir() - ballVelDir))* 180 / Param::Math::PI;

	CGeoSegment defendSeg=CGeoSegment(CGeoPoint(-Param::Field::PITCH_LENGTH/2,-Param::Field::PENALTY_AREA_WIDTH/3),CGeoPoint(-Param::Field::PITCH_LENGTH/2,Param::Field::PENALTY_AREA_WIDTH/3));
	CGeoLineLineIntersection inter=CGeoLineLineIntersection(defendSeg,ballMovingLine);
	bool canEnterDefendArea=false;
	if (ballSpeed>200)
	{
		if (inter.Intersectant()){
			CGeoPoint interPoint=inter.IntersectPoint();
			if (defendSeg.IsPointOnLineOnSegment(interPoint))
			{
				canEnterDefendArea=true;
			}
		}
	}
	canEnterDefendArea=canEnterDefendArea&&Utils::InOurPenaltyArea(self.Pos(),40);

	bool noInterInBack=false;
	if (flags & PlayerStatus::ACCURATELY){
		noInterInBack=true;
	}
	if (noInterInBack&&self.Pos().x()<-100
		&&fabs(ballVelDir)>Param::Math::PI*60/180&&fabs(ballVelDir)<Param::Math::PI*120/180)
	{
		noInterInBack=true;
	}else{
		noInterInBack=false;
	}
		
	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1
		|| BallStatus::Instance()->IsBallKickedOut(runner)
		|| _lastRunner != runner){
		setState(BEGINNING);
		Touch2ChaseCnt = 0;
		Touch2InterCnt = 0;
		Chase2TouchCnt = 0;
	}
	int new_state = state(), old_state = state();
	do {
		old_state = new_state;
		switch (old_state){
			case BEGINNING:
				{
					if( ballSpeed > 120 && a_m2t_2_m2b<65 && a_m2t_2_bsr<65 && self2ball.mod()>40 && a_m2b_2_bsr<20 
						&& ball2projDist/projDist>2 && projDist<75){
						if(verBos) cout<<"BEGINNING-->Touch"<<endl;
						new_state = TOUCH_KICK;
					} else if( !noInterInBack&&ballSpeed>120 &&(ballSpeed>160 && fabs(ballVelDir)/Param::Math::PI*180>105 ||meOnBallMovingSeg&&ball2projDist>80&&(ball2projDist/projDist>1.2))){
						if(verBos) cout<<"BEGINNING-->Inter"<<endl;
						new_state = INTERCEPT_KICK;
					} else{
						if(verBos) cout<<"BEGINNING-->Chase"<<endl;
						new_state = CHASE_KICK;
					}
					break;
				}
			case TOUCH_KICK:
				if( ballSpeed<100 && !RobotSensor::Instance()->IsInfraredOn(runner)
					||meBall_ballVelDiff<Param::Math::PI/2&&fabs(ballVelDir)<Param::Math::PI/2){
					if( Touch2ChaseCnt > 5){
						if(verBos) cout<<"Touch-->Chase"<<endl;
						new_state = CHASE_KICK;
						Touch2ChaseCnt = 0;
					} else{
						Touch2ChaseCnt++;
					}
				} else if ( !noInterInBack && (meBall_ballVelDiff<Param::Math::PI/3||a_m2t_2_bsr > 75 ||a_m2t_2_m2b>75)){
					if( Touch2InterCnt > 5){
						if(verBos) cout<<"Touch-->Inter"<<endl;
						new_state = INTERCEPT_KICK;
						Touch2InterCnt = 0;
					} else{
						Touch2InterCnt++;
					}
				}
				break;
			case CHASE_KICK:
				if( a_m2t_2_m2b<65 && a_m2t_2_bsr<65 && ballSpeed > 120 && self2ball.mod() > 50 && ball2projDist/projDist>1.8){
					if( Chase2TouchCnt > 1){
						if(verBos) cout<<"Chase-->Touch"<<endl;
						new_state = TOUCH_KICK;
						Chase2TouchCnt = 0;
					} else{
						Chase2TouchCnt++;
					}
				} else if(!noInterInBack && ballSpeed > 120 &&(fabs(ballVelDir)/Param::Math::PI*180>100 
					||meOnBallMovingSeg&&ball2projDist>80&&(ball2projDist/projDist>1.2))){
					if( Chase2InterCnt > 8){
						if(verBos) cout<<"Chase-->Inter"<<endl;
						new_state = INTERCEPT_KICK;
						Chase2InterCnt = 0;
					} else{
						Chase2InterCnt++;
					}
				} 
				break;
			case INTERCEPT_KICK:
				if(ballSpeed<100 && !RobotSensor::Instance()->IsInfraredOn(runner) || (a_m2t_2_m2b<45 && a_m2t_2_bsl<45)
					|| meBall_ballVelDiff<Param::Math::PI/3&&fabs(ballVelDir)<Param::Math::PI*90/180){
					if(verBos) cout<<"Inter-->Chase"<<endl;
					_out_inter = true;
					new_state = CHASE_KICK;
				} else if( canEnterDefendArea|| projDist<35 && a_m2t_2_m2b<65 && a_m2t_2_bsr<65 && self2ball.mod()>45 && a_m2b_2_bsr<15){
					if(verBos) cout<<"Inter-->INTER_TO_TOUCH"<<endl;
					new_state = INTER_TO_TOUCH;
					//double finalDirReverse = Utils::Normalize(finalDir+Param::Math::PI);	// 目标方向反向
					//fixPos = projMePos + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_SIZE, finalDirReverse);
				}
				break;
			case INTER_TO_TOUCH:
				if( ballSpeed<100 && !RobotSensor::Instance()->IsInfraredOn(runner)
					|| meBall_ballVelDiff<Param::Math::PI/3||a_m2t_2_bsr > 75 ||a_m2t_2_m2b>75){
					if(verBos) cout<<"INTER_TO_TOUCH-->Chase"<<endl;
					new_state = CHASE_KICK;
					//fixPos = CGeoPoint(0,0);
				}
				break;
			default:
				new_state = BEGINNING;
				break;
		}
	} while (false);
	setState(new_state);
	if(verBos) cout<<"shoot"<<new_state<<endl;
	if(_out_inter){
		_out_inter_cnt++;
		if (_out_inter_cnt < MaxOutInterDribbleCycle){
			DribbleStatus::Instance()->setDribbleCommand(runner,3);
		} else{
			_out_inter = false;
			_out_inter_cnt = 0;
			DribbleStatus::Instance()->setDribbleCommand(runner,0);
		}
	}
	

	bool changeKickDir=false;
	if (rawBallPos.x()<-230){
		CGeoSegment shootSeg=CGeoSegment(rawBallPos,rawBallPos+Utils::Polar2Vector(120,finalDir));
		for (int i=1;i<=Param::Field::MAX_PLAYER;i++)
		{
			if (pVision->OurPlayer(i).Valid())
			{
				CGeoPoint proj=shootSeg.projection(pVision->OurPlayer(i).Pos());
				double dist = proj.dist(pVision->OurPlayer(i).Pos());
				if (dist<15 && shootSeg.IsPointOnLineOnSegment(pVision->OurPlayer(i).Pos())){
					changeKickDir=true;
					break;
				}
			}
		}
	}

	flags = flags | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	KickDirection::Instance()->GenerateShootDir(runner,pVision->OurPlayer(runner).Pos());

	switch (state()){
		case TOUCH_KICK:
			{
				if (changeKickDir)
				{
					finalDir=finalDir-Utils::Sign(finalDir)*Param::Math::PI*20/180;
				}
				setSubTask(PlayerRole::makeItTouchKick(runner, finalDir, false, flags));
			}
			break;
		case CHASE_KICK:
			{
				finalDir = 	KickDirection::Instance()->getRawKickDir();
				if (changeKickDir)
				{
					finalDir=finalDir-Utils::Sign(finalDir)*Param::Math::PI*20/180;
				}
				setSubTask(PlayerRole::makeItChaseKickV2(runner, finalDir, flags));
			}
			break;
		case INTERCEPT_KICK:
			{
				finalDir = 	KickDirection::Instance()->getRawKickDir();
				if (changeKickDir)
				{
					finalDir=finalDir-Utils::Sign(finalDir)*Param::Math::PI*20/180;
				}
				setSubTask(PlayerRole::makeItInterKickV3(runner,finalDir,flags));
			}
			break;
		case INTER_TO_TOUCH:
			{
				TouchKickPos::Instance()->GenerateTouchKickPos(pVision,runner,finalDir);
				CGeoPoint targetPos = TouchKickPos::Instance()->getKickPos();
				if (changeKickDir)
				{
					finalDir=finalDir-Utils::Sign(finalDir)*Param::Math::PI*30/180;
				}
				setSubTask(PlayerRole::makeItGoto(runner, targetPos, finalDir, flags));
				break;
			}
		default:
			setSubTask(PlayerRole::makeItChaseKickV2(runner, finalDir, flags));
			break;
	}

	bool is_runner_dir_ok = WorldModel::Instance()->KickDirArrived(pVision->Cycle(),
		finalDir,task().player.kickprecision,runner);
	if (is_runner_dir_ok || NormalPlayUtils::faceTheirGoal(pVision,runner)&&!ball.Valid()) {
		if (!task().player.ischipkick) {
			KickStatus::Instance()->setKick(runner,task().player.kickpower);
		} else {
			KickStatus::Instance()->setChipKick(runner,task().player.chipkickpower);
		}
	}
	//cout<<"isdirOk"<<NormalPlayUtils::faceTheirGoal(pVision,runner)<<endl;
	GDebugEngine::Instance()->gui_debug_line( self.Pos(),self.Pos()+Utils::Polar2Vector(1000,finalDir),COLOR_WHITE);

	_lastCycle = pVision->Cycle();
	_lastRunner = runner;
	CPlayerTask::plan(pVision);
}
CPlayerCommand* CShootBall::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}

	return NULL;
}
