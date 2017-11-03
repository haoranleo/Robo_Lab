#include "DefPos2013.h"
#include "param.h"
#include "utils.h"
#include "WorldModel.h"
#include "BestPlayer.h"
#include "GDebugEngine.h"
#include <math.h>
#include "BallSpeedModel.h"
#include <TaskMediator.h>
#include "ChipBallJudge.h"
#include <atlstr.h>

namespace{
	const double GoalBuffer = 2;
	const double PLAYERSIZE = Param::Vehicle::V2::PLAYER_SIZE - 1.0; 
	CGeoPoint RLEFTGPOINT;
	CGeoPoint RRIGHTGPOINT;
	CGeoPoint RGOAL_LEFT_POS;
	CGeoPoint RGOAL_RIGHT_POS;
	CGeoPoint RGOAL_CENTRE_POS;
	CGeoPoint RleftGudgePoint;
	CGeoPoint RrightGudgePoint;
	CGeoPoint RleftMiddlePoint;
	CGeoPoint RrightMiddlePoint;
	double RgudgeRightDir = - Param::Math::PI/2 - atan(0.8);
	double RgudgeLeftDir = Param::Math::PI/2+ atan(0.8);
	double gudgeBuffer =Param::Math::PI * 5 / 180;
	enum MiddleSide{
		UP_SIDE = 0,
		MIDDLE_SIDE,
		DOWN_SIDE
	};
	const double RExtremeLeftDir = -Param::Math::PI/2 - atan(1.0);
	const double RExtremeRightDir = Param::Math::PI/2 + atan(1.0);
	const double PENALTYBUFFER = 12;
	
	//ofstream file("D:\\test.txt");
};

CDefPos2013::CDefPos2013()
{
	_defendPoints.leftD = CGeoPoint(-200*Param::Field::RATIO,-20*Param::Field::RATIO);
	_defendPoints.rightD = CGeoPoint(-200*Param::Field::RATIO,20*Param::Field::RATIO);
	_defendPoints.middleD = CGeoPoint(0,0);
	_defendPoints.singleD = CGeoPoint(-200*Param::Field::RATIO,0);
	_defendPoints.goalieD = CGeoPoint(-275*Param::Field::RATIO,0);
	_laststoredgoaliePoint = CGeoPoint(-275*Param::Field::RATIO,0);
	_RleftmostDir = 0;
	_RrightmostDir = 0;
	_RgoalieLeftDir = 0;
	_RgoalieRightDir = 0;
	_RleftgoalDir = 0;
	_RrightgoalDir = 0;
	_RmiddlegoalDir = 0.0;
	_RdefendDir = 0.0;
	_RdefendTarget = CGeoPoint(0,0);
	RLEFTGPOINT = CGeoPoint(Param::Field::PITCH_LENGTH/2-Param::Vehicle::V2::PLAYER_SIZE-2,Param::Field::GOAL_WIDTH/2+2);
	RRIGHTGPOINT = CGeoPoint(Param::Field::PITCH_LENGTH/2-Param::Vehicle::V2::PLAYER_SIZE-2,-Param::Field::GOAL_WIDTH/2-2);
	RGOAL_LEFT_POS = CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2 + GoalBuffer);
	RGOAL_RIGHT_POS = CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2 - GoalBuffer);
	RGOAL_CENTRE_POS = CGeoPoint(Param::Field::PITCH_LENGTH / 2,0);
	RleftGudgePoint = CGeoPoint(200*Param::Field::RATIO,160*Param::Field::RATIO);
    RrightGudgePoint = CGeoPoint(200*Param::Field::RATIO,-160*Param::Field::RATIO);
	RleftMiddlePoint = CGeoPoint(250*Param::Field::RATIO,200*Param::Field::RATIO);
	RrightMiddlePoint = CGeoPoint(250*Param::Field::RATIO,-200*Param::Field::RATIO);

	_lastCycle = 0;
}

CDefPos2013::~CDefPos2013()
{}

void CDefPos2013::generatePos(const CVisionModule* pVision){
	CGeoPoint RleftPoint;
	CGeoPoint RrightPoint;
	CGeoPoint RmiddlePoint;
	CGeoPoint RsinglePoint;
	CGeoPoint RgoaliePoint;
	CGeoPoint RsidePoint;
	const CGeoLine RtargetLine = DefendUtils::getDefenceTargetAndLine(_RdefendTarget,_RdefendDir);
	const CVector RleftgoalVector = RGOAL_LEFT_POS - _RdefendTarget;
	_RleftgoalDir = RleftgoalVector.dir();
	const CVector RrightgoalVector = RGOAL_RIGHT_POS - _RdefendTarget;
	_RrightgoalDir = RrightgoalVector.dir();
	const CVector RmiddlegoalVector = RGOAL_CENTRE_POS - _RdefendTarget;
	_RmiddlegoalDir = RmiddlegoalVector.dir();
	double RgudgeDir = Utils::Normalize(_RmiddlegoalDir + Param::Math::PI);
	static bool goalieFirst = false;
	static bool leftFirst = false;
	static bool defendSide = false;
	static bool defendSide2 = false;
	static posSide RtargetSide = POS_SIDE_MIDDLE;
	static MiddleSide RmiddleSide = MIDDLE_SIDE;
	CGeoPoint RballPos = DefendUtils::reversePoint(pVision->Ball().Pos());
	double RBallPosX = (-1)*pVision->Ball().Pos().x();
	double RBallPosY = (-1)*pVision->Ball().Pos().y();
	const MobileVisionT& ball = pVision->Ball();
	if (UP_SIDE == RmiddleSide){
		if (RBallPosX > -120*Param::Field::RATIO){
			RmiddleSide = MIDDLE_SIDE;
		}
	}else if (MIDDLE_SIDE == RmiddleSide){
		if (RBallPosX < -100*Param::Field::RATIO){
			RmiddleSide = UP_SIDE;
		}else if (RBallPosX >30*Param::Field::RATIO){
			RmiddleSide = DOWN_SIDE;
		}
	}else if (DOWN_SIDE == RmiddleSide){
		if (RBallPosX < 10*Param::Field::RATIO){
			RmiddleSide = MIDDLE_SIDE;
		}
	}
	//cout<<RgudgeDir*180/Param::Math::PI<<endl;
	if (POS_SIDE_MIDDLE == RtargetSide)
	{
		if (RgudgeDir < 0 && RgudgeDir > RgudgeRightDir + gudgeBuffer)
		{
			RtargetSide = POS_SIDE_RIGHT;
		} else if (RgudgeDir > 0 && RgudgeDir < RgudgeLeftDir - gudgeBuffer)
		{
			RtargetSide = POS_SIDE_LEFT;
		}
	} else if (POS_SIDE_RIGHT == RtargetSide)
	{
		if (RgudgeDir < RgudgeRightDir || RgudgeDir > 0)
		{
			RtargetSide = POS_SIDE_MIDDLE;
		}
	} else if (POS_SIDE_LEFT == RtargetSide)
	{
		if (RgudgeDir > RgudgeLeftDir || RgudgeDir < 0)
		{
			RtargetSide = POS_SIDE_MIDDLE;
		}
	}
	//cout<<TaskMediator::Instance()->leftBack()<<"  left"<<endl;
	//cout<<TaskMediator::Instance()->rightBack()<<"  right"<<endl;
	//cout<<TaskMediator::Instance()->advancer()<<"  advance"<<endl;
	//cout<<TaskMediator::Instance()->goalie()<<"  goalie"<<endl;
	//由于匹配要慢一帧，需要在单后卫时计算left和right，反之亦然
	int left = TaskMediator::Instance()->leftBack();
	int right = TaskMediator::Instance()->rightBack();
	if (TaskMediator::Instance()->leftBack()!=0 && TaskMediator::Instance()->rightBack()!=0){
			if (Utils::InOurPenaltyArea(pVision->Ball().Pos(),6)){
				//left,right防破防的车
				if (RBallPosX>250*Param::Field::RATIO){
					if (POS_SIDE_MIDDLE == RtargetSide){
						//守门员封球门连线
						RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
						RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir-0.1,POS_SIDE_LEFT,0);
						RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir+0.1,POS_SIDE_RIGHT,0);
						RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
					}else if (POS_SIDE_LEFT == RtargetSide){
						RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
						RleftPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir()+0.1,POS_SIDE_LEFT,0);
						RrightPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir()-0.1,POS_SIDE_RIGHT,0);
						RsinglePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,CVector(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_LEFT,0);
					}else{
						RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
						RleftPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir()+0.1,POS_SIDE_LEFT,0);
						RrightPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir()-0.1,POS_SIDE_RIGHT,0);
						RsinglePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,CVector(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_RIGHT,0);
					}
				}else{
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
					RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir-0.05,POS_SIDE_LEFT,0);
					RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir+0.05,POS_SIDE_RIGHT,0);
					RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
				}
			}else{
				//这里defendbasedir可能处理的略微有点问题
				double defendbasedir = 0;
				CGeoPoint RtempLeftPoint;
				CGeoPoint RtempRightPoint;
				if (DefendUtils::BallIsInShootingTarget()){
					defendbasedir = _RdefendDir;
				}else{
					defendbasedir = _RmiddlegoalDir ;
				}
				if (POS_SIDE_MIDDLE == RtargetSide){
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
					//double Rleftdir = ( (defendbasedir+ DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint)) > _RleftgoalDir)?(_RdefendDir+ DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint)):(_RleftgoalDir);
					//double Rrightdir = ( (defendbasedir- DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint)) > _RrightgoalDir)?_RrightgoalDir:(_RdefendDir- DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint));
					double Rleftdir = defendbasedir+ DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint);
					double Rrightdir = defendbasedir- DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint);
					RtempLeftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,Rleftdir+0.005,POS_SIDE_LEFT,0);
					RtempRightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,Rrightdir-0.005,POS_SIDE_RIGHT,0);
					if (RBallPosX > 100*Param::Field::RATIO){
						RtempLeftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,Rleftdir,POS_SIDE_LEFT,0);
						RtempRightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,Rrightdir,POS_SIDE_RIGHT,0);
					}
					//当球速比较小且朝着门里进的
					double ball2leftDir = (RtempLeftPoint - RballPos).dir();
					double ball2rightDir  = (RtempRightPoint - RballPos).dir();
					double preFrame = 0.15;
					bool cannotattach = false;
					if (RballPos.dist(RGOAL_CENTRE_POS)<preFrame*ball.Vel().mod()){
						cannotattach = true;
					}
					if (DefendUtils::BallIsInShootingTarget() && ball.Vel().mod() > 50*Param::Field::RATIO && Utils::AngleBetween(_RdefendDir,ball2leftDir,ball2rightDir,0)
						&& !cannotattach){
						RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,0);
						RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,0);
						//GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(150,0),"close mode");
					}else{
						RleftPoint = RtempLeftPoint;
						RrightPoint = RtempRightPoint;
					}
					RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
					//cout<<"middle"<<endl;
				}else if (POS_SIDE_LEFT == RtargetSide){
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
					RleftPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir()+0.03,POS_SIDE_LEFT,0);
					RrightPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir()-0.03,POS_SIDE_RIGHT,0);
					RsinglePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,CVector(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_RIGHT,0);
					 //cout<<"left"<<endl;
				}else if (POS_SIDE_RIGHT == RtargetSide){
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RmiddlegoalDir ,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
					RleftPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir()+0.03,POS_SIDE_LEFT,0);
					RrightPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir()-0.03,POS_SIDE_RIGHT,0);
					RsinglePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,CVector(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_LEFT,0);
					//cout<<"right"<<endl;
				}
				//GDebugEngine::Instance()->gui_debug_x(_defendPoints.leftD,COLOR_PURPLE);
				//GDebugEngine::Instance()->gui_debug_x(_defendPoints.rightD,COLOR_PURPLE);
				//GDebugEngine::Instance()->gui_debug_x(_defendPoints.goalieD,COLOR_PURPLE);
			}
				//若左右后卫距离较远，守门员防主朝向
			if ((pVision->OurPlayer(TaskMediator::Instance()->leftBack()).Pos().dist(DefendUtils::reversePoint(RleftPoint))>25
				||!pVision->OurPlayer(TaskMediator::Instance()->leftBack()).Valid()
				||!pVision->OurPlayer(TaskMediator::Instance()->rightBack()).Valid()
				||pVision->OurPlayer(TaskMediator::Instance()->rightBack()).Pos().dist(DefendUtils::reversePoint(RrightPoint))>25)){
				RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
				if (POS_SIDE_LEFT == RtargetSide){
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
				}else if (POS_SIDE_RIGHT == RtargetSide){
					RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
				}	
				//GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-75,0),"major defend",COLOR_WHITE);
			}
			//对球已经踢出，门将还在防球门连线的补充
			CVector RballVel = Utils::Polar2Vector(ball.Vel().mod(),Utils::Normalize(ball.Vel().dir() + Param::Math::PI));
			double RballVelDir = RballVel.dir(); 
			CVector Rball2LeftGoal = RGOAL_LEFT_POS - RballPos;
			CVector Rball2RightGoal = RGOAL_RIGHT_POS - RballPos;
			CVector Rball2CenterGoal = RGOAL_CENTRE_POS - RballPos;
			double Rball2LeftDir = Rball2LeftGoal.dir();
			double Rball2RightDir = Rball2RightGoal.dir();
			double Rball2CenterDir = Rball2CenterGoal.dir();
			//CGeoPoint tmpDefenderPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
			bool ballShooted = DefendUtils::getBallShooted() && ((ball.Vel().mod()>250 && Utils::InBetween(RballVelDir,Rball2LeftDir,Rball2RightDir)&& RBallPosX<0)||
				(ball.Vel().mod()>150-RBallPosX && Utils::InBetween(RballVelDir,Rball2LeftDir,Rball2RightDir)&& RBallPosX>=0 &&RBallPosX<=150)||
			(ball.Vel().mod()>100 && Utils::InBetween(RballVelDir,Rball2LeftDir,Rball2RightDir)&& RBallPosX>150));
			////GDebugEngine::Instance()->gui_debug_msg(DefendUtils::reversePoint(tmpDefenderPoint),"T",COLOR_WHITE);
			////cout<<"ballshooted is "<<ballShooted<<endl;
			////cout<<"ball speed is "<<ball.Vel().mod()<<endl;
			//static bool beginshoot = false;
			//static bool firstinflag = false;
			//static int lastcycle = 0;
			//if (beginshoot == false && ballShooted && firstinflag ==false){
			//	lastcycle = vision->Cycle();
			//	firstinflag = true;
			//}
			//if (beginshoot == false && ballShooted ){
			//	int cnt = vision->Cycle()-lastcycle;
			//	//cout<<"cnt is "<<cnt<<endl;
			//	if(cnt >=2 && ballShooted && RBallPosX<265){
			//		cnt = 0;
			//		GDebugEngine::Instance()->gui_debug_msg(DefendUtils::reversePoint(tmpDefenderPoint),"T",COLOR_YELLOW);
			//		double leftDefOkDist = pVision->OurPlayer(TaskMediator::Instance()->leftBack()).Pos().dist(tmpDefenderPoint);
			//		double rightDefOkDist = pVision->OurPlayer(TaskMediator::Instance()->rightBack()).Pos().dist(tmpDefenderPoint);
			//		//cout<<leftDefOkDist<<" "<<rightDefOkDist<<endl;
			//		if (leftDefOkDist >rightDefOkDist){
			//			leftFirst = false;
			//		}else{
			//			leftFirst = true;
			//		}
			//		if (POS_SIDE_LEFT == RtargetSide){
			//			leftFirst = true;
			//			if (leftDefOkDist>rightDefOkDist+30){
			//				leftFirst = false;
			//			}
			//		}else if (POS_SIDE_RIGHT == RtargetSide){
			//			leftFirst = false;
			//			if (rightDefOkDist>leftDefOkDist+30){
			//				leftFirst = true;
			//			}
			//		}
			//		if (ball.Vel().y() > 0){
			//			leftFirst = true;
			//		}else{
			//			leftFirst = false;
			//		}
			//		if (RBallPosY<0 && Utils::InBetween(RballVelDir,Rball2LeftDir,Rball2CenterDir)){
			//			leftFirst = true;
			//		}
			//		if (RBallPosY>0 && Utils::InBetween(RballVelDir,Rball2CenterDir,Rball2RightDir)){
			//			leftFirst = false;
			//		}
			//		beginshoot = true;
			//	}
			//}
			////double leftDefOkDist = pVision->OurPlayer(TaskMediator::Instance()->leftBack()).Pos().dist(tmpDefenderPoint);
			////double rightDefOkDist = pVision->OurPlayer(TaskMediator::Instance()->rightBack()).Pos().dist(tmpDefenderPoint);
			//////cout<<leftDefOkDist<<" "<<rightDefOkDist<<endl;
			////if (leftDefOkDist >rightDefOkDist){
			////	leftFirst = false;
			////}else{
			////	leftFirst = true;
			////}
			//if (!ballShooted && beginshoot == true){
			//	if (ball.VelX()>=0 && RballPos.dist(tmpDefenderPoint)>40){
			//		beginshoot = false;
			//		firstinflag= false;
			//	}
			///*	if (RballPos.dist(tmpDefenderPoint)>100){
			//		beginshoot = false;
			//		cout<<"end shoot"<<endl;
			//	}*/
			//}
		
			//int enemyNum = DefendUtils::getEnemyShooter();
			//double RenemyDir = Utils::Normalize(vision->TheirPlayer(enemyNum).Dir() + Param::Math::PI);	
			//if (ball.Vel().mod()<200 && !Utils::InBetween(RballVelDir,Rball2LeftDir,Rball2RightDir) && (fabs(Utils::Normalize(ball.Vel().dir()-(ball.Pos()-vision->TheirPlayer(enemyNum).Pos()).dir()))<Param::Math::PI/6 ||Utils::InBetween(RenemyDir,Rball2LeftDir,Rball2RightDir))){
			//	beginshoot = false;
			//}
		
			//if (beginshoot/*ballShooted &&Utils::AngleBetween(_RdefendDir,(RGOAL_LEFT_POS - RballPos).dir(),(RGOAL_RIGHT_POS - RballPos).dir(),0)*/){
			//	CString debug;
			//	GDebugEngine::Instance()->gui_debug_msg(DefendUtils::reversePoint(tmpDefenderPoint),"T",COLOR_YELLOW);
			//	debug.Format("leftFirst is %d",leftFirst);
			//	//cout<<"leftFirst is "<<leftFirst<<endl;
			//	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(150,0),debug,COLOR_WHITE);
			//	if (leftFirst == true){
			//		RleftPoint = tmpDefenderPoint;
			//		CVector transVector = Utils::Polar2Vector(PLAYERSIZE+3,Utils::Normalize(_RdefendDir + (-1 )* Param::Math::PI / 2));
			//		CGeoPoint transPoint = _RdefendTarget + transVector;
			//		RrightPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_RIGHT,0);
			//	}else{
			//		RrightPoint = tmpDefenderPoint;
			//		CVector transVector = Utils::Polar2Vector(PLAYERSIZE+3,Utils::Normalize(_RdefendDir + Param::Math::PI / 2));
			//		CGeoPoint transPoint = _RdefendTarget + transVector;
			//		RleftPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_LEFT,0);
			//	}
			//}

			if (RtargetSide == POS_SIDE_MIDDLE && ballShooted &&Utils::AngleBetween(_RdefendDir,(RGOAL_LEFT_POS - RballPos).dir(),(RGOAL_RIGHT_POS - RballPos).dir(),0)){
				RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
				GDebugEngine::Instance()->gui_debug_msg(DefendUtils::reversePoint(RgoaliePoint),"G",COLOR_WHITE);
			}
	}else if (TaskMediator::Instance()->singleBack()!=0)
	{
		//cout<<"2222222222"<<endl;
		if (POS_SIDE_RIGHT == RtargetSide){
			RsinglePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,CVector(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_LEFT,0);
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
			RleftPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir(),POS_SIDE_LEFT,0);
			RrightPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir(),POS_SIDE_RIGHT,0);
		}else if (POS_SIDE_LEFT == RtargetSide){
			RsinglePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,CVector(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_LEFT,0);
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
			RleftPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir(),POS_SIDE_LEFT,0);
			RrightPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir(),POS_SIDE_RIGHT,0);
		}else{
			RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
			RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_LEFT,0);
			RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_RIGHT,0);
			double goalieOKdist = pVision->OurPlayer(PlayInterface::Instance()->getNumbByRealIndex(TaskMediator::Instance()->goalie())).Pos().dist(DefendUtils::reversePoint(RgoaliePoint));
			double defOKdist = pVision->OurPlayer(TaskMediator::Instance()->singleBack()).Pos().dist(DefendUtils::reversePoint(RsinglePoint));
			if (defOKdist > goalieOKdist +2){
				goalieFirst = true;
			}else if (goalieOKdist > defOKdist + 5){
				goalieFirst = false;
			}
			if (goalieFirst == true){
				//门将封主朝向
				double Rtarget2goalieDir = CVector(RgoaliePoint - _RdefendTarget).dir();
				/*double shiftdist = 0;
				if (RBallPosX < -100){
					shiftdist = 0.02;
				}else if(RBallPosX > -100 && RBallPosX <150){
					shiftdist = 0.06/250*(RBallPosX+100)+0.02;
				}else{
					shiftdist = 0.08;
				}*/
				_RgoalieLeftDir = Rtarget2goalieDir+ DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint)/*+shiftdist*/;
				_RgoalieRightDir = Rtarget2goalieDir - DefendUtils::calcBlockAngle(_RdefendTarget,RgoaliePoint)/*-shiftdist*/;
				double leftRange =  _RleftgoalDir - _RgoalieLeftDir ;
				double rightRange =   _RgoalieRightDir - _RrightgoalDir;				
				if (leftRange > rightRange + 0.01)
				{
					defendSide = true;
				} else if (rightRange > leftRange + 0.01)
				{
					defendSide = false;
				}				
				//辅助防守次级判断
				static posSide RExtremeSide = POS_SIDE_MIDDLE;
				if (POS_SIDE_MIDDLE == RExtremeSide){
					if (RgudgeDir < 0 && RgudgeDir > RExtremeLeftDir + gudgeBuffer){
						RExtremeSide = POS_SIDE_LEFT;
					} else if (RgudgeDir > 0 && RgudgeDir < RExtremeRightDir - gudgeBuffer){
						RExtremeSide = POS_SIDE_RIGHT;
					}
				} else if (POS_SIDE_LEFT == RExtremeSide){
					if (RgudgeDir < RExtremeLeftDir || RgudgeDir > 0){
						RExtremeSide = POS_SIDE_MIDDLE;
					}
				} else if (POS_SIDE_RIGHT == RExtremeSide){
					if (RgudgeDir > RExtremeRightDir || RgudgeDir < 0){
						RExtremeSide = POS_SIDE_MIDDLE;
					}
				}
				if (POS_SIDE_LEFT == RExtremeSide){
					defendSide = true;
				} else if (POS_SIDE_RIGHT == RExtremeSide){
					defendSide = false;
				}
				posSide defSide = defendSide ? POS_SIDE_LEFT : POS_SIDE_RIGHT;
				double anotherDir = defendSide ? _RgoalieLeftDir : _RgoalieRightDir;
				//后卫站位点，当球速过快，不考虑封最大朝向，对球速线进行偏置
				RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,anotherDir,defSide,0);
				bool ballShooted = DefendUtils::getBallShooted();
				if (ballShooted){
					RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,defSide,0);
				}
				bool enemyPass = DefendUtils::getEnemyPass();
				if (enemyPass){
					if (pVision->Ball().VelY()>100*Param::Field::RATIO){
						RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,0);
					}else if (pVision->Ball().VelY()<-100*Param::Field::RATIO){
						RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,0);
					}
				}
				
			}else{
				double Rtarget2defDir = CVector(RsinglePoint- _RdefendTarget).dir();
				double _RdefLeftDir = Rtarget2defDir + DefendUtils::calcBlockAngle(_RdefendTarget,RsinglePoint);
				double _RdefRightDir = Rtarget2defDir - DefendUtils::calcBlockAngle(_RdefendTarget,RsinglePoint);
				double leftRange = _RleftgoalDir -_RdefLeftDir ;
				double rightRange = _RdefRightDir-_RrightgoalDir ;
				if (leftRange > rightRange + 0.01){
					defendSide2 = true;
				} else if (rightRange > leftRange + 0.01){
					defendSide2 = false;
				}
				//cout<<"defendSide is "<<defendSide2<<endl;
				posSide defSide = defendSide2 ? POS_SIDE_LEFT : POS_SIDE_RIGHT;
				double goalieDir = defendSide2 ? _RdefLeftDir:_RdefRightDir;
				int sideFactor;
				if (POS_SIDE_LEFT == defSide){
					sideFactor =1;
				} else{
					sideFactor = -1;
				}
				if (RBallPosX > 150*Param::Field::RATIO){
					goalieDir = Rtarget2defDir + sideFactor*0.1;
				}
				CVector transVector = Utils::Polar2Vector(PLAYERSIZE,Utils::Normalize(goalieDir + sideFactor * Param::Math::PI / 2));
				CGeoPoint transPoint = _RdefendTarget + transVector;
				//GDebugEngine::Instance()->gui_debug_line(DefendUtils::reversePoint(transPoint),DefendUtils::reversePoint(transPoint+Utils::Polar2Vector(200,goalieDir)),COLOR_WHITE);
				RgoaliePoint = DefendUtils::calcGoaliePointV2(transPoint,goalieDir,defSide,_laststoredgoaliePoint,1);
				//GDebugEngine::Instance()->gui_debug_msg(DefendUtils::reversePoint(RgoaliePoint),"W",COLOR_YELLOW);
			}
		}
	}else if (TaskMediator::Instance()->leftBack() == 0 && TaskMediator::Instance()->rightBack()==0 && TaskMediator::Instance()->singleBack() ==0)
	{
		//单守门员
		//cout<<"3333333333"<<endl;
		RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
		if (POS_SIDE_LEFT == RtargetSide){
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
		}else if (POS_SIDE_RIGHT == RtargetSide){
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
		}
		RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_LEFT,0);
		RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_RIGHT,0);
		RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
	}else{
		//其他情况如调用leftback和goalie，车看不见等
		//cout<<"4444444444"<<endl;
		//cout<<"left back is "<<TaskMediator::Instance()->leftBack()<<" right back is "<<TaskMediator::Instance()->rightBack()<<endl;
		RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,_laststoredgoaliePoint,0);
		RsinglePoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0);
		if (POS_SIDE_RIGHT == RtargetSide){
			RsinglePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,CVector(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_LEFT,0);
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_RIGHT,_laststoredgoaliePoint,0);
			RleftPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir(),POS_SIDE_LEFT,0);
			RrightPoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS-RrightGudgePoint).dir(),POS_SIDE_RIGHT,0);
		}else if (POS_SIDE_LEFT == RtargetSide){
			RsinglePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,CVector(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_LEFT,0);
			RgoaliePoint = DefendUtils::calcGoaliePointV2(_RdefendTarget,_RdefendDir,POS_SIDE_LEFT,_laststoredgoaliePoint,0);
			RleftPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir(),POS_SIDE_LEFT,0);
			RrightPoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS-RleftGudgePoint).dir(),POS_SIDE_RIGHT,0);
		}
		RleftPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_LEFT,0);
		RrightPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RmiddlegoalDir,POS_SIDE_RIGHT,0);
	}
	/*if (TaskMediator::Instance()->defendMiddle()!=0){
		if (UP_SIDE == RmiddleSide){
			RmiddlePoint = DefendUtils::calcMiddlePoint(_RdefendTarget,_RdefendDir,260*Param::Field::RATIO);
		}else if (MIDDLE_SIDE == RmiddleSide){
			double radius = ((-1)*RBallPosX + 160*Param::Field::RATIO)>260*Param::Field::RATIO?260*Param::Field::RATIO:((-1)*RBallPosX + 160*Param::Field::RATIO);
			if (radius > 260*Param::Field::RATIO){
				radius = 260*Param::Field::RATIO;
			}
			if (radius < 135*Param::Field::RATIO ){
				radius = 135*Param::Field::RATIO;
			}
			RmiddlePoint = DefendUtils::calcMiddlePoint(_RdefendTarget,_RdefendDir,radius);
		}else if(DOWN_SIDE == RmiddleSide){
			if (POS_SIDE_MIDDLE == RtargetSide){
				RmiddlePoint = DefendUtils::calcMiddlePoint(_RdefendTarget,_RdefendDir,135*Param::Field::RATIO);
			}else if (POS_SIDE_LEFT == RtargetSide){
				if (RBallPosX <= 230*Param::Field::RATIO){
					RmiddlePoint = DefendUtils::calcMiddlePoint(_RdefendTarget,_RdefendDir,140*Param::Field::RATIO);
				}else{
					RmiddlePoint = DefendUtils::calcMiddlePoint(RleftMiddlePoint,(RGOAL_CENTRE_POS-RleftMiddlePoint).dir(),140*Param::Field::RATIO);
				}
			}else if (POS_SIDE_RIGHT == RtargetSide){
				if (RBallPosX <= 230*Param::Field::RATIO){
					RmiddlePoint = DefendUtils::calcMiddlePoint(_RdefendTarget,_RdefendDir,140*Param::Field::RATIO);
				}else{
					RmiddlePoint = DefendUtils::calcMiddlePoint(RrightMiddlePoint,(RGOAL_CENTRE_POS-RrightMiddlePoint).dir(),140*Param::Field::RATIO);
				}
			}
		}
	}*/
	//cout<<RtargetSide<<endl;
	//当后卫站位点附近有我方车且对方车距离较远时强行将车移开一车身
	int oppnum =  BestPlayer::Instance()->theirFastestPlayerToBallList().at(0).num;	
	if (TaskMediator::Instance()->leftBack()!=0 && TaskMediator::Instance()->rightBack()!=0
		&& pVision->TheirPlayer(oppnum).Pos().dist(DefendUtils::reversePoint(RleftPoint))> 100
		&& pVision->TheirPlayer(oppnum).Pos().dist(DefendUtils::reversePoint(RrightPoint))> 100){
		double dist1 = checkCollision(TaskMediator::Instance()->leftBack(),DefendUtils::reversePoint(RleftPoint),pVision);
		double dist2 = checkCollision(TaskMediator::Instance()->rightBack(),DefendUtils::reversePoint(RrightPoint),pVision);
		//cout<<"dist1 = "<<dist1<<" dist2 = "<<dist2<<endl;
		if (dist1 >0 && dist2>0){
			if (dist1 <dist2){
				CVector transVector = Utils::Polar2Vector(PLAYERSIZE*3,Utils::Normalize(_RdefendDir + (-1 )* Param::Math::PI / 2));
				CGeoPoint transPoint = _RdefendTarget + transVector;
				GDebugEngine::Instance()->gui_debug_line(DefendUtils::reversePoint(transPoint),DefendUtils::reversePoint(transPoint+Utils::Polar2Vector(200,_RdefendDir)),COLOR_WHITE);
				RleftPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_LEFT);
				RrightPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_RIGHT);
			}else{
				CVector transVector = Utils::Polar2Vector(PLAYERSIZE*3,Utils::Normalize(_RdefendDir + Param::Math::PI / 2));
				CGeoPoint transPoint = _RdefendTarget + transVector;
				GDebugEngine::Instance()->gui_debug_line(DefendUtils::reversePoint(transPoint),DefendUtils::reversePoint(transPoint+Utils::Polar2Vector(200,_RdefendDir)),COLOR_WHITE);
				RleftPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_LEFT);
				RrightPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_RIGHT);
			}
		}else if (dist1> 0 && dist2 == -1){
			CVector transVector = Utils::Polar2Vector(PLAYERSIZE*3,Utils::Normalize(_RdefendDir + (-1 )* Param::Math::PI / 2));
			CGeoPoint transPoint = _RdefendTarget + transVector;
			GDebugEngine::Instance()->gui_debug_line(DefendUtils::reversePoint(transPoint),DefendUtils::reversePoint(transPoint+Utils::Polar2Vector(200,_RdefendDir)),COLOR_WHITE);
			RleftPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_LEFT);
			RrightPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_RIGHT);
		}else if (dist1 == -1 && dist2 >0){
			CVector transVector = Utils::Polar2Vector(PLAYERSIZE*3,Utils::Normalize(_RdefendDir + Param::Math::PI / 2));
			CGeoPoint transPoint = _RdefendTarget + transVector;
			GDebugEngine::Instance()->gui_debug_line(DefendUtils::reversePoint(transPoint),DefendUtils::reversePoint(transPoint+Utils::Polar2Vector(200,_RdefendDir)),COLOR_WHITE);
			RleftPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_LEFT);
			RrightPoint = DefendUtils::calcDefenderPointV2(transPoint,_RdefendDir,POS_SIDE_RIGHT);
		}
	}

	if (/*TaskMediator::Instance()->defendMiddle()!=0*/true){
		//if (Utils::InOurPenaltyArea(pVision->Ball().Pos(),6)){
		//	if (POS_SIDE_MIDDLE == RtargetSide){
		//		RmiddlePoint =  DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,3);
		//		//cout<<"1111111111"<<endl;
		//	}else if (POS_SIDE_LEFT == RtargetSide){
		//		RmiddlePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_MIDDLE,3);
		//		GDebugEngine::Instance()->gui_debug_line(RrightGudgePoint,RrightGudgePoint+Utils::Polar2Vector(100,(RGOAL_CENTRE_POS - RrightGudgePoint).dir()));
		//		//cout<<"2222222222"<<endl;
		//	}else{
		//		RmiddlePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_MIDDLE,3);
		//		//cout<<"3333333333"<<endl;
		//	}
		//}else{
			if (POS_SIDE_MIDDLE == RtargetSide){
				RmiddlePoint =  DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,3);
			}else if (POS_SIDE_LEFT == RtargetSide){
				RmiddlePoint = DefendUtils::calcDefenderPointV2(RleftGudgePoint,(RGOAL_CENTRE_POS - RleftGudgePoint).dir(),POS_SIDE_MIDDLE,3);
			}else{
				RmiddlePoint = DefendUtils::calcDefenderPointV2(RrightGudgePoint,(RGOAL_CENTRE_POS - RrightGudgePoint).dir(),POS_SIDE_MIDDLE,3);
			}
		}
	//}
	double RsideDir;
	CGeoPoint RsideTarget;
	const CGeoLine line = DefendUtils::getSideBackTargetAndLine(RsideTarget,RsideDir);
	RsidePoint = DefendUtils::calcDefenderPointV2(RsideTarget,RsideDir,POS_SIDE_MIDDLE,1);

	


	//任意球时守门员防挑球的特殊处理,跑上一帧的投影点
	//cout<<pVision->GetCurrentRefereeMsg()<<endl;
	/*if (pVision->GetCurrentRefereeMsg() =="theirDirectKick" || pVision->GetCurrentRefereeMsg() =="theirIndirectKick"){
	double ballRealDir = 0;
	CGeoPoint ballStartPoint = CGeoPoint(0,0);
	if (ChipBallJudge::Instance()->doJudge(pVision,ballRealDir,ballStartPoint)){
	CGeoLine ballMovingLine = CGeoLine(ballStartPoint,ballRealDir);
	CGeoPoint goaliePoint = ballMovingLine.projection(_laststoredgoaliePoint);
	if (Utils::InOurPenaltyArea(goaliePoint,10) && !Utils::OutOfField(goaliePoint,5)){
	RgoaliePoint = DefendUtils::reversePoint(goaliePoint);
	}
	}
	}*/
	_defendPoints.sideD = DefendUtils::reversePoint(RsidePoint);
	_defendPoints.leftD = DefendUtils::reversePoint(RleftPoint);
	_defendPoints.rightD = DefendUtils::reversePoint(RrightPoint);
	_defendPoints.middleD = DefendUtils::reversePoint(RmiddlePoint);
	_defendPoints.singleD = DefendUtils::reversePoint(RsinglePoint);
	_defendPoints.goalieD = DefendUtils::reversePoint(RgoaliePoint);
	_laststoredgoaliePoint = 	_defendPoints.goalieD;
	GDebugEngine::Instance()->gui_debug_x(_defendPoints.goalieD,COLOR_WHITE);
	GDebugEngine::Instance()->gui_debug_x(_defendPoints.leftD,COLOR_WHITE);
	GDebugEngine::Instance()->gui_debug_x(_defendPoints.rightD,COLOR_WHITE);
	//cout<<_defendPoints.goalieD<<endl;
	//以下为小场地的一传一射计算补偿角代码
	CGeoPoint startPoint = CGeoPoint(295,195);
	CGeoPoint receiver = CGeoPoint(100,0);
	CGeoPoint lastPoint = receiver + Utils::Polar2Vector(500,(Param::Math::PI*(10)/180));
	CGeoPoint endPoint = CGeoPoint(295,-195);
	//GDebugEngine::Instance()->gui_debug_line(startPoint,endPoint,COLOR_PURPLE);
	//GDebugEngine::Instance()->gui_debug_line(receiver,lastPoint,COLOR_PURPLE);
	CGeoLine line1 = CGeoLine(startPoint,endPoint);
	CGeoLine line2 = CGeoLine(receiver,lastPoint);
	CGeoLineLineIntersection intersect = CGeoLineLineIntersection(line1,line2);
	CGeoPoint passer1;
	passer1 = intersect.IntersectPoint();
	//cout<<passer1<<endl;
	//GDebugEngine::Instance()->gui_debug_x(passer1,COLOR_WHITE);
	CGeoPoint TargetPos[12] = {CGeoPoint(295,195),
	CGeoPoint(295,163.324),
	CGeoPoint(295,136.54),
	CGeoPoint(295,90.93),
	CGeoPoint(295,52.2501),
	CGeoPoint(295,17.0603),
	CGeoPoint(295,0),
	CGeoPoint(295,-17.0603),
	CGeoPoint(295,-52.2501),
	CGeoPoint(295,-90.93),
	CGeoPoint(295,-136.54),
	CGeoPoint(295,-195)};
	//CGeoPoint testPoint = DefendUtils::calcDefenderPointV2(_RdefendTarget,_RdefendDir,POS_SIDE_MIDDLE,0.3);
	//GDebugEngine::Instance()->gui_debug_x(testPoint,COLOR_WHITE);
	/*GDebugEngine::Instance()->gui_debug_x(TargetPos[0],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[1],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[2],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[3],COLOR_PURPLE);	
	GDebugEngine::Instance()->gui_debug_x(TargetPos[4],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[5],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[6],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[7],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[8],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[9],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[10],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_x(TargetPos[11],COLOR_PURPLE);
	GDebugEngine::Instance()->gui_debug_line(receiver,TargetPos[0]);
	GDebugEngine::Instance()->gui_debug_line(receiver,TargetPos[1]);
	GDebugEngine::Instance()->gui_debug_x(_defendPoints.goalieD,COLOR_WHITE);*/
	//file<<_defendPoints.leftD<<" "<<_defendPoints.rightD<<"  "<<_defendPoints.goalieD<<endl;
}
	
defend2013 CDefPos2013::getDefPos2013(const CVisionModule* pVision)
{
	if (pVision->Cycle() == _lastCycle){
		return _defendPoints;
	} else{
		_lastCycle = pVision->Cycle();
	}
	generatePos(pVision);
	return _defendPoints;
}

double CDefPos2013::checkCollision(int myself, CGeoPoint targetPoint, const CVisionModule* pVision)
{
	bool result = false;
	double dist = 100;
	for (int i = 1; i<= Param::Field::MAX_PLAYER; i++){
		if (i!= myself && i!=TaskMediator::Instance()->defendMiddle() 
			&& i!= TaskMediator::Instance()->leftBack()
			&& i!= TaskMediator::Instance()->rightBack()
			&& i!=TaskMediator::Instance()->sideBack()
			&& vision->OurPlayer(i).Pos().dist(targetPoint)< 20){
			result = true;
			//cout<<"i is "<<i<<endl;
			double temp = vision->OurPlayer(i).Pos().dist(targetPoint);
			if (temp<dist ){
				dist = temp;
			}
		}
	}
	if (result == true){
		return dist;
	}else{
		return -1;
	}
}