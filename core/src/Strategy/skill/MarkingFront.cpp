#include "MarkingFront.h"
#include "GDebugEngine.h"
#include <VisionModule.h>
#include "skill/Factory.h"
#include <utils.h>
#include <ControlModel.h>
#include <KickStatus.h>
#include <RobotSensor.h>
#include "utils.h"
#include "ShootRangeList.h"
#include <CommandFactory.h>
#include "TouchKickPos.h"
#include "BallSpeedModel.h"
#include <GDebugEngine.h>
#include "PlayInterface.h"
#include "NormalPlayUtils.h"
#include "WorldModel.h"
#include "BestPlayer.h"
#include "defence/DefenceInfo.h"

namespace
{
	//2.轨迹生成算法使用变量
	enum{
		Marking = 1,
		GotoFirstPos,
		Touch,
		MarkingAgain,

	};

	bool verBos=false;
	const double REF_AVOID_BALL = Param::Field::FREE_KICK_AVOID_BALL_DIST + Param::Vehicle::V2::PLAYER_SIZE*3.5;
	CGeoPoint ourGoal = CGeoPoint(-Param::Field::PITCH_LENGTH/2,0);
}

CMarkingFront::CMarkingFront()
{
	_lastCycle = 0;
	_stateCounter=0;
	_markEnemyNum=0;
	_marktoTouchCouter=0;
	_touchToBeginCouter=0;
	_markAgainToBeginCouter=0;
	_kickEnemyNum=0;
	_ballVelChangeCouter=0;
	_passDir=0;
	_ballVelDirChanged=false;
}

void CMarkingFront::resetState(){
	setState(BEGINNING);
	_kickEnemyNum=0;
	_markEnemyNum=0;
	_stateCounter=0;
	_marktoTouchCouter=0;
	_markAgainToBeginCouter=0;
	_touchToBeginCouter=0;
	_ballVelChangeCouter=0;
	_passDir=0;
	_ballVelDirChanged=false;
}

void CMarkingFront::plan(const CVisionModule* pVision){

	const int robotNum = task().executor;
	const double finalKickDir=task().player.angle;
	const int playerFlag = task().player.flag;
	const int markEnemyNum=task().ball.Sender;
	//true = horizal

	const MobileVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const double dist2Ball=me.Pos().dist(ball.Pos());


	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1 ){
		resetState();
		_lastCycle=0;
	}

	
	//cout<<"enemy"<<_kickEnemyNum<<endl;
	double ballVelDir = ball.Vel().dir();									// 球速方向
	double ballVelReverse = Utils::Normalize(ballVelDir+Param::Math::PI);	// 球速反向
	double kickDirReverse = Utils::Normalize(finalKickDir+Param::Math::PI);		// 目标方向反向
	double angleDiff_ballVelDir2kickDirReverse = fabs(Utils::Normalize(finalKickDir - ballVelReverse));

	if (state()!=Touch){
		_markEnemyNum=markEnemyNum;
		int temptNum=NormalPlayUtils::getTheirMostClosetoPos(pVision,ball.Pos());
		PlayerVisionT temptKickEnemy=pVision->TheirPlayer(temptNum);
		double hetoBallDir=(ball.Pos()-temptKickEnemy.Pos()).dir();
		if (fabs(Utils::Normalize(hetoBallDir-temptKickEnemy.Dir()))<Param::Math::PI*10/180)
		{
			_kickEnemyNum=temptNum;
		}
	}
	const PlayerVisionT kickEnemy=pVision->TheirPlayer(_kickEnemyNum);

	//待测试 将ball改成kickenemy
	CGeoLine passLine=CGeoLine(kickEnemy.Pos(),kickEnemy.Dir());
	CGeoSegment passSegment=CGeoSegment(ball.Pos(),ball.Pos()+Utils::Polar2Vector(800,kickEnemy.Dir()));
	GDebugEngine::Instance()->gui_debug_line(ball.Pos(),ball.Pos()+Utils::Polar2Vector(600,kickEnemy.Dir()),COLOR_BLACK);
	//cout<<_markEnemyNum<<endl;
	const PlayerVisionT he=pVision->TheirPlayer(_markEnemyNum);
	const double metoBallDir=(ball.Pos()-me.Pos()).dir();
	const double antiMetoBallDir=Utils::Normalize(metoBallDir+Param::Math::PI);
	const double hetoBallDir=(ball.Pos()-he.Pos()).dir();
	const double antiHetoBallDir=Utils::Normalize(hetoBallDir+Param::Math::PI);
	const CGeoSegment metoBallSeg = CGeoSegment(me.Pos()+Utils::Polar2Vector(8,metoBallDir),ball.Pos());
	const CGeoSegment hetoBallSeg = CGeoSegment(he.Pos()+Utils::Polar2Vector(8,hetoBallDir),ball.Pos());
	const double meBall_ballVelDiff=fabs(Utils::Normalize(metoBallDir - ballVelDir))* 180 / Param::Math::PI;
	double kickEnemyToBallDir=(ball.Pos()-kickEnemy.Pos()).dir();
	double diffAngleBallVel_HeDir=fabs(Utils::Normalize(ball.Vel().dir()-kickEnemy.Dir()));
	double diffAngleBallVel_HetoBall=fabs(Utils::Normalize(kickEnemyToBallDir-ball.Vel().dir()));
	double diffAngleHeDir_HetoBall=fabs(Utils::Normalize(kickEnemyToBallDir-kickEnemy.Dir()));
	//state jump
	//CGeoPoint meProjPassLine=passSegment.projection(me.Pos());
	//CGeoPoint heProjPassLine=passSegment.projection(he.Pos());
	CGeoLine ballLine = CGeoLine(ball.Pos(),ball.Pos()+Utils::Polar2Vector(100,ballVelDir));
	CGeoSegment ballSeg = CGeoSegment(ball.Pos(),ball.Pos()+Utils::Polar2Vector(800,ballVelDir));
	CGeoPoint meInBallVel=ballLine.projection(me.Pos());
	const double distMeProjBallVeltoMe=meInBallVel.dist(me.Pos());
	bool isBallCrossMe=!ballSeg.IsPointOnLineOnSegment(meInBallVel);


	CGeoPoint meProjhe=hetoBallSeg.projection(me.Pos());
	bool isMarked=hetoBallSeg.IsPointOnLineOnSegment(meProjhe)
		&&me.Pos().dist(he.Pos())<65||me.Pos().dist(_markPos)<18||_markEnemyNum==0;
	CGeoPoint heProjme=metoBallSeg.projection(he.Pos());
	bool isEnemyFrontMe=metoBallSeg.IsPointOnLineOnSegment(heProjme)
		&&Utils::pointToLineDist(he.Pos(),metoBallSeg)<15;

	bool isBallVelOk=fabs(Utils::Normalize(pVision->Ball(_lastCycle-1).Vel().dir()-ball.Vel().dir()))<Param::Math::PI*8/180;
	isBallVelOk=true;
	bool isBallPassed=//isBallVelOk&&
		//&&diffAngleBallVel_HeDir<Param::Math::PI*20/180&&diffAngleBallVel_HetoBall<Param::Math::PI*20/180
		diffAngleHeDir_HetoBall<Param::Math::PI*75/180&&
		kickEnemy.Pos().dist(ball.Pos())>15&&ball.Vel().mod()>50;
		//&&(fabs(Utils::Normalize(antiMetoBallDir-ballVelDir))<Param::Math::PI*45/180
		//||fabs(Utils::Normalize(antiHetoBallDir-ballVelDir))<Param::Math::PI*45/180);

	bool failTouch=ball.Vel().mod()<100 && !RobotSensor::Instance()->IsInfraredOn(robotNum)
		||meBall_ballVelDiff<Param::Math::PI/2 && ball.Vel().mod()<300;
		
		

	switch (state())
	{
	case BEGINNING:
		setState(Marking);
	case Marking:
		if (isMarked&&isBallPassed){
			_marktoTouchCouter++;
			//计算车开球瞬间车位置-车朝向连线与站位线的交点
			if (_marktoTouchCouter==1){
				_passDir=kickEnemy.Dir();
				CGeoLine markLine=CGeoLine(_markPos,Utils::Normalize(Param::Math::PI/2+_passDir));
				CGeoLineLineIntersection inter=CGeoLineLineIntersection(markLine,passLine);
				if (inter.IntersectPoint().dist(_markPos)>50)
				{
					setState(MarkingAgain);
				}else{
					if (inter.Intersectant())
					{
						_markPos=inter.IntersectPoint();
					}
					if (me.Pos().dist(_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse))>10)
					{
						setState(Touch);
					}else{
						setState(Touch);
					}
				}
				_marktoTouchCouter=0;
			}
		}
		break;
	case GotoFirstPos:
		if(verBos) cout<<"dist"<<me.Pos().dist(_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse))<<endl;
		if (me.Pos().dist(_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse))<5)
		{
			setState(Touch);
		}
		break;
	case Touch:
		if (isEnemyFrontMe||failTouch)
		{
			_touchToBeginCouter++;
			if (_touchToBeginCouter==180){
				setState(BEGINNING);
				_touchToBeginCouter=0;
			}
		}
		if (WorldModel::Instance()->IsBallKicked(robotNum))
		{
			setState(BEGINNING);
		}
		break;
	case MarkingAgain:
		_markAgainToBeginCouter++;
		if (_markAgainToBeginCouter==300)
		{
			setState(BEGINNING);
			_markAgainToBeginCouter=0;
		}
		break;
	}
	if(verBos) cout<<robotNum<<" stateJump: "<<state()<<" "<<isMarked<<" "<<isBallPassed<<endl;

	if (state()==Marking||state()==MarkingAgain)
	{
		_markPos=he.Pos()+Utils::Polar2Vector(35,hetoBallDir);
		const string refMsg = WorldModel::Instance()->CurrentRefereeMsg();
		const CGeoPoint ballPos = ball.Pos();
		CGeoPoint p1,p2;
		if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg || "theirKickOff" == refMsg || "gameStop" == refMsg)
		{
			if (_markPos.dist(ballPos) < REF_AVOID_BALL)
			{
				CGeoCirlce ballCircle = CGeoCirlce(ballPos,REF_AVOID_BALL);
				CGeoLine markLine = CGeoLine(_markPos,ourGoal);
				CGeoLineCircleIntersection markIntersect = CGeoLineCircleIntersection(markLine,ballCircle);
				if (markIntersect.intersectant())
				{
					p1 = markIntersect.point1();
					p2 = markIntersect.point2();
					if (p1.dist(ourGoal) < p2.dist(ourGoal))
					{
						_markPos = p1;
					} else _markPos = p2;
				}
			}
		}
	}

	CGeoPoint markTouchPos=_markPos;
	if (state()==Touch)
	{
		CGeoLine pointLine =CGeoLine(_markPos,Utils::Normalize(Param::Math::PI/2+_passDir));
		CGeoLineLineIntersection intersect = CGeoLineLineIntersection(pointLine,ballLine);

		double ball2playerdir = (pVision->OurPlayer(robotNum).Pos() - ball.Pos()).dir();
		double diff_ball2playerdir_ballveldir = fabs(Utils::Normalize(ballVelDir - ball2playerdir));

		double ball2playerdist = (pVision->OurPlayer(robotNum).Pos() - ball.Pos()).mod();
		
		//判断球速是否直线
		if (fabs(Utils::Normalize(pVision->Ball().Vel().dir()-pVision->Ball(_lastCycle-2).Vel().dir()))>Param::Math::PI*3.5/180)
		{
			_ballVelChangeCouter++;
			_ballVelChangeCouter=min(_ballVelChangeCouter,4);
		}else{
			_ballVelChangeCouter--;
			_ballVelChangeCouter=max(_ballVelChangeCouter,0);
		}
		if (_ballVelChangeCouter==3){
			_ballVelDirChanged=true;		
		}
		if (_ballVelChangeCouter==0)
		{
			_ballVelDirChanged=false;
		}
		if (verBos) cout<<"ballVel"<<_ballVelDirChanged<<" "<<_ballVelChangeCouter<<endl;
			
		if(intersect.Intersectant()){
			CGeoPoint interP = intersect.IntersectPoint();
			if (!_ballVelDirChanged&&ball2playerdist<300||ball2playerdist<50)
			{	
				if (interP.dist(_markPos)<35)
				{
					markTouchPos=interP;
				}		
			}else
			{
				double compensateDist=5;
				CVector compensateVector=(interP-_markPos);
				markTouchPos=_markPos+compensateVector/compensateVector.mod()*compensateDist;
			}
		}
	}
	
	GDebugEngine::Instance()->gui_debug_line(kickEnemy.Pos(),kickEnemy.Pos()+Utils::Polar2Vector(800,kickEnemy.Dir()));
	GDebugEngine::Instance()->gui_debug_line(ball.Pos(),ball.Pos()+Utils::Polar2Vector(100,ballVelDir));

	//state excution	
	if (NormalPlayUtils::faceTheirGoal(pVision,robotNum)){
		KickStatus::Instance()->setKick(robotNum,1200);
	}
	TaskT markTouchTask;
	markTouchTask.executor=robotNum;
	//避开球区域
	
	switch (state()){
	case Marking:
		markTouchTask.player.pos=_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse);
		markTouchTask.player.angle=finalKickDir;
		markTouchTask.player.flag=playerFlag |PlayerStatus::DODGE_REFEREE_AREA;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(markTouchTask));
		break;
	case MarkingAgain:
		markTouchTask.player.pos=_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse);
		markTouchTask.player.angle=finalKickDir;
		markTouchTask.player.flag=playerFlag |PlayerStatus::DODGE_REFEREE_AREA;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(markTouchTask));
		break;
	case GotoFirstPos:
		markTouchTask.player.pos=_markPos+Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse);
		markTouchTask.player.angle=finalKickDir;
		markTouchTask.player.flag=playerFlag|PlayerStatus::DODGE_REFEREE_AREA;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(markTouchTask));
		break;
	case Touch:
		markTouchTask.player.pos=markTouchPos+ Utils::Polar2Vector(Param::Vehicle::V2::TOUCH_SHIFT_DIST,kickDirReverse);
		markTouchTask.player.angle=finalKickDir;
		markTouchTask.player.flag=playerFlag|PlayerStatus::DODGE_REFEREE_AREA;
		markTouchTask.player.max_acceleration=1400;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(markTouchTask));
		break;
	}

	GDebugEngine::Instance()->gui_debug_msg(markTouchTask.player.pos,"F",COLOR_WHITE);
	//cout<<"flag is "<<markTouchTask.player.flag<<endl;
	_lastCycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}




CPlayerCommand* CMarkingFront::execute(const CVisionModule* pVision)
{
	if( subTask() ){
		return subTask()->execute(pVision);
	}
	return NULL;
}
