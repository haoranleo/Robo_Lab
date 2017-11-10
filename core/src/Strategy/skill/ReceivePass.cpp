#include "ReceivePass.h"
#include "GDebugEngine.h"
#include <VisionModule.h>
#include "skill/Factory.h"
#include <utils.h>
#include <DribbleStatus.h>
#include <RobotSensor.h>
#include <CommandFactory.h>

namespace{	
	enum receivePassState {
		GOTOPROJ =1,
		GETBALL,
	};
	int infrareCnt = 0;
	int prj2waitCnt = 0;
	int MAX_INFRARE_CNT = 2;
	CGeoPoint projMe = CGeoPoint(0,0);
}

CReceivePass::CReceivePass()
{
	_lastCycle = 0;
	_directCommand = NULL;
}

void CReceivePass::plan(const CVisionModule* pVision)
{
	_directCommand = NULL;

	const MobileVisionT& ball = pVision->Ball();
	const CGeoPoint ballPos = ball.Pos();
	const int vecNumber = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const CVector self2ball = ballPos - me.Pos();
	const CVector ball2self = me.Pos() - ballPos;
	const double finalDir = task().player.angle;
	CGeoLine ballMoveingLine(ball.RawPos(), ball.Vel().dir());
	CGeoPoint projMe = ballMoveingLine.projection(me.Pos());
	const double antiBallVelDir = Utils::Normalize(ball.Vel().dir() + Param::Math::PI);

	
	bool isInfraredOn = RobotSensor::Instance()->IsInfraredOn(vecNumber);
	int flags = task().player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;

	bool isBallMovingToMe;
	isBallMovingToMe = abs(Utils::Normalize((me.Pos() - ball.RawPos()).dir() - ball.Vel().dir())) < Param::Math::PI / 6;

	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1
		|| _lastRunner != vecNumber){
		setState(BEGINNING);
		infrareCnt = 0;
		prj2waitCnt = 0;
	}

	int new_state = state();
	int old_state;
	do{
		old_state = new_state;
		switch ( old_state )
		{
			case BEGINNING:
				{		
					 if (ball.Vel().mod() > 50 && isBallMovingToMe)
					 {
						  new_state = GOTOPROJ;
						 _mePos=me.Pos();
					 }
					 else{
						 new_state=GETBALL;
					 }
				}
				break;
			case GOTOPROJ:
				break;
			case GETBALL:
				if (ball.Vel().mod() > 50 && isBallMovingToMe)
				{
					new_state = GOTOPROJ;
					_mePos=me.Pos();
				}	
				break;
			default:
				new_state = GOTOPROJ;
				_mePos=me.Pos();
				break;
		}
	} while ( old_state != new_state );

	//cout << new_state<<endl;
	CGeoPoint newPos;
	setState(new_state);
	DribbleStatus::Instance()->setDribbleCommand(vecNumber,2);
	if (GOTOPROJ == state()){
		if (ball.Vel().mod() > 50 && isBallMovingToMe){
			CGeoPoint receivePos=ballMoveingLine.projection(_mePos);
			setSubTask(PlayerRole::makeItGoto(vecNumber, receivePos, antiBallVelDir, flags, 0));
		} 
		else{
			setSubTask(PlayerRole::makeItNoneTrajGetBall(vecNumber, finalDir, CVector(0,0), flags,0));
		}
	} else if (GETBALL == state()){
		setSubTask(PlayerRole::makeItInterKickV3(vecNumber, finalDir,flags));
	} else{
		setSubTask(PlayerRole::makeItNoneTrajGetBall(vecNumber, finalDir, CVector(0,0), flags));
	}


	_lastCycle = pVision->Cycle();
	_lastRunner = vecNumber;
	CStatedTask::plan(pVision);
}

CPlayerCommand* CReceivePass::execute(const CVisionModule* pVision)
{
	if( _directCommand ){
		return _directCommand;
	} else if (subTask())
	{
		return subTask()->execute(pVision);
	}
	return NULL;
}
