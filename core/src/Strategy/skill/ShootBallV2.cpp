#include "GDebugEngine.h"
#include "skill/Factory.h"
#include <utils.h>
#include "param.h"
#include <cmath>
#include <utils.h>
#include <VisionModule.h>
#include "BallStatus.h"
#include "ShootBallV2.h"
#include "TouchKickPos.h"
#include "KickDirection.h"
#include "WorldModel.h"
#include <KickStatus.h>

namespace
{
	enum ShootBall_State{
		TOUCH_KICK = 1,
		CHASE_KICK,
	};
	bool verBos = false;
	int Touch2ChaseCntSpeed = 0;
	int Touch2ChaseCntAngle = 0;
	int Chase2TouchCnt = 0;
};

CShootBallV2::CShootBallV2()
{
	_lastCycle = 0;
}

// 传进来的朝向是一个比较稳定的朝向，适用于TouchKick，但是对
// 于Chase和Intercept来说需要时时计算朝向，Touch是静态动作，而
// Chase和Intercept是动态动作！！！！
void CShootBallV2::plan(const CVisionModule* pVision)
{
	const int runner = task().executor;
	int flags = task().player.flag;
	double finalDir = task().player.angle;
	const PlayerVisionT& self = pVision->OurPlayer(runner);
	const MobileVisionT& ball = pVision->Ball();
	CVector self2ball = ball.Pos() - self.Pos();
	const double ballVelDir = ball.Vel().dir();
	double antiBallVelDir = Utils::Normalize(ballVelDir + Param::Math::PI);
	double ballSpeed = ball.Vel().mod();
	CGeoSegment shootSeg(self.Pos(), self.Pos() + Utils::Polar2Vector(1500, finalDir));
	CGeoPoint ballShootSegInterP = shootSeg.projection(ball.Pos());
	CGeoLine ballMovingLine = CGeoLine(ball.Pos(),ball.Pos()+Utils::Polar2Vector(1000,ball.Vel().dir()));
	CGeoPoint projMePos = ballMovingLine.projection(self.Pos());
	double me2ballMovingLineDist = projMePos.dist(self.Pos());

	bool  isBallOnShootSeg = shootSeg.IsPointOnLineOnSegment(ballShootSegInterP);
	double a_m2t_2_m2b = fabs(Utils::Normalize(self2ball.dir() - finalDir)) * 180 / Param::Math::PI;
	double a_m2t_2_bsr = fabs(Utils::Normalize(antiBallVelDir - finalDir)) * 180 / Param::Math::PI;
	double a_m2t_2_bsl = fabs(Utils::Normalize(ballVelDir - finalDir)) * 180 / Param::Math::PI; 
	double a_m2b_2_bsr = fabs(Utils::Normalize(self2ball.dir() - antiBallVelDir)) * 180 / Param::Math::PI;

	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1
		|| BallStatus::Instance()->IsBallKickedOut(runner)
		|| _lastRunner != runner){
		setState(BEGINNING);
		int Touch2ChaseCntSpeed = 0;
		int Touch2ChaseCntAngle = 0;
		int Chase2TouchCnt = 0;
	}

	int new_state = state(), old_state = state();
	do {
		old_state = new_state;
		switch (old_state){
			case BEGINNING:
				{
					if( ballSpeed > 50 && a_m2t_2_m2b<60 && a_m2t_2_bsr<90  && a_m2b_2_bsr<60){
						if(verBos) cout<<"BEGINNING-->Touch"<<endl;
						new_state = TOUCH_KICK;
					} else{
						if(verBos) cout<<"BEGINNING-->Chase"<<endl;
						new_state = CHASE_KICK;
					} 
				}
				break;
			case TOUCH_KICK:
				//cout<<ballSpeed<<" "<<a_m2t_2_bsr<<" "<<a_m2t_2_m2b<<endl;
				if( ballSpeed<100 && a_m2t_2_bsr > 100){
					if( Touch2ChaseCntSpeed > 10){
						if(verBos) cout<<"Touch-->Chase"<<endl;
						new_state = CHASE_KICK;
						Touch2ChaseCntSpeed = 0;
					} else{
						Touch2ChaseCntSpeed++;
					}
				}
				break;
			case CHASE_KICK:
				if( a_m2t_2_bsr > 90 && a_m2t_2_m2b > 60 && a_m2b_2_bsr > 60 && ballSpeed > 120 ){
					if( Chase2TouchCnt > 1){
						if(verBos) cout<<"Chase-->Touch"<<endl;
						new_state = TOUCH_KICK;
						Chase2TouchCnt = 0;
					} else{
						Chase2TouchCnt++;
					}
				} 
				break;
			default:
				new_state = BEGINNING;
				break;
		}
	} while (false);
	setState(new_state);

	flags = flags | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	KickDirection::Instance()->GenerateShootDir(runner,pVision->OurPlayer(runner).Pos());
	switch (state()){
		case TOUCH_KICK:
			{
				setSubTask(PlayerRole::makeItTouchKick(runner, finalDir, false, flags));
			}
			break;
		case CHASE_KICK:
			{
				//finalDir = 	KickDirection::Instance()->getRawKickDir();
				setSubTask(PlayerRole::makeItChaseKickV1(runner, finalDir, flags));
			}
			break;
		default:
			setSubTask(PlayerRole::makeItChaseKickV1(runner, finalDir, flags));
			break;
	}

	_lastCycle = pVision->Cycle();
	_lastRunner = runner;
	CPlayerTask::plan(pVision);
}
CPlayerCommand* CShootBallV2::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}

	return NULL;
}
