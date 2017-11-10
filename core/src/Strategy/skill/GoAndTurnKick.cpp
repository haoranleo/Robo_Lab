#include "GoAndTurnKick.h"
#include "GDebugEngine.h"
#include <VisionModule.h>
#include "skill/Factory.h"
#include "ControlModel.h"
#include "KickStatus.h"

#include <utils.h>
#include <ControlModel.h>

namespace
{
	//配置状态机
	enum goAndTurnKickState
	{
		GOTO =1,
		GOAROUND,
		GETKICK,
	};
	double useYSpeed;
	double useRotVel;
	bool IS_SIMULATION = false;
	int roundCnt = 0;
	int MaxRoundCnt = 1;
}

CGoAndTurnKick::CGoAndTurnKick()
{
	DECLARE_PARAM_READER_BEGIN(General)
		READ_PARAM(IS_SIMULATION)
	DECLARE_PARAM_READER_END
	_lastCycle = 0;
}

void CGoAndTurnKick::plan(const CVisionModule* pVision)
{
	// 判断是否需要进行状态重置
	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1){
		setState(BEGINNING);
		_lastRotVel = 0;
	}
	_lastCycle = pVision->Cycle();
		
	// 小车相关图像信息获取
	const MobileVisionT& ball = pVision->Ball();
	const CGeoPoint ballPos = ball.Pos();
	const int vecNumber = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const CVector self2ball = ballPos - me.Pos();
	const CVector ball2self = me.Pos() - ballPos;
	const double finalDir = task().player.angle;
	

	//任务设置
	TaskT myTask(task());
	//设置避开我方禁区标签
	myTask.player.flag = myTask.player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	
	int new_state = state();
	int old_state;
	do{
		old_state = new_state;
		switch (old_state) {
			case BEGINNING:			// 直接跳到 GOTO
				new_state = GOTO;
				break;

			case GOTO:				// 球在车嘴巴前方并且转速比较小，跳到 GOAROUND
				if ( me.Pos().dist(ballPos) < (Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + Param::Field::BALL_SIZE + 10)
					&& fabs(Utils::Normalize(self2ball.dir() - me.Dir())) < 8*Param::Math::PI/180) {
					new_state = GOAROUND;
				}
				break;

			case GOAROUND:			// 朝向精度已经满足，跳到GETKICK
				if (fabs(Utils::Normalize(me.Dir() - finalDir)) < 15*Param::Math::PI/180){
					if(roundCnt > MaxRoundCnt){
						new_state = GETKICK;
						roundCnt = 0;
					} else{
						roundCnt ++;
					}
				}
				break;

			case GETKICK:
				if ( me.Pos().dist(ballPos) > 40) {
					new_state = GOTO;
				}
				break;

			default:
				new_state = GOTO;
				break;
		}
		//printf("loop: %s --> %s\n", stateToString(old_state), stateToString(new_state));
	} while (old_state != new_state);

	setState(new_state);
	if (GOTO == state())
	{
	//	cout<<pVision->Cycle()<<" GOTO  "<<ball.Pos()<<endl;
		double reverse_final = Utils::Normalize(finalDir + Param::Math::PI);
		myTask.player.pos = ballPos + Utils::Polar2Vector(14,reverse_final/*ball2self.dir()*/);
		myTask.player.angle = /*self2ball.dir()*/finalDir;
		if( ball.Pos().dist(me.Pos()) < 30) {
			setSubTask(TaskFactoryV2::Instance()->GotoPosition(myTask));
		} else {
			setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(myTask));
		}
		//myTask.player.rotvel = 6;
		//setSubTask(TaskFactoryV2::Instance()->NoneTrajGetBall(myTask));
	} else if (GOAROUND == state()){
	//	cout<<pVision->Cycle()<<" GOAROUND  "<<ball.Pos()<<endl;
		myTask.player.rotvel = 5;//避球参数
		double angeDiff = Utils::Normalize(finalDir - me.Dir());
		double factor = angeDiff > 0 ? 1 : (-1);
		if(IS_SIMULATION){
			//factor = factor * (fabs(angeDiff) / Param::Math::PI + 1) / 1.2;
			//useYSpeed = 50;
			//useRotVel = 4;

			//CVector localVel(0, -useYSpeed * factor); // 全局坐标系中的速度矢量
			//CVector globalVel = localVel.rotate(me.Dir());
			//myTask.player.speed_x = globalVel.x();
			//myTask.player.speed_y = globalVel.y();
			//myTask.player.rotate_speed = useRotVel * factor;
			//setSubTask(TaskFactoryV2::Instance()->Speed(myTask));

			/*double r = ball.Pos().dist(me.Pos());
			if (r <= Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + Param::Field::BALL_SIZE) {
				r = Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + Param::Field::BALL_SIZE + 1;
			}
			double curRotVel = zeroFinalAngularSpeed(angeDiff,me.RotVel(),10,20);
			double curVy = -curRotVel * (r+2);
			CVector localVel(0, curVy);
			CVector globalVel = localVel.rotate(me.Dir());
			myTask.player.speed_x = globalVel.x();
			myTask.player.speed_y = globalVel.y();
			myTask.player.rotate_speed = curRotVel;
			setSubTask(TaskFactoryV2::Instance()->Speed(myTask));*/
			setSubTask(TaskFactoryV2::Instance()->NoneTrajGetBall(myTask));
		} else{
			// 旧车版本 zhyaic 2012.5.26
			double DegreeAngle = angeDiff * 180 / Param::Math::PI;
			useRotVel = zeroFinalAngularSpeed(angeDiff, me.RotVel(), 4, 20);
			std::cout<<me.RotVel()<<" "<<useRotVel<<endl;
			if(fabs(useRotVel) >= 4){
				useYSpeed = 72*factor;
				useRotVel = fabs(useRotVel)*factor;
			} else if( fabs(useRotVel) >= 3){
				useYSpeed =  (fabs(useRotVel) * 26 - 32)*factor;
				useRotVel = fabs(useRotVel)*factor;
			} else if( fabs(useRotVel) >= 1){
				useYSpeed = (fabs(useRotVel) * 16 - 2)*factor;
				useRotVel = fabs(useRotVel)*factor;
			} else{
				useRotVel = 1*factor;
				useYSpeed = 14*factor;
			}
			useRotVel = 4;
			useYSpeed = 72;

			double useXSpeed = 0;
			if(me.Pos().dist(ball.Pos()) < 14){
				useXSpeed = 15;
			}
			CVector localVel(-useXSpeed, - useYSpeed); // 全局坐标系中的速度矢量
			CVector globalVel = localVel.rotate(me.Dir());
			myTask.player.speed_x = globalVel.x();
			myTask.player.speed_y = globalVel.y();
			myTask.player.rotate_speed = useRotVel;
			setSubTask(TaskFactoryV2::Instance()->Speed(myTask));

			// 旧车版本 cliffyin
			/*double r = ball.Pos().dist(me.Pos());
			if (r <= Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + Param::Field::BALL_SIZE+2) {
				r = Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + Param::Field::BALL_SIZE + 2;
			}
			double curRotVel = zeroFinalAngularSpeed(angeDiff,me.RotVel(),15,25);
			double curVy = -curRotVel * (r+2.0);
			CVector localVel(0, curVy);
			CVector globalVel = localVel.rotate(me.Dir());
			myTask.player.speed_x = globalVel.x();
			myTask.player.speed_y = globalVel.y();
			myTask.player.rotate_speed = curRotVel;
			setSubTask(TaskFactoryV2::Instance()->Speed(myTask));*/

			// 直接用拿球的版本
		//	setSubTask(TaskFactoryV2::Instance()->NoneTrajGetBall(myTask));
		//	setSubTask(TaskFactoryV2::Instance()->GoAroundBall(myTask));
		}
	} else if (GETKICK == state()){
		myTask.player.rotvel = -2;
		KickStatus::Instance()->setKick(vecNumber,400);
		setSubTask(TaskFactoryV2::Instance()->NoneTrajGetBall(myTask));
	}
	
	return CStatedTask::plan(pVision);
}

CPlayerCommand* CGoAndTurnKick::execute(const CVisionModule* pVision)
{
	if(subTask()) {
		return subTask()->execute(pVision);
	}

	return NULL;
}