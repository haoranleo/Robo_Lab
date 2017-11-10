#include "WaitTouch.h"
#include "skill/Factory.h"
#include "GDebugEngine.h"
#include "BallSpeedModel.h"
#include "BallStatus.h"
#include "WaitKickPos.h"
#include "DribbleStatus.h"

namespace{
	int MAX_CNT_FOR_SHOOT = 2;
}

CWaitTouch::CWaitTouch() 
{
	last_cycle = 0;
	cur_cnt = 0;
	need_shoot = false;
}

void CWaitTouch::plan(const CVisionModule* pVision) 
{
	///> 1.任务参数及图像信息
	if (pVision->Cycle() - last_cycle  > 6) {
		need_shoot = false;
	}

	int runner = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(runner);
	const MobileVisionT& ball = pVision->Ball();
	double kick_dir;

	TaskT wait_kick_task(task());
	wait_kick_task.player.flag = wait_kick_task.player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	kick_dir	= wait_kick_task.player.angle;

	if (!need_shoot) {
		CVector car_2_ball = ball.Pos() - me.Pos();
		CVector car_2_goal = CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - me.Pos();
		if (pVision->Ball().Vel().mod() < 50 && car_2_ball.mod() < 40/* && fabs(Utils::Normalize(car_2_ball.dir() - car_2_goal.dir())) < Param::Math::PI / 12*/) {
			cur_cnt++;
			if (cur_cnt > MAX_CNT_FOR_SHOOT) {
				cur_cnt = 0;
				need_shoot = true;
			}
		}
		WaitKickPos::Instance()->GenerateWaitKickPos(wait_kick_task.ball.pos,wait_kick_task.ball.angle,runner,kick_dir);
		wait_kick_task.player.pos = WaitKickPos::Instance()->getKickPos(runner);
		setSubTask(TaskFactoryV2::Instance()->GotoPosition(wait_kick_task));
	} else{
		setSubTask(PlayerRole::makeItShootBall(runner,kick_dir,false,Param::Math::PI/18,1300,0,wait_kick_task.player.flag));
	}
	last_cycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

CPlayerCommand* CWaitTouch::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}
	return NULL;
}