#include "Marking.h"
#include <skill/Factory.h>
#include "VisionModule.h"
#include "utils.h"
#include "GDebugEngine.h"
#include "geometry.h"
#include "WorldModel.h"
#include "defence/DefenceInfo.h"
#include "MarkingPosV2.h"
namespace{
	const bool VERBOSE_MODE = false;

	const bool FRIENDLY_MODE = false;//友好盯人

	const bool NOT_VOILENCE = false;//不采取暴力模式,优先级高于FRIENDLY_MODE
	//////////////////////////////////////////////////////////////////////////
	//NOT_VOILENCE = false:暴力模式
	//NOT_VOILENCE = true,FRIENDLY_MODE = false：普通模式	
	//NOT_VOILENCE = true,FRIENDLY_MODE = true：友好模式
	//////////////////////////////////////////////////////////////////////////
	CGeoPoint ourGoalPoint;       //我方球门

	const double simpleGotoDirLimt = Param::Math::PI*150/180;
	const double enemyPredictTime = 3;											//敌人的位置预测
	const double simpleGotoStrictDirLimt = Param::Math::PI * 70/180;			//更加严厉的角度限制
	const double PATH_PLAN_CHANGE_DIST = 75;									//修改该值以获得更好的速度规划变动点!!!
	const double FRIENDLY_DIST_LIMIT = 10;										//友好模式 距离限制

	bool kickOffSide[Param::Field::MAX_PLAYER+1] = {false,false,false,false,false,false,false};
}
CMarking::CMarking()
{
	ourGoalPoint = CGeoPoint(-Param::Field::PITCH_LENGTH/2,0);
	enemyNum = 1;
	_lastCycle = 0;
}

void CMarking::plan(const CVisionModule* pVision)
{
	static bool is_first = true;
	
	//内部状态进行重置
	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1 )
	{
		setState(BEGINNING);		
	}

	const MobileVisionT& ball = pVision->Ball();
	int vecNumber = task().executor;                                 //我方小车
	int enemyNum = task().ball.Sender;                             //敌方
	bool front = task().ball.front;
	CGeoPoint markPos;	

	if (is_first == true)
	{
		is_first = false;
		//cout<<"marking!!!"<<endl;
	}

	//在makeitmarking 和 makeitmarking2 中切换的特殊处理
	CGeoPoint taskPos = task().player.pos;
	if (taskPos.dist(CGeoPoint(1000,1000)) < 10)
	{
		markPos = MarkingPosV2::Instance()->getMarkingPosByNum(pVision,enemyNum);
	}else markPos = task().player.pos;

	//向worldModel中注册盯人信息，重要！！
	//WorldModel::Instance()->setMarkList(enemyNum,vecNumber,pVision->Cycle());
	DefenceInfo::Instance()->setMarkList(pVision,vecNumber,enemyNum);

	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& enemy = pVision->TheirPlayer(enemyNum);
	CGeoPoint enemyPos = enemy.Pos();
	CGeoPoint mePos = me.Pos();	
	CVector markPos2me = mePos - markPos;
	CVector markPos2ourGoal = ourGoalPoint - markPos;
	////加速
	if (0/*enemyNum == DefenceInfo::Instance()->getAttackOppNumByPri(0)*/)
	{
		double alpha = 0.0;
		double extr_dist = 60;
		if (markPos2me.mod() > 100 ) {
			alpha = markPos2me.mod() / 150;//original :  100
			if (alpha > 1.0) {
				alpha = 1.0;
			}
			extr_dist *= (markPos2me.mod() / 100);
			if (extr_dist > 200) {
				extr_dist = 200;
			}
			CGeoPoint fastPoint = markPos + Utils::Polar2Vector(extr_dist*alpha,(markPos - me.Pos()).dir());
			if (! Utils::OutOfField(fastPoint,0)) {
				markPos = fastPoint;
			}
		}
	}

	double me2ballDir = CVector(ball.Pos() - mePos).dir();
	double me2theirGoalDir = CVector(CGeoPoint(Param::Field::PITCH_LENGTH/2.0,0) - mePos).dir();
	TaskT MarkingTask(task());
	//避禁区
	MarkingTask.player.flag |= PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	//定位球避开开球50cm区域
	const string refMsg = WorldModel::Instance()->CurrentRefereeMsg();
	if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg || "theirKickOff" == refMsg || "gameStop" == refMsg)
	{
		MarkingTask.player.flag |= PlayerStatus::DODGE_REFEREE_AREA;
	}

	//路径规划设置
	if (me.Pos().dist(markPos) < PATH_PLAN_CHANGE_DIST)
	{
		//采用CMU的轨迹规划
		MarkingTask.player.is_specify_ctrl_method = true;
		MarkingTask.player.specified_ctrl_method = CMU_TRAJ;
	}

	//横向被甩开补充
	CVector opp2ourGoalVector = CVector(ourGoalPoint - enemyPos);
	CVector oppVel = enemy.Vel();
	double angle_oppVel_opp2Goal = fabs(Utils::Normalize(opp2ourGoalVector.dir() - oppVel.dir()));
	double sinPre = std::sin(angle_oppVel_opp2Goal);
	double cosPre = std::cos(angle_oppVel_opp2Goal);
	double dist = fabs(pVision->TheirPlayer(enemyNum).Y() - mePos.y());
	//cout<<sinPre<<" "<<oppVel.mod()<<" "<<dist<<endl;
	
	//避障设置
	CVector enemyPos2me = mePos - enemyPos;
	CVector enemyPos2ourGoal = ourGoalPoint - enemyPos;
	bool friendlyAngleOK = fabs(Utils::Normalize(enemyPos2ourGoal.dir() - enemyPos2me.dir())) < simpleGotoDirLimt;
	bool friendlyLimit = true;
	if (FRIENDLY_MODE && NOT_VOILENCE)
	{	
		friendlyLimit = fabs(Utils::Normalize(enemyPos2ourGoal.dir() - enemyPos2me.dir())) < simpleGotoStrictDirLimt;
	}
	if (sinPre>0.8 && oppVel.mod() >40){
		GDebugEngine::Instance()->gui_debug_msg(markPos,"M",COLOR_PURPLE);
		double ratio = 0;
		double absvel = fabs(enemy.Vel().y());
		if (absvel >120){
			ratio = 1;
		}else if (absvel >80 && absvel<=120){
			ratio = 0.025*absvel -2;
		}else{
			ratio = 0;
		}
	
		double shiftDist = ratio*oppVel.y();
		if (shiftDist <-100){
			shiftDist = -100;
		}
		if (shiftDist >100){
			shiftDist = 100;
		}
		//shiftDist = 0;
		markPos = markPos + Utils::Polar2Vector(shiftDist,Param::Math::PI/2);

	}
	if (Utils::OutOfField(markPos,0)){
		markPos = Utils::MakeInField(markPos,20);
	}
	//GDebugEngine::Instance()->gui_debug_msg(markPos,"M",COLOR_YELLOW);
	MarkingTask.player.pos = markPos;
	MarkingTask.player.angle = CVector(CGeoPoint(Param::Field::PITCH_LENGTH/2.0,0) - me.Pos()).dir();
	//MarkingTask.player.angle = 0;
	//GDebugEngine::Instance()->gui_debug_line(mePos,mePos+Utils::Polar2Vector(500,MarkingTask.player.angle),COLOR_WHITE);
	MarkingTask.player.max_acceleration = 650;
	MarkingTask.player.is_specify_ctrl_method = true;
	MarkingTask.player.specified_ctrl_method = CMU_TRAJ;
	//当是角球区域，盯人车对着球
	if (("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg || "gameStop" == refMsg) && pVision->Ball().X()<-180 )
	{
		//cout<<"markingCorner"<<endl;
		MarkingTask.player.angle = me2ballDir;
	}
	if (DefenceInfo::Instance()->getOppPlayerByNum(enemyNum)->isTheRole("RReceiver"))
	{
		MarkingTask.player.max_acceleration = 1000;
		MarkingTask.player.is_specify_ctrl_method = true;
		MarkingTask.player.specified_ctrl_method = CMU_TRAJ;
		if (true == NameSpaceMarkingPosV2::DENY_LOG[enemyNum] && fabs(Utils::Normalize(me2ballDir - me2theirGoalDir)) < Param::Math::PI * 60 / 180)
		{
			MarkingTask.player.angle = me2theirGoalDir;
		}
	}
	bool checkKickOffArea = false;
	if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg){
		if (pVision->Ball().Y() >0){
			kickOffSide[vecNumber] = false;
		}else{
			kickOffSide[vecNumber] = true;
		}
	}
	if (kickOffSide[vecNumber] == false){
		CGeoPoint leftUp = CGeoPoint(300,30);
		CGeoPoint rightDown = CGeoPoint(80,200);
		if (DefenceInfo::Instance()->checkInRecArea(enemyNum,pVision,MarkField(leftUp,rightDown))){
			checkKickOffArea = true;
		}
	}else if (kickOffSide[vecNumber] == true){
		CGeoPoint leftUp = CGeoPoint(300,-200);
		CGeoPoint rightDown = CGeoPoint(80,-30);
		if (DefenceInfo::Instance()->checkInRecArea(enemyNum,pVision,MarkField(leftUp,rightDown))){
			checkKickOffArea = true;
		}
	}
	if(true == front && enemyPos.x()>-180 && false == checkKickOffArea)
	{
		MarkingTask.player.flag |= PlayerStatus::NOT_AVOID_THEIR_VEHICLE;
		//cout<<MarkingTask.player.flag<<endl;
		DefenceInfo::Instance()->setMarkMode(vecNumber,enemyNum,true);
		setSubTask(PlayerRole::makeItMarkingFront(vecNumber,enemyNum,me2theirGoalDir,MarkingTask.player.flag));
		//cout<<enemyNum<<" front Mode"<<endl;
	}else if ((friendlyAngleOK && friendlyLimit) || !NOT_VOILENCE)
	{	
		//不避开对手车
		MarkingTask.player.flag |= PlayerStatus::NOT_AVOID_THEIR_VEHICLE;
		DefenceInfo::Instance()->setMarkMode(vecNumber,enemyNum,false);
		//cout<<MarkingTask.player.flag<<endl;
		//cout<<"2222222"<<endl;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(MarkingTask));	
	}else {
		//cout<<MarkingTask.player.flag<<endl;
		DefenceInfo::Instance()->setMarkMode(vecNumber,enemyNum,false);
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(MarkingTask));
	}
	GDebugEngine::Instance()->gui_debug_msg(MarkingTask.player.pos,"M",COLOR_WHITE);
	//cout<<enemyNum<<": "<<pVision->TheirPlayer(enemyNum).Vel().mod()<<" "<<vecNumber<<endl;
	_lastCycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}


CPlayerCommand* CMarking::execute(const CVisionModule* pVision)
{
	if( subTask() ){
		return subTask()->execute(pVision);
	}
	return NULL;
}

