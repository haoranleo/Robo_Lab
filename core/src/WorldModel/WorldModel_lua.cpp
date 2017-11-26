#include "WorldModel.h"
#include "PlayInterface.h"
#include "KickStatus.h"
#include "BestPlayer.h"
#include <ShootRangeList.h>
#include "bayes/MatchState.h"
#include "RobotSensor.h"
#include "BallStatus.h"
#include "BufferCounter.h"
#include "BallSpeedModel.h"
#include "KickDirection.h"
#include "NormalPlayUtils.h"
#include "TaskMediator.h"
#include "FreeKickUtils.h"


namespace{
	int passCouter=0;
	int shootCouter=0;
	int kickAtEnemyCouter=0;
}

int CWorldModel::OurRobotNum() 
{
	int validNum = 0;
	for (int i = 1; i <= Param::Field::MAX_PLAYER; i++) {		
		if (_pVision->OurPlayer(i).Valid()) {	
			validNum++;
		}
	}
	validNum = validNum > Param::Field::MAX_PLAYER ? Param::Field::MAX_PLAYER : validNum;
	return validNum;
}

int CWorldModel::GetRealNum(int num)
{
	return PlayInterface::Instance()->getRealIndexByNum(num);
}

int CWorldModel::GetTacticNum(int num)
{
	return PlayInterface::Instance()->getNumbByRealIndex(num);
}

bool CWorldModel::IsBallKicked(int num)
{
	KickStatus::Instance()->updateForceClose(this->vision()->Cycle());
	if(KickStatus::Instance()->isForceClosed()){
		return false;
	}
	bool isballkicktemp = BallStatus::Instance()->IsBallKickedOut(num);
	return isballkicktemp;
}

bool CWorldModel::CanDefenceExit()
{
	for (int i = 1;i <= Param::Field::MAX_PLAYER;i++)
	{
		if(RobotSensor::Instance()->IsInfraredOn(i)||BallStatus::Instance()->IsBallKickedOut()){
			return true;
		}
	}
	return false;
}

bool CWorldModel::NeedExitAttackDef(CGeoPoint leftUp, CGeoPoint rightDown, int mode)
{
	bool result = false;
	if (mode == 1){
		//纵向截球，如果球不向这个区域内或者球的y值已经越过markingTouch的车，则需要退出
		int markTouchNum = DefenceInfo::Instance()->getMarkingTouchNum(leftUp,rightDown);
		if (_pVision->OurPlayer(markTouchNum).Y()<0 && markTouchNum!=0){
			if (_pVision->Ball().Y()<_pVision->OurPlayer(markTouchNum).Y()-5){
				result = true;
			}
		}else if (_pVision->OurPlayer(markTouchNum).Y()>0 && markTouchNum!=0){
			if (_pVision->Ball().Y()>_pVision->OurPlayer(markTouchNum).Y()+5){
				result = true;
			}
		}else if (markTouchNum == 0){
		 result = true;
		 }
		//判断球是否朝这个纵向的方向去,!!!!!!!待添加
		if (leftUp.y()< 0){
			CGeoPoint upGudge = CGeoPoint(leftUp.x()+10,rightDown.y());
			CGeoPoint downGudge = CGeoPoint(rightDown.x()-20,rightDown.y());
			//cout<<upGudge<<" "<<downGudge<<endl;
			GDebugEngine::Instance()->gui_debug_x(upGudge,COLOR_BLACK);
			GDebugEngine::Instance()->gui_debug_x(downGudge,COLOR_BLACK);
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),upGudge,COLOR_PURPLE);
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),downGudge,COLOR_PURPLE);
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),_pVision->Ball().Pos()+Utils::Polar2Vector(300,_pVision->Ball().Vel().dir()),COLOR_PURPLE);
			if (!Utils::AngleBetween(_pVision->Ball().Vel().dir(),(downGudge - _pVision->Ball().Pos()).dir(),(upGudge - _pVision->Ball().Pos()).dir(),0) && _pVision->Ball().Vel().mod()>50){
				result = true;
				//cout<<"111111111"<<endl;
			}
		}else if (leftUp.y()>0){
			CGeoPoint upGudge = CGeoPoint(leftUp.x()+10,leftUp.y());
			CGeoPoint downGudge = CGeoPoint(rightDown.x()-20,leftUp.y());
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),upGudge,COLOR_PURPLE);
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),downGudge,COLOR_PURPLE);
			GDebugEngine::Instance()->gui_debug_line(_pVision->Ball().Pos(),_pVision->Ball().Pos()+Utils::Polar2Vector(300,_pVision->Ball().Vel().dir()),COLOR_PURPLE);
			if (!Utils::AngleBetween(_pVision->Ball().Vel().dir(),(upGudge - _pVision->Ball().Pos()).dir(),(downGudge - _pVision->Ball().Pos()).dir(),0) && _pVision->Ball().Vel().mod()>50){
				result = true;
				//cout<<"222222222"<<endl;
			}
		}
	}else if (mode == 2){
		int markTouchNum = DefenceInfo::Instance()->getMarkingTouchNum(leftUp,rightDown);
		if (markTouchNum!= 0 && _pVision->OurPlayer(markTouchNum).X()>_pVision->Ball().X()){
			result = true;
		}else if (markTouchNum == 0){
			result = true;
		}
		//判断球是否朝这个横向的方向去,!!!!!!!待修改
	}
	return result;
}

bool CWorldModel::ball2MarkingTouch(CGeoPoint leftUp, CGeoPoint rightDown)
{
	double dist = 0;
	bool result =false;
	int markTouchNum =  DefenceInfo::Instance()->getMarkingTouchNum(leftUp,rightDown);
	if (markTouchNum!=0){
		dist = vision()->Ball().RawPos().dist(vision()->OurPlayer(markTouchNum).Pos());
		if (dist <100){
			result = true;
		}
	}
	return result;
}
//bool CWorldModel::CanDefenceExit()
//{
//	for (int i = 1;i <= Param::Field::MAX_PLAYER;i++)
//	{
//		if(RobotSensor::Instance()->IsInfraredOn(i)||BallStatus::Instance()->IsBallKickedOut()){
//			return true;
//		}
//	}
//	const MobileVisionT ball=_pVision->Ball();
//	if (ball.Valid()&&ball.Vel().mod()<80&&_pVision->gameState().gameOn()){
//		for (int i = 1;i <= Param::Field::MAX_PLAYER;i++){
//			if (_pVision->OurPlayer(i).Valid()){
//				const PlayerVisionT me=_pVision->OurPlayer(i);
//				CVector self2ball=ball.Pos()-me.Pos();
//				double dist2ball=self2ball.mod();
//				if (dist2ball<40){		
//					double allowInfrontAngleBuffer = (dist2ball/(Param::Vehicle::V2::PLAYER_SIZE))*Param::Vehicle::V2::KICK_ANGLE < Param::Math::PI/5.0?
//						(dist2ball/(Param::Vehicle::V2::PLAYER_SIZE))*Param::Vehicle::V2::KICK_ANGLE:Param::Math::PI/5.0;
//					bool isBallInFront = fabs(Utils::Normalize(self2ball.dir()-me.Dir())) < allowInfrontAngleBuffer
//						&& dist2ball < (2.5*Param::Vehicle::V2::PLAYER_SIZE + Param::Field::BALL_SIZE);
//					if (isBallInFront){
//						return true;
//					}
//				}
//			}
//		}
//	}
//	return false;
//}

bool CWorldModel::isMarkingFrontValid(CGeoPoint checkPos, double checkDir)
{
	vector<int> markFrontList;
	markFrontList.clear();
	for (int i=1;i<=Param::Field::MAX_PLAYER;i++){
		if (DefenceInfo::Instance()->queryMarked(i) && DefenceInfo::Instance()->getMarkMode(i)){
			markFrontList.push_back(i);
		}
	}
	//static int lasyCycle = 0;
	//static double checkDir = 0;
	//static CGeoPoint checkPos = CGeoPoint(0,0);
	//static bool firstIn = false;
	//static int bestEnemy = 0;
	//if (vision()->Cycle() - lasyCycle>6){
	//	firstIn = false;
	//	bestEnemy = BestPlayer::Instance()->getTheirBestPlayer();
	//}
	//lasyCycle = vision()->Cycle();
	////球开出时的判断
	//if (vision()->Ball().Vel().mod()>50 && vision()->Ball().Pos().dist(vision()->TheirPlayer(bestEnemy).Pos())>15 && firstIn == false){
	//	checkDir = vision()->TheirPlayer(bestEnemy).Dir();
	//	checkPos = vision()->TheirPlayer(bestEnemy).Pos();
	//	firstIn = true;
	//}
	//bool result = false;
	//if (firstIn == true){
	//	for(vector<int>::iterator ir =markFrontList.begin();ir!=markFrontList.end();ir++){
	//		if (fabs(Utils::Normalize(checkDir - (vision()->TheirPlayer(*ir).Pos() - checkPos).dir()))<Param::Math::PI/12){
	//			result =true;
	//			GDebugEngine::Instance()->gui_debug_line(checkPos,checkPos+Utils::Polar2Vector(300,checkDir),COLOR_WHITE);
	//			GDebugEngine::Instance()->gui_debug_line(checkPos,vision()->TheirPlayer(*ir).Pos(),COLOR_WHITE);
	//			break;
	//		}
	//	}
	//}

	bool result = false;
	int toMeNum = 0;
	for(vector<int>::iterator ir =markFrontList.begin();ir!=markFrontList.end();ir++){
		if (fabs(Utils::Normalize(checkDir - (vision()->TheirPlayer(*ir).Pos() - checkPos).dir()))<Param::Math::PI/12){
			result =true;
			toMeNum = DefenceInfo::Instance()->getOurMarkDenfender(*ir);
			GDebugEngine::Instance()->gui_debug_line(checkPos,checkPos+Utils::Polar2Vector(300,checkDir),COLOR_WHITE);
			GDebugEngine::Instance()->gui_debug_line(checkPos,vision()->TheirPlayer(*ir).Pos(),COLOR_WHITE);
			break;
		}
	}
	//如果在球的路线上有敌方车站在前面，则跳出markingFront
	for(vector<int>::iterator ir =markFrontList.begin();ir!=markFrontList.end();ir++){
		int meNum = DefenceInfo::Instance()->getOurMarkDenfender(*ir);
		for (int i=1;i<Param::Field::MAX_PLAYER+1;i++){
			double ball2EnemyDir = (vision()->Ball().Pos() - vision()->TheirPlayer(i).Pos()).dir();
			double ball2MeDir = (vision()->Ball().Pos() -vision()->OurPlayer(meNum).Pos()).dir();
			//GDebugEngine::Instance()->gui_debug_line(vision()->Ball().Pos(),vision()->TheirPlayer(2).Pos(),COLOR_PURPLE);
			//GDebugEngine::Instance()->gui_debug_line(vision()->Ball().Pos(),vision()->OurPlayer(meNum).Pos(),COLOR_PURPLE);
		/*	if (meNum == 5 && i ==2){
				cout<<meNum<<" "<<i<<" : "<<fabs(Utils::Normalize(ball2EnemyDir - ball2MeDir))*180/Param::Math::PI<<" "
					<< vision()->OurPlayer(meNum).X()<<" "<<vision()->TheirPlayer(i).X()<<" "<<fabs(Utils::Normalize(ball2MeDir - Utils::Normalize(checkDir+Param::Math::PI)))*180/Param::Math::PI<<endl;
				cout<<(fabs(Utils::Normalize(ball2EnemyDir - ball2MeDir))<Param::Math::PI/12 && vision()->OurPlayer(meNum).X()< vision()->TheirPlayer(i).X()
					&& fabs(Utils::Normalize(ball2MeDir - Utils::Normalize(checkDir+Param::Math::PI)))<Param::Math::PI/6)<<endl;
			}*/
			if (fabs(Utils::Normalize(ball2EnemyDir - ball2MeDir))<Param::Math::PI/12 && vision()->OurPlayer(meNum).X()< vision()->TheirPlayer(i).X()
				&& fabs(Utils::Normalize(ball2MeDir - Utils::Normalize(checkDir+Param::Math::PI)))<Param::Math::PI/6){
				result = false;
				//cout<<"in change"<<endl;
				return false;
			}
		}
	}
	//如果球过了markingFront中截球的一辆车则跳出markingFront
	if (vision()->Ball().X() < vision()->OurPlayer(toMeNum).X()){
		return false;
	}
	return result;
}

//每帧只调用一次
int CWorldModel::getEnemyKickOffNum()
{
	const string refMsg = vision()->GetCurrentRefereeMsg();
	int kickOffNum = 0;
	vector<int> kickOffList;
	static int lastKickOffNum = 1;
	static int keepCnt = 0;
	static int lastCycle = 0;
	static int retVal = 0;
	if (vision()->Cycle() - lastCycle >6){
		lastKickOffNum = 1;
		keepCnt = 0;
	}
	if (lastCycle == vision()->Cycle()){
		return retVal;
	}
	kickOffList.clear();
	//暂且用区域内的车的数量判断敌方开球车数量
	if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg){
		CGeoCirlce kickOffArea = CGeoCirlce(vision()->Ball().Pos(),75);
		for (int i =1;i<=Param::Field::MAX_PLAYER;i++){
			if (kickOffArea.HasPoint(vision()->TheirPlayer(i).Pos())){
				kickOffNum++;
				//cout<<i<<" might be kicking"<<endl;
				kickOffList.push_back(i);
			}
		}
	}
	if (kickOffNum == lastKickOffNum){
		keepCnt++;
	}else if (kickOffNum!=lastKickOffNum && kickOffNum!=0){
		lastKickOffNum = kickOffNum;
		keepCnt = 0;
	}else{
		keepCnt = 0;
	}
	//当有大于两辆车开球时，剔除不合理的盯人车
	if (keepCnt >=3 && kickOffNum>=2){
		double minToBallDist = 1000;
		int minNum = 0;
		for (vector<int>::iterator ir = kickOffList.begin();ir!=kickOffList.end();ir++){
			double check =vision()->TheirPlayer(*ir).Pos().dist(vision()->Ball().Pos());
			//cout<<*ir<<" "<<check<<endl;
			if (check<minToBallDist){
				minNum = *ir;
				minToBallDist = check;
			}
		}
		//cout<<"minNum is "<<minNum<<endl;
		DefenceInfo::Instance()->resetMarkingInfo();
		for (vector<int>::iterator ir = kickOffList.begin();ir!=kickOffList.end();ir++){
			if (*ir!=minNum){
				DefenceInfo::Instance()->setNoMarkingNum(*ir);
			}
		}
	}else{
		DefenceInfo::Instance()->resetMarkingInfo();
	}

	lastCycle = vision()->Cycle();

	if (keepCnt>=3 && kickOffNum>=2){
		retVal = kickOffNum;
		return kickOffNum;
	}else{
		retVal = 1;
		return 1;
	}
}

bool CWorldModel::checkEnemyKickOffNumChanged()
{
	static int lastKickOffNum = 1;
	static int lastCycle = 0;
	static bool result = false;
	if (vision()->Cycle() == lastCycle){
		return result;
	}
	int kickOffNum = getEnemyKickOffNum();
	if (lastKickOffNum != kickOffNum){
		lastKickOffNum = kickOffNum;
		result =true;
	}
	lastCycle = vision()->Cycle();
	return result;
}

int CWorldModel::getMarkingTouchArea(CGeoPoint leftPos1,CGeoPoint leftPos2,CGeoPoint rightPos1,CGeoPoint rightPos2)
{
	int leftNum = DefenceInfo::Instance()->getMarkingTouchNum(leftPos1,leftPos2);
	int rightNum = DefenceInfo::Instance()->getMarkingTouchNum(rightPos1,rightPos2);
	if (vision()->OurPlayer(leftNum).X()>vision()->OurPlayer(rightNum).X()){
		//cout<<"choose 1"<<endl;
		return 1;
	}else{
		//cout<<"choose 2"<<endl;
		return 2;
	}
}

bool CWorldModel::IsBestPlayerChanged()
{
	static int _last_bestNum = BestPlayer::Instance()->getOurBestPlayer();
	if(_last_bestNum != BestPlayer::Instance()->getOurBestPlayer()){
		_last_bestNum = BestPlayer::Instance()->getOurBestPlayer();
		return true;
	} else{
		return false;
	}
}


// 【#】 由球这点判断是否可以射门


string	CWorldModel::CurrentBayes()
{
	return MatchState::Instance()->getMatchStateInString();
}

//play状态机跳转清理
void CWorldModel::SPlayFSMSwitchClearAll(bool clear_flag)
{
	if (! clear_flag) {
		return ;
	}

	// 暂时只有清理 球被提出的状态
	KickStatus::Instance()->resetKick2ForceClose(true,this->vision()->Cycle());
	BallStatus::Instance()->clearKickCmd();
	BufferCounter::Instance()->clear();
	// To be added if needed

	return ;
}


const bool CWorldModel::canProtectBall(int current_cycle, int myNum){
	static int last_cycle = -1;
	static bool _canProtectBall;

	if (last_cycle < current_cycle) {
		const MobileVisionT ball=_pVision->Ball();
		const PlayerVisionT he=_pVision->TheirPlayer(BestPlayer::Instance()->getTheirBestPlayer());
		const PlayerVisionT me=_pVision->OurPlayer(BestPlayer::Instance()->getOurBestPlayer());
		CGeoPoint predictBallPos=BallSpeedModel::Instance()->posForTime(30,_pVision);
		const CVector self2rawball=ball.Pos()-me.Pos();
		if (IsOurBallByAutoReferee()&&ball.Vel().mod()>150&&Utils::OutOfField(predictBallPos,-10)&&fabs(ball.Vel().dir())>Param::Math::PI/1.5
				&&he.Pos().dist(ball.Pos())>me.Pos().dist(ball.Pos())+30&&me.Pos().dist(ball.Pos())>30
				&&(fabs(Utils::Normalize(ball.Vel().dir() - Utils::Normalize(self2rawball.dir()+Param::Math::PI))) > Param::Math::PI/2.5)){		
					_canProtectBall=true;
		}else{
			_canProtectBall=false;
		}
		last_cycle=current_cycle;
	}
	return _canProtectBall;
}



const bool CWorldModel::canShootOnBallPos(int current_cycle, int myNum) {
	static int last_cycle = -1;
	static bool _canshootonballpos;


	if (last_cycle < current_cycle) {
		if (! Utils::PlayerNumValid(myNum)) {
			myNum = BestPlayer::Instance()->getOurBestPlayer();
			if (! Utils::PlayerNumValid(myNum)) {
				myNum = 1;
			}
		}
		_canshootonballpos = false;
		CShootRangeList shootRangeList(_pVision, myNum, _pVision->Ball().Pos());
		const CValueRange* bestRange = NULL;
		const CValueRangeList& shootRange = shootRangeList.getShootRange();
		if (shootRange.size() > 0) {
			bestRange = shootRange.getMaxRangeWidth();
			//cout << "ShootWidth" << bestRange->getWidth() << endl;
			if (bestRange && bestRange->getWidth() > Param::Field::BALL_SIZE + 5) {	// 要求射门空档足够大
				_canshootonballpos = true;
			}
		}
		last_cycle = current_cycle;
	}

	return _canshootonballpos;
}


const bool CWorldModel::canPassOnBallPos(int current_cycle,CGeoPoint& passPos,CGeoPoint& guisePos,int myNum ){
	static int last_cycle = -1;
	static bool _canPassOnBallPos;

	if (last_cycle < current_cycle) {
		if (! Utils::PlayerNumValid(myNum)) {
			myNum = BestPlayer::Instance()->getOurBestPlayer();
			if (! Utils::PlayerNumValid(myNum)) {
				myNum = 1;
			}
		}
		const MobileVisionT ball=_pVision->Ball();
		const PlayerVisionT he=_pVision->TheirPlayer(BestPlayer::Instance()->getTheirBestPlayer());
		const PlayerVisionT me=_pVision->OurPlayer(myNum);
		CGeoPoint passPosOne,passPosTwo;

		CGeoPoint predictBall=BallSpeedModel::Instance()->posForTime(60,_pVision);
		NormalPlayUtils::generateTwoPassPoint(predictBall,passPosOne,passPosTwo);
		if (NormalPlayUtils::canPassBetweenTwoPos(_pVision,passPosOne,myNum))
		{
			passPos=passPosOne;
			guisePos=passPosTwo;
			_canPassOnBallPos=true;
		}else{
			if (NormalPlayUtils::canPassBetweenTwoPos(_pVision,passPosTwo,myNum))
			{
				passPos=passPosTwo;
				guisePos=passPosOne;
				_canPassOnBallPos=true;
			}else{
				_canPassOnBallPos=false;
			}
		}
		last_cycle=current_cycle;
	}
	return _canPassOnBallPos;
}

const bool CWorldModel::canKickAtEnemy(int current_cycle,CGeoPoint& kickDir, int myNum,int priority){
	static int last_cycle = -1;
	static bool _canKickAtEnemy;

	if (last_cycle < current_cycle) {
		if (! Utils::PlayerNumValid(myNum)) {
			myNum = BestPlayer::Instance()->getOurBestPlayer();
			if (! Utils::PlayerNumValid(myNum)) {
				myNum = 1;
			}
		}

		const MobileVisionT ball=_pVision->Ball();
		const PlayerVisionT me=_pVision->OurPlayer(myNum);
		const PlayerVisionT he=_pVision->TheirPlayer(BestPlayer::Instance()->getTheirBestPlayer());
		const double shootDir=KickDirection::Instance()->getRealKickDir();
		kickDir.setX(shootDir);
		if (fabs(shootDir)<Param::Math::PI/90){
			kickDir.setX(Param::Math::PI/90);
		}

		if (true)
		{
			if (priority==0)
			{
				if (kickDir.x()>0){
					kickDir.setX(kickDir.x()-Param::Math::PI/20);
				}else{
					kickDir.setX(kickDir.x()+Param::Math::PI/20);
				}
			}else if (priority=1)
			{
				if (kickDir.x()>0){
					kickDir.setX(kickDir.x()+Param::Math::PI/20);
				}else{
					kickDir.setX(kickDir.x()-Param::Math::PI/20);
				}
			}
			CGeoPoint theirGoal1=CGeoPoint(Param::Field::PITCH_LENGTH/2,-(Param::Field::GOAL_WIDTH/2+5));
			CGeoPoint theirGoal2(Param::Field::PITCH_LENGTH/2,Param::Field::GOAL_WIDTH/2+5);
			double dir1=(theirGoal1-ball.Pos()).dir();
			double dir2=(theirGoal2-ball.Pos()).dir();
			if (kickDir.x()>max(dir1,dir2)){
				kickDir.setX(max(dir1,dir2));
			}
			if (kickDir.x()<min(dir1,dir2)){
				kickDir.setX(min(dir1,dir2));
			}
			_canKickAtEnemy=true;
		}else{
			_canKickAtEnemy=false;
		}
		last_cycle=current_cycle;
	}
	return _canKickAtEnemy;

}

const string CWorldModel::getBallStatus(int current_cycle,int meNum){
	static int last_cycle=-1;
	static string lastState="None";
	if (last_cycle < current_cycle) {
		last_cycle=current_cycle;
		lastState=BallStatus::Instance()->checkBallState(_pVision,meNum);
	}
	return lastState;
}

void CWorldModel::setBallHandler(int num){
	TaskMediator::Instance()->setBallHandler(num);
}

// For FreeKick
bool CWorldModel::isBeingMarked(int myNum) {
	bool beingMarked = FreeKickUtils::isBeingMarked(_pVision, myNum);
	//cout << "beingMarked: " << beingMarked << endl;
	return beingMarked;
}

bool CWorldModel::isPassLineBlocked(int myNum, double allowance) {
	bool passLineBlocked = FreeKickUtils::isPassLineBlocked(_pVision, myNum, allowance);
	return passLineBlocked;
}

bool CWorldModel::isShootLineBlocked(int myNum, double range, double allowance) {
	bool shootLineBlocked = FreeKickUtils::isShootLineBlocked(_pVision, myNum, range, allowance);
	return shootLineBlocked;
}

bool CWorldModel::isShootLineBlockedFromPoint(CGeoPoint pos, double range) {
	bool shootLineBlocked = FreeKickUtils::isShootLineBlockedFromPoint(_pVision, pos, range);
	return shootLineBlocked;
}

void CWorldModel::drawReflect(int myNum) {
	FreeKickUtils::drawReflect(_pVision, myNum);
}

const CGeoPoint CWorldModel::getLeftReflectPos(int myNum) {
	CGeoPoint leftReflectPos = FreeKickUtils::getLeftReflectPos(_pVision, myNum);
	return leftReflectPos;
}

const CGeoPoint CWorldModel::getRightReflectPos(int myNum) {
	CGeoPoint rightReflectPos = FreeKickUtils::getRightReflectPos(_pVision, myNum);
	return rightReflectPos;
}

const CGeoPoint CWorldModel::getReflectPos(int myNum) {
	CGeoPoint reflectPos = FreeKickUtils::getReflectPos(_pVision, myNum);
	return reflectPos;
}

const CGeoPoint CWorldModel::getReflectTouchPos(int myNum, double y) {
	CGeoPoint reflectTouchPos = FreeKickUtils::getReflectTouchPos(_pVision, myNum, y);
	return reflectTouchPos;
}