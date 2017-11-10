#include "TestSkillPlay.h"
#include <skill/Factory.h>
#include <tinyxml/ParamReader.h>
#include <param.h>
#include "GDebugEngine.h"
#include "math.h"
#include <KickStatus.h>
#include "TimeCounter.h"
#include "ChipBallJudge.h"
#include "KickDirection.h"
#include "IEvaluator.h"
#include "PlayInterface.h"
#include "TaskMediator.h"
#include "ShootRangeList.h"
#include <vector>
#include "CommandSender.h"
#include "RobotSensor.h"
#include "WorldModel.h"
#include <fstream>
#include "defence/EnemyDefendTacticAnalys.h"

using namespace std;
// for test


//////////////////////////////////////////////////////////////////////////
#include <ControlModel.h>
//////////////////////////////////////////////////////////////////////////

struct learnParam{
	double K;
	double D;
	double Tau;
	double a;
	double width;
	double wnum;
};

struct posParam{
	double x;
	double y;
	double w;
};
namespace{
	//xml
	 int sinT = 25;
	enum TEST_SKILL_T {
		SKILL_Stop = 0,
		SKILL_Getball,  //1
		SKILL_ADVANCE,	//2
		TEST_TIME_DELAY,//3
		SKILL_Learning, //4
		Goalie_RRT,		//5
		TEST_RRT		//6
	};
	int TEST_SKILL = 1;

	//执行小车定义
	int runner = 1;
	int passer = 2;
	IEvaluator * evaluatePoint = NULL ;
	//double x = 0;
	//double y = 0;

	//对方球门中心
	CGeoPoint theirGoalCenter;
	const CGeoPoint fieldCenter = CGeoPoint(0,0);
	const CGeoPoint dockPoint = CGeoPoint(50,30);
	const CGeoPoint getBallPoint1 = CGeoPoint(200, 0);
	const CGeoPoint getBallPoint2 = CGeoPoint(-250, -200);
	const CGeoPoint passerStartPoint = CGeoPoint(-200, 0);
	const double buff = 0;
	const double distprecise = 2.00;
	const double passerStopDist = 30.0;
	int firststate = 0;

	string file[3] = {"params\\wx.txt", "params\\wy.txt", "params\\ww.txt"};
	double w[3][50];
	learnParam p[3];

	void readParams(){
		for(int i = 0; i<3; i++){
			ifstream infile(file[i].c_str());
			string line;
			while (getline(infile,line)) {
				istringstream lineStream(line);
				string header;
				int num;
				lineStream>>header>>num;
				if(header == "K"){
					p[i].K = num;
				} else if(header == "D"){
					p[i].D = num;
				} else if(header == "Tau"){
					p[i].Tau = num;
				} else if(header == "a"){
					p[i].a = num;
				} else if(header == "width"){
					p[i].width = num;
				} else if(header == "w"){
					p[i].wnum = num;
					for(int j = 0; j < num; j++){
						getline(infile,line);
						istringstream lineStream2(line);
						int index;
						double wvalue;
						lineStream2>>index>>wvalue;
						w[i][j] = wvalue;
					}
				}
			}
		}
	}

	double calGussiFunc(double u, double c, double width)
	{
		return exp(-1.0*width*(u-c)*(u-c));
	}

	double calU(double t, double a, double tau)
	{
		return exp(-1.0*a*t/tau);
	}

	double getTargetValue(int cycle, int index, double start, double end){
		double ttu = calU(cycle, p[index].a, p[index].Tau);
		double item = p[index].Tau/sqrt(p[index].K);
		double item2 = item*item;
		double fNum = 0;
		for(int i = 0; i<p[index].wnum; i++){
			fNum += w[index][i] * calGussiFunc(ttu, i*1.0/p[index].wnum, p[index].width) * ttu;
		}
		double fDem = 0;
		for(int i = 0; i<p[index].wnum; i++){
			fDem += calGussiFunc(ttu, i*1.0/p[index].wnum, p[index].width);
		}
		double f = fNum/fDem;
		double res = (item2 - item2*exp(-1.0*cycle/item) - item*cycle*exp(-1.0*cycle/item))
					*(p[index].K*end + (end - start) * f) / (p[index].Tau * p[index].Tau);
		return res;
	}
}

CTestSkillPlay::CTestSkillPlay()
{
	DECLARE_PARAM_READER_BEGIN(CPlayBook)
	READ_PARAM(TEST_SKILL)
	DECLARE_PARAM_READER_END
	theirGoalCenter = CGeoPoint(Param::Field::PITCH_LENGTH/2.0,0.0);
}
void CTestSkillPlay::executePlay(const COptionModule* pOption, const CVisionModule* pVision)
{	
	bool runner_visible = pVision->OurPlayer(runner).Valid();
	bool ball_visible = pVision->Ball().Valid();
	CGeoPoint ball_pos = pVision->Ball().Pos();
	const PlayerVisionT& me = pVision->OurPlayer(runner);
	const PlayerVisionT& thepasser = pVision->OurPlayer(passer);
	const MobileVisionT& ball = pVision->Ball();
	static int _lastCycle = 0;
	static int count = 0;
	CEnemyDefendTacticAnalys analys;
	if (0/*!runner_visible || !ball.Valid()*/) {	//场上没有该小车，小车停止运动
		setTask(runner,PlayerRole::makeItStop(runner),HighestPriority);
        setTask(passer,PlayerRole::makeItStop(passer),HighestPriority);
	} else {
		if (0/*Utils::OutOfField(ball_pos,buff)*/) {
			setTask(runner,PlayerRole::makeItStop(runner),HighestPriority);
		} else {
			/************************************************************************/
			/* 每一个测试的SKILL对应一个数字编号，注意要写好注解	           */
			/************************************************************************/
			switch (TEST_SKILL)
			{
			case SKILL_Stop://stop
				{
					CGeoPoint target = CGeoPoint(0,0);
					setTask(runner,PlayerRole::makeItStop(runner),HighestPriority);
					break;
				}
			case SKILL_Getball://拿球
				{
					double finalDir = (CGeoPoint(-300,0) - pVision->Ball().Pos()).dir();
					CVector faceVector = CGeoPoint(-300,0) - ball_pos;
					double faceDir = faceVector.dir();
					setTask(runner,PlayerRole::makeItNoneTrajGetBall(runner,faceDir,CVector(0,0),0),HighestPriority);
				}
				break;
			case SKILL_ADVANCE:
				{
					CGeoPoint p1 = CGeoPoint(-150,0);
					CGeoPoint p2 = CGeoPoint(280,0);
					static bool go1 = false;
					if (me.Pos().dist(p1) < 10)
					{
						go1 = false;
					} else if (me.Pos().dist(p2) < 10)
					{
						go1 = true;
					}
					if (false == go1)
					{
						setTask(runner,PlayerRole::makeItSimpleGoto(runner,p2,0),HighestPriority);
					} else {
						setTask(runner,PlayerRole::makeItSimpleGoto(runner,p1,0),HighestPriority);
					}
				}
				break;
			case TEST_TIME_DELAY:
				{
					setTask(runner,PlayerRole::makeItTimeDelayTest(runner),HighestPriority);
				}
				break;
			case Goalie_RRT:
				{
				//	setTask(1,PlayerRole::makeItGoalie2012(1,1),HighestPriority);
				}
				break;
			case TEST_RRT:
				{
					CGeoPoint p2 = CGeoPoint(280,0);
					setTask(2,PlayerRole::makeItGoto(2,p2,0),HighestPriority);
				}
				break;
			case SKILL_Learning:
				{
					static bool hasread = false;
					if(!hasread){
						readParams();
						hasread = true;
					}
					if(pVision->Cycle() > 200){
						static posParam startP = {me.X(), me.Y(), me.Dir()};
						static posParam endP = {me.X()+44, me.Y()-22, me.Dir()-1.996};
						static int my_cycle = 0;
						double x = getTargetValue(my_cycle, 0, startP.x, endP.x);
						double y = getTargetValue(my_cycle, 1, startP.y, endP.y);
						double dir = getTargetValue(my_cycle, 2, startP.w, endP.w);
						double vx = getTargetValue(my_cycle+1, 0, startP.x, endP.x) - x;
						double vy = getTargetValue(my_cycle+1, 1, startP.y, endP.y) - y;
						double rotvel = getTargetValue(my_cycle+1, 2, startP.w, endP.w) - dir;
						CVector vel = CVector(vx, vy);
						my_cycle++;
						if(pVision->OurPlayer(runner).Pos().dist(CGeoPoint(endP.x, endP.y)) > 3){
							setTask(runner,PlayerRole::makeItSimpleGoto(runner, CGeoPoint(x,y), dir, vel, rotvel, 0),HighestPriority);
						} else{
							setTask(runner,PlayerRole::makeItStop(runner),HighestPriority);
						}
					}
				}
				break;
			default:
				break;
			}
			
		}
	}

	//rescueAction(pOption,pVision);

	return ;
}

void CTestSkillPlay::rescueAction(const COptionModule* pOption, const CVisionModule* pVision)
{
	//非调试小车停止运动
	for (int i = 1; i <= Param::Field::MAX_PLAYER; i++) {
		if (i == runner || 3 || 4) {
			continue;
		}
		if (i == passer) {
			continue;
		}

		setTask(i,PlayerRole::makeItStop(i),HighestPriority);
	}
}

CBasicPlay* CTestSkillPlay::transmitPlay(const COptionModule* pOption, const CVisionModule* pVision)
{
	return this;
}

//void CTestSkillPlay::testPTG(const COptionModule* pOption, const CVisionModule* pVision)
//{
//	bool runner_visible = pVision->OurPlayer(runner).Valid();
//	bool ball_visible = pVision->Ball().Valid();
//	const PlayerVisionT& me = pVision->OurPlayer(runner);
//	const MobileVisionT& ball = pVision->Ball();
//	const CVector me2ball = ball.Pos() - me.Pos();
//
//	// 设定目标点
//	CGeoPoint targetPoint = me.Pos() + Utils::Polar2Vector(me2ball.mod()-15,me2ball.dir());
//
//	if (runner_visible && ball_visible) {
//		//////////////////////////////////////////////////////////////////////////
//		CControlModel controlModel;
//
//		PlayerPoseT start;					// 初始状态
//		PlayerPoseT final;					// 目标状态
//		PlayerCapabilityT capability;		// 运动性能
//
//		start.SetPos(me.Pos());				// (x,y)
//		start.SetDir(me.Dir());				// (theta)
//		start.SetVel(me.Vel());				// (v,theta)
//		start.SetRotVel(me.RotVel());		// w
//
//		final.SetPos(targetPoint);
//		final.SetDir(me2ball.dir());
//		final.SetVel(CVector(0,0));
//		final.SetRotVel(0);	
//		
//		PTG_CTRL_MODE mode = OMNI;
//		controlModel.makePTGTrajectory(start,final,capability,mode);
//		//////////////////////////////////////////////////////////////////////////
//
//		//////////////////////////////////////////////////////////////////////////
//		bool clientDebug = true;
//		if (clientDebug) {
//			CVector lineVec = Utils::Polar2Vector(1000,0);
//			if (OMNI == mode) {
//				lineVec = Utils::Polar2Vector(1000,me.Vel().dir());
//			} else if (DIFF == mode) {
//				lineVec = Utils::Polar2Vector(1000,me.Dir());
//			}
//			GDebugEngine::Instance()->gui_debug_line(me.Pos(),me.Pos()+lineVec,COLOR_CYAN);
//
//			vector< vector<double> > pathList = controlModel.getFullPath();
//			for (size_t i = 0; i < pathList.size(); i ++) {
//				CVector cur_vec(pathList[i][0],pathList[i][1]);
//				if (OMNI == mode) {
//					start.SetDir(me.Vel().dir());
//				}
//
//				CVector cur_vec2 = Utils::Polar2Vector(cur_vec.mod(),Utils::Normalize(cur_vec.dir()+start.Dir()));
//				GDebugEngine::Instance()->gui_debug_x(start.Pos() + cur_vec2);
//			}
//		}		
//		//////////////////////////////////////////////////////////////////////////
//
//		//////////////////////////////////////////////////////////////////////////
//		PlayerPoseT nextPose = controlModel.getNextStep();
//		CVector globalVel = nextPose.Vel();
//		setTask(runner,PlayerRole::makeItRun(runner,globalVel.x(),globalVel.y(),nextPose.RotVel()), HighestPriority);
//		//////////////////////////////////////////////////////////////////////////
//	} else {
//		cout << "Error : not found vehicle or ball" << endl;
//		setTask(runner,PlayerRole::makeItStop(runner), HighestPriority);
//	}
//
//	return ;
//}