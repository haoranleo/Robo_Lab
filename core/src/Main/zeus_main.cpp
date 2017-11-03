#include <WinSock2.h>
#include <iostream>
#include <fstream>
#include <ServerInterface.h>
#include <weerror.h>
#include <WorldModel.h>
#include "DecisionModule.h"
#include "ActionModule.h"
#include <tinyxml/ParamReader.h>
#include <CtrlBreakHandler.h>
#include <VisionReceiver.h>
#include <TimeCounter.h>
#include <GDebugEngine.h>
#include "bayes/MatchState.h"
#include "gpuBestAlgThread.h"
#include "Global.h"
#include "DefendUtils.h"

/*! \mainpage Zeus - Run for number one
*
* \section Introduction
*
*	ZJUNlict is robot soccer team of Robocup in Small Size League.
*			
* \section Strategy
*	Frame : GameState -> Play -> Agent -> Skill -> Motion Control -> Wireless
*
* \subsection step1: GameState
* \subsection step2: Play
* \subsection step3: Agent
* \subsection step4: Skill
* \subsection step5: Motion Control : PathPlan and Trajectory Generation
* 
* etc...
*/

/// <summary> XML configure file for zues. </summary>
std::string CParamReader::_paramFileName = "zeus2005";

/// <summary> For Synchronize strategy. </summary>
extern CEvent *visionEvent;
extern CMutex *decisionMutex;
/// <summary> For GPU. </summary>
CMutex* _best_visiondata_copy_mutex = 0;
CMutex* _value_getter_mutex = 0;

CUsecTimer _usecTimer;
CUsecTimer _usecTime2;

using Param::Latency::TOTAL_LATED_FRAME;

namespace { 
	// 调试输出开关
	bool VERBOSE_MODE = true;
}

/// <summary> The graphical user interface socket </summary>
UDPSocket gui_socket;
UDPSocket gui_recordPos_socket;
int self_port = 20002;
int gui_port = 20001;
int RECORDPOS_PORT =0;
bool gui_pause = false;
bool wireless_off = false;
bool send_debug_msg = false;
bool record_run_pos_on =false;
CThreadCreator* gui_recv_thread = 0;
CThreadCreator* gui_recordPos_thread = 0;

/// <summary>	Executes the graphical user interface send operation. </summary>
///
/// <remarks>	cliffyin, 2011/7/26. </remarks>
///
/// <param name="cycle">	The cycle. </param>

static void do_gui_send(int cycle);

/// <summary>	Executes the graphical user interface recv operation. </summary>
///
/// <remarks>	cliffyin, 2011/7/26. </remarks>

DWORD THREAD_CALLBACK do_gui_recv(LPVOID lpParam);

DWORD THREAD_CALLBACK do_recordPos_recv(LPVOID lpParam);

/**
@brief 程序应用入口：Main entry-point for this application.
@param argc 命令行参数个数：Number of command-line arguments.
@param argv 命令行参数：Array of command-line argument strings.
@retval 0 成功：success!
*/

#ifndef RELEASE_LIB
bool USE_SMALL_FIELD = true; // 是否仿真
int main(int argc, char* argv[])
{
	/************************************************************************/
	/* 各模块进行初始化                                                     */
	/************************************************************************/

	// 载入策略的xml配置参数
	PARAM_READER->readParams();

	DECLARE_PARAM_READER_BEGIN(General)
		READ_PARAM(USE_SMALL_FIELD)
		READ_PARAM(RECORDPOS_PORT)
	DECLARE_PARAM_READER_END

	if(!USE_SMALL_FIELD){
		Param::Field::PITCH_LENGTH = 809;// 场地长
		Param::Field::PITCH_WIDTH = 605;// // 场地宽
		Param::Field::RATIO = 1.34;//有关比例的使用
		Param::Field::PITCH_MARGIN = 1; // 边界宽度
		Param::Field::CENTER_CIRCLE_R =	50; // 中圈半径
		Param::Field::PENALTY_AREA_WIDTH = 250 ; // 禁区宽度(禁区修改参数 195)
		Param::Field::PENALTY_AREA_DEPTH = 100 ; // 禁区深度(禁区修改参数 80)
		Param::Field::PENALTY_AREA_R = 100 ; // 两个圆弧，2008年的参数，在这里没有用(禁区修改参数 80)                       
		Param::Field::PENALTY_AREA_L = 50 ; //  连接两个圆弧的线段，2008年参数，在这里没有用
		Param::Field::OUTER_PENALTY_AREA_WIDTH = 175; // 外围禁区宽度(界外开球时不能站在该线内)
		Param::Field::FREE_KICK_AVOID_BALL_DIST = 50; // 开任意球的时候,对方必须离球这么远
		Param::Field::FIELD_WALL_DIST = 20;  // 场地护栏到边界的距离
		Param::Field::GOAL_DEPTH = 18;
		Param::Field::GOAL_WIDTH = 100;
		DefendUtils::changeBasicData();
		//cout<<Param::Field::RATIO<<endl;
	}

	// 初始化各单例
	initializeSingleton();

	// 打开记录帧率的文件，方便查看图像的真实帧率
	ofstream file;
	file.open("./frequence.txt",ios::out);

	// 为了在程序崩溃的时候让所有小车停止,在catch中调用action->stopAll(),所以将ActionModule的定义部分拿出来
	// TODO ：尚未经过调试
	// 键盘起停控制模块
	CCtrlBreakHandler breakHandler;

	// 程序选项模块
	COptionModule option(argc,argv);

	// 程序图像模块
	vision->registerOption(&option);

	// 程序策略模块
	CDecisionModule decision(&option, vision);

	// 程序动作模块
	CActionModule action(&option, vision, &decision);

	// 世界模型注册图像模块
	WORLD_MODEL->registerVision(vision);

	// 贝叶斯比赛形势估计器初始化
	MATCH_STATE->initialize(&option,vision);

	/************************************************************************/
	/* 进入 Main Loop  闭环决策                                       */
	/************************************************************************/
	// 进入主框架部分
	try {
		// 计算点的线程初始化
		_best_visiondata_copy_mutex = new CMutex; 
		_value_getter_mutex = new CMutex;
		GPUBestAlgThread::Instance()->initialize(VISION_MODULE);


		// 启动和 ZjuNlictClient 调试面板通信的 Socket
		gui_socket.init(self_port);

		// GUI的接受线程
		gui_recv_thread = new CThreadCreator(do_gui_recv, 0);

		// DefendDebug线程
		if (RECORDPOS_PORT)
		{
			gui_recordPos_socket.init(RECORDPOS_PORT);
			gui_recordPos_thread = new CThreadCreator(do_recordPos_recv, 0);
			
		}
	
		// 视觉接受模块
		const VisionReceiver *receiver = VisionReceiver::instance(&option, &breakHandler); 

		// 视觉信息对象
		CServerInterface::VisualInfo visionInfo;

		// 包含比分的裁判盒信息
		RefRecvMsg refRecvMsg;

		// 正常模式，设置为高的优先级
		SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);	    

		// 程序停止标志位
		bool stop = false;



		// 主循环，可响应键盘起停操作

		while (breakHandler.breaked() < 1) {
			double usedtime = 0.0f;
			double totalusedtime = 0.0f;

			// 从仿真/图像获取场上信息		
			visionEvent->wait();
			// [2010-02-18 yph 加锁]
			decisionMutex->lock();

			// 获取图像信息
			if (! receiver->getVisionInfo(visionInfo, refRecvMsg)) {
				std::cout << "no vision input" << std::endl;
				stop = true;
			}

			// 记录实际运行的帧率，方便查看
			if (VERBOSE_MODE) {
				if (visionInfo.cycle % 2 ==0) {
					_usecTime2.start();
				} else {
					_usecTime2.stop();					
					if (file.is_open()) {
						file<<" Frequence : "<<visionInfo.cycle<<" : "<<_usecTime2.time()/1000.0f<<"\n";
					}						
				}
				_usecTimer.start();
				vision->SetRefRecvMsg(refRecvMsg);
				vision->SetNewVision(visionInfo);
				//lua_getglobal(L, "execute");
				//tolua_pushusertype(L, this, "CVisionModule");
				//lua_call(L,1,0);
				_usecTimer.stop();

				if (visionInfo.cycle%10 == 0) {
					usedtime = _usecTimer.time()/1000.0f;
					totalusedtime += usedtime;
					//std::cout<<" VisionModule : "<<visionInfo.cycle<<" : "<<usedtime<<"\n";
				}
			} else {
				// 更新图像模块，此步骤非常重要，为决策模块做准备
				vision->SetRefRecvMsg(refRecvMsg);
				vision->SetNewVision(visionInfo);
			}

			// 键盘结束程序处理
			if (0 == breakHandler.breaked()) {
				stop = true;
			}


			// 调试面板暂停处理
			if (breakHandler.halted() || gui_pause) {
				decision.DoDecision(true);
			} else {
				// 进行策略选取，此步骤非常重要，进行实际的决策过程
				decision.DoDecision(stop);
			}

			// 进行命令生成，此步骤非常重要，进行决策后的指令生成并发送
			if (! wireless_off) {
				action.sendAction(visionInfo.ourRobotIndex);
			} else {
				action.sendNoAction(visionInfo.ourRobotIndex);
			}

			// [2010-02-18 yph 解锁]
			decisionMutex->unlock();	

			/************************************************************************/
			/* GUI 调试通信                                                         */
			/************************************************************************/
			do_gui_send(visionInfo.cycle);

			// 程序停止处理 ：100ms再退出程序，确保stop指令发出
			if (stop) {
				Sleep(100);
				VisionReceiver::destruct();
				break;
			}
		}
	} catch(WEError e) {
		// 程序异常，停止小车
		action.stopAll();
		e.print();
		std::cout << "Usage : " << argv[0] << " [s(l|r)] [t(1|2)] [c(y|b)] [n(1..5)]" << Param::Output::NewLineCharacter;
		std::cout << "Press any key to exit." << Param::Output::NewLineCharacter;
		getchar();
	}

	/************************************************************************/
	/* 程序退出                                                             */
	/************************************************************************/
	// 关闭打开的帧率记录文件
	if (file.is_open()) {
		file.close();
	}

	// 停止小车
	action.stopAll();
	std::cout << "system exits, press any key to continue..." << std::endl;
	getchar();

	return 0;
}

#endif
/**
@brief 发送点线等调试信息到控制面板（客户端ZjuNlictClient)
@param cycle 标识当前的时间周期
*/
static void do_gui_send(int cycle)
{
	// 有客户端连接的时候才发送，否则清空发送缓冲
	if (send_debug_msg) {
		net_gdebugs new_msgs;
		new_msgs.totalnum = 0;
		new_msgs.totalsize = 0;
		memset(new_msgs.msg, 0, 500*sizeof(net_gdebug));

		// 修改后的UDP版本，一次传包
		while(!GDebugEngine::Instance()->is_empty()){
			net_gdebug new_msg = GDebugEngine::Instance()->get_queue_front();
			new_msgs.totalsize = vision->Cycle();
			new_msgs.msg[new_msgs.totalnum] = new_msg;
			GDebugEngine::Instance()->pop_front();
			new_msgs.totalnum++;
			if(new_msgs.totalnum >= 500){
				cout<<"!!!!!!!!!To Many Debug Message To Client!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
			}
		}
		gui_socket.sendData(gui_port, (char*)(&new_msgs), 8+sizeof(net_gdebug)*new_msgs.totalnum, "127.0.0.1");
	} else {
		while ( !GDebugEngine::Instance()->is_empty() ) {
			GDebugEngine::Instance()->pop_front();
		}
	}

	return;
}

/**
@brief 接收控制面板的控制命令（客户端ZjuNlictClient)
*/
DWORD THREAD_CALLBACK do_gui_recv(LPVOID lpParam)
{
	while (true) {
		char msg[net_gui_in_maxsize];
		if (gui_socket.receiveData(msg, net_gui_in_maxsize)) {
			net_gcommand* ngc = (net_gcommand*)msg;
			switch (ngc->msgtype) 
			{
			case NET_GUI_CONTROL:
				{
					if ( strcmp(ngc->cmd, "r") == 0 )
						gui_pause = false;
					else if ( strcmp(ngc->cmd, "p") == 0 )
						gui_pause = true;
					else if ( strcmp(ngc->cmd, "wireless_on") == 0 )
						wireless_off = false;
					else if ( strcmp(ngc->cmd, "wireless_off") == 0 )
						wireless_off = true;
					else if ( strcmp(ngc->cmd, "strategy_on") == 0 )
						send_debug_msg = true;
					else if ( strcmp(ngc->cmd, "strategy_off") == 0 )
						send_debug_msg = false;
					break;
				}
			default:
				break;
			}
		}
		Sleep(100);
	}
}


DWORD THREAD_CALLBACK do_recordPos_recv(LPVOID lpParam)
{
	while (true) {
		char msg[net_gui_in_maxsize];
		if (gui_recordPos_socket.receiveData(msg, net_gui_in_maxsize)) {
			net_gcommand* ngc = (net_gcommand*)msg;
			switch (ngc->msgtype) 
			{
			case NET_GUI_CONTROL:
				{
					if ( strcmp(ngc->cmd, "record_run_pos_on") == 0 )
						record_run_pos_on = true;
					else if ( strcmp(ngc->cmd, "record_run_pos_off") == 0 )
						record_run_pos_on = false;
					break;
				}
			default:
				break;
			}
		}
		Sleep(100);
	}
}