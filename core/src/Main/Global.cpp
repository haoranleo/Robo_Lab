#include "Global.h"

CVisionModule*   vision;
CKickStatus*    kickStatus;
CDribbleStatus* dribbleStatus;
CGDebugEngine*  debugEngine;
CWorldModel*	world;
CKickDirection* kickDirection;
CGPUBestAlgThread* bestAlg;
CDefPos2013* defPos2013;
CIndirectDefender* indirectDefender;
CTandemPos* tandemPos;
CBestPlayer* bestPlayer;
CDefenceInfo* defenceInfo;
CChipBallJudge* chipBallJudge;
CTTSEngine* tts;
CRecognizer* recognizer;
CHttpServer* httpServer;

void initializeSingleton()
{
	vision        = VisionModule::Instance();
	kickStatus    = KickStatus::Instance();
	dribbleStatus = DribbleStatus::Instance();
	debugEngine   = GDebugEngine::Instance();
	world		  = WorldModel::Instance();
	kickDirection = KickDirection::Instance();
	bestAlg		  = GPUBestAlgThread::Instance();
	defPos2013	  = DefPos2013::Instance();
	bestPlayer    = BestPlayer::Instance();
	defenceInfo = DefenceInfo::Instance();
	tandemPos = TandemPos::Instance();
	chipBallJudge = ChipBallJudge::Instance();
	tts	= TTSEngine::Instance();
	recognizer = Recognizer::Instance();
	httpServer = HttpServer::Instance();
	indirectDefender = IndirectDefender::Instance();
}