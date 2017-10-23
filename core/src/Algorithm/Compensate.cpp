#include "Compensate.h"
#include "Global.h"
#include <./tinyxml/ParamReader.h>

namespace{
	bool record = false;
	bool IS_SIMULATION = false;
}

CCompensate::CCompensate(){
	DECLARE_PARAM_READER_BEGIN(General)
	READ_PARAM(IS_SIMULATION)
	DECLARE_PARAM_READER_END
	 readCompensateTable();
}

void CCompensate::readCompensateTable(){
	const string path ="play_books\\";
	string fullname = path +"data.txt";
	ifstream infile(fullname.c_str());
	if (!infile) {
		cerr << "error opening file data"<< endl;
		exit(-1);
	}
	string line;
	int rowcount;
	int columncount;
	getline(infile,line);
	istringstream lineStream(line);
	lineStream>>rowcount>>columncount;
	for (int i =0;i<rowcount;i++){
		getline(infile,line);
		istringstream lineStream(line);
		for(int j=0;j<columncount;j++){
			lineStream>>compensatevalue[i][j];
		}
	}
}

double CCompensate::getKickDir(int playerNum, CGeoPoint kickTarget){
	CVisionModule* pVision = vision;
	const MobileVisionT & ball = pVision->Ball();
	const PlayerVisionT & kicker = pVision->OurPlayer(playerNum);
	double rawkickdir = (kickTarget - kicker.Pos()).dir();
	double ballspeed =ball.Vel().mod();
	double tempdir = (Utils::Normalize(Utils::Normalize(pVision->Ball().Vel().dir()+Param::Math::PI)-(kickTarget - kicker.Pos()).dir()))*180/Param::Math::PI;
	int ratio = 0;
	if (tempdir>0){
		ratio = 1;
	}else{
		ratio = -1;
	}
	double compensatevalue;
	double rawdir=fabs((Utils::Normalize(Utils::Normalize(pVision->Ball().Vel().dir()+Param::Math::PI)-(kickTarget - kicker.Pos()).dir()))*180/Param::Math::PI);
	if (rawdir > 80){
		rawdir = 80;
		//cout<<"kickdirection rawdir changed to 80"<<endl;
	}
	compensatevalue = checkCompensate(ballspeed,rawdir);
	if (pVision->Ball().Vel().mod()<10){
		//cout<<"Compensate 0 because the ball's velocity is too small (<10)"<<endl;
		compensatevalue = 0;
	}
	if(IS_SIMULATION){
		compensatevalue = 0;
	}
	double realkickdir= Utils::Normalize(Utils::Normalize(ratio*compensatevalue*Param::Math::PI/180)+rawkickdir);
	//cout<<vision->Cycle()<<" "<<ballspeed<<" "<<rawdir<<" "<<compensatevalue<<endl;
	return realkickdir;
}

double CCompensate::checkCompensate(double ballspeed,double rawdir){
	double compensate = 0;
	if (ballspeed<195){
		ballspeed = 195;
	}
	if (ballspeed > 650){
		ballspeed = 650;
	}
	int column = ceil(rawdir/5);
	int  row =ceil((ballspeed-195)/5);
	if (row<1){
		row = 1;
	}
	if (column < 1){
		column = 1;
	}
	double distleft = rawdir -(column-1)*5;
	double distright = column*5 - rawdir;
	double distup = ballspeed - ((row -1)*5+195);
	double distdown = row*5+195 - ballspeed;
	double leftfactor = distright/5;
	double rightfactor = distleft/5;
	double upfactor = distdown/5;
	double downfactor = distup/5;
	compensate = (leftfactor*compensatevalue[row -1][column -1] +rightfactor*compensatevalue[row-1][column])*upfactor
		+(leftfactor*compensatevalue[row][column -1] +rightfactor*compensatevalue[row][column ])*downfactor;
	if (record == true){
		compensate = 0;
	}
	return compensate;
}