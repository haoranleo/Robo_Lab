#ifndef _COMPENSATE_
#define _COMPENSATE_

//用于计算一传一射时的角度补偿
#include <geometry.h>
#include "singleton.h"

class CCompensate{
public:
	CCompensate(); 
	~CCompensate();
	double checkCompensate(double ballspeed,double rawdir);
	double getKickDir(int playerNum, CGeoPoint kickTarget);
private:
	void readCompensateTable();
	double compensatevalue[100][50];
};

typedef NormalSingleton< CCompensate > Compensate;

#endif 