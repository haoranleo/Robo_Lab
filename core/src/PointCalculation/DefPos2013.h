#ifndef _DEFPOS_2013_H_
#define _DEFPOS_2013_H_

#include <singleton.h>
#include "DefendUtils.h"

typedef struct  
{
	CGeoPoint leftD;	///<左后卫站位点
	CGeoPoint rightD;	///<右后卫站位点
	CGeoPoint middleD;	///<middle站位点
	CGeoPoint singleD;
	CGeoPoint goalieD;
	CGeoPoint sideD;

	CGeoPoint getSidePos(){return sideD;}

	CGeoPoint getLeftPos(){return leftD;}
	
	CGeoPoint getRightPos(){return rightD;}
	
	CGeoPoint getMiddlePos(){return middleD;}

	CGeoPoint getSinglePos(){return singleD;}

	CGeoPoint getGoaliePos(){return goalieD;}
} defend2013;

class CVisionModule;

class CDefPos2013
{
public:
	CDefPos2013();
	~CDefPos2013();	

   void generatePos(const CVisionModule* pVision);

	defend2013 getDefPos2013(const CVisionModule* pVision);

	double checkCollision(int myself, CGeoPoint targetPoint, const CVisionModule* pVision);

protected:
	defend2013 _defendPoints;	
	CGeoPoint _RdefendTarget;//防守目标点
	CGeoPoint _laststoredgoaliePoint;
	double	  _RdefendDir;//防守朝向	
	double _RleftgoalDir;
	double _RrightgoalDir;
	double _RmiddlegoalDir;
	
	double _RgoalieLeftDir;
	double _RgoalieRightDir;
	double _RleftmostDir;
	double _RrightmostDir;

	int _lastCycle;
};

typedef NormalSingleton< CDefPos2013 > DefPos2013;

#endif 