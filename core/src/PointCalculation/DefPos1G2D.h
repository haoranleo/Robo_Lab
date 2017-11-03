/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team：		SSL-ZJUNlict											*/
/* HomePage:	http://www.nlict.zju.edu.cn/ssl/WelcomePage.html		*/
/************************************************************************/
/* File:	  DefPos1G2D.h												*/
/* Func:	  1个门将和2个后卫联合封门									*/
/* Author:	  王群 2012-08-18											*/
/* Refer:	  ###														*/
/* E-mail:	  wangqun1234@zju.edu.cn									*/
/* Version:	  0.0.1														*/
/************************************************************************/

#ifndef _DEFPOS_1G_2D_H_
#define _DEFPOS_1G_2D_H_

#include "AtomPos.h"
#include <singleton.h>
#include "DefendUtils.h"




/** 
@brief  门将站位点和后卫站位点的结构体 */
typedef struct  
{
	CGeoPoint leftD;	///<左后卫站位点
	CGeoPoint rightD;	///<右后卫站位点
	CGeoPoint middleD;	///<门将站位点
	/** 
	@brief  左后卫站位点的对外次级接口 */
	CGeoPoint getLeftPos(){return leftD;}
	/** 
	@brief  右后卫站位点的对外次级接口 */
	CGeoPoint getRightPos(){return rightD;}
	/** 
	@brief  门将站位点的对外次级接口 */
	CGeoPoint getGoaliePos(){return middleD;}
} defend3;

class CVisionModule;

/**
@brief    1个门将和2个后卫联合封门
@details  本类的计算函数中，是将所有的位置信息取反后再计算的
@note	  注意！在调用本站位点组时，要注册车号，即同时调用setLeftDefender和setRightDefender函数*/
class CDefPos1G2D:public CAtomPos
{
public:
	CDefPos1G2D();
	~CDefPos1G2D();	

	/** 
	@brief  生成站位点的函数，内部使用 */
	virtual CGeoPoint generatePos(const CVisionModule* pVision);

	/** 
	@brief  给出实际射门线，并且更新防守目标点和防守朝向 */
	//CGeoLine getDefenceTargetAndLine(const CVisionModule* pVision);
	/** 
	@brief  找出敌方最有可能射门的队员 */
	//int DefendUtils::getEnemyShooter(const CVisionModule* pVision);
	///** 
	//@brief  计算守门员的站位点 */
	//CGeoPoint calcGoaliePoint(const CVisionModule* pVision,const CGeoPoint Rtarget,const double Rdir,const int mode = 0);
	/////** 
	//@brief  计算后卫的站位点 */
	//CGeoPoint DefendUtils::calcDefenderPoint(const CVisionModule* pVision,const CGeoPoint Rtarget,const double Rdir,const posSide Rside);

	/** 
	@brief  计算我方某一队员 对目标点至我方球门的射门角度的 阻碍角 */
	/*double DefendUtils::calcBlockAngle(const CGeoPoint& target,const CGeoPoint& player);*/
	/** 
	@brief  计算某一个点相对于直角坐标系坐标原点的对称点 */
	//CGeoPoint reversePoint(const CGeoPoint& p){return CGeoPoint(-1*p.x(),-1*p.y());}
	/** 
	@brief  禁区左半圆上的点是否在界内 */
	//bool DefendUtils::leftCirValid(const CGeoPoint& p);
	/** 
	@brief  禁区右半圆上的点是否在界内 */
	//bool DefendUtils::rightCirValid(const CGeoPoint& p);

	/** 
	@brief 外部取点接口 */
	defend3 getDefPos1G2D(const CVisionModule* pVision);
	/** 
	@brief  外部调用本点时，注册左后卫车号的接口
	@param  num 我方左后卫的车号
	@param	cycle 本帧下的帧数*/
	void setLeftDefender(const int num,const int cycle){_leftDefender = num;_leftDefRecord = cycle;}
	/** 
	@brief  外部调用本点时，注册右后卫车号的接口 
	@param  num 我方右后卫的车号
	@param	cycle 本帧下的帧数*/
	void setRightDefender(const int num,const int cycle){_rightDefender = num;_rightDefRecord = cycle;}

	/** 
	@brief  更新后卫信息，如果长时间没有调用本函数，则后卫队员设置为0号 */
	void updateDefenders(const CVisionModule* pVision);

	//为方便其他算法类的继承，此处使用protect
protected:
	defend3 _defendPoints;	
	defend3 _lastPoints;
	//防守目标对球门的方向数据，注意加R(reverse)字头的都是反向以后的变量
	CGeoPoint _RdefendTarget;//防守目标点
	double	  _RdefendDir;//防守朝向	
	double _RleftgoalDir;
	double _RrightgoalDir;
	double _RmiddlegoalDir;
	
	double _RgoalieLeftDir;
	double _RgoalieRightDir;
	double _RleftmostDir;
	double _RrightmostDir;
	
	int _lastCycle;	

private:
	int _leftDefender;
	int _rightDefender;
	int _leftDefRecord;
	int _rightDefRecord;
};

typedef NormalSingleton< CDefPos1G2D > DefPos1G2D;

#endif //_DEFPOS_1G_2D_H_