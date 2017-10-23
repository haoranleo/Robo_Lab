/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team：		SSL-ZJUNlict											*/
/* HomePage:	http://www.nlict.zju.edu.cn/ssl/WelcomePage.html		*/
/************************************************************************/
/* File:	  gpuBestAlgThread.h										*/
/* Func:	  计算最佳跑位点的线程，支持CPU模式和GPU模式				*/
/* Author:	  王群 2012-08-18											*/
/*            叶杨笙 2014-03-01                                         */
/* Refer:	  ###														*/
/* E-mail:	  wangqun1234@zju.edu.cn									*/
/* Version:	  0.0.1														*/
/************************************************************************/

#ifndef _GPU_BEST_ALG_THREAD_H
#define _GPU_BEST_ALG_THREAD_H

#include <fstream>
#include <MultiThread.h>
#include <singleton.h>
#include "VisionModule.h"
#include "geometry.h"

#define GRIDX 815
#define GRIDY 610
#define  C_POINT_PASS_OFF 200
#define  C_MAX_PLAYER		6

enum ReCalModeT {
	CIRCLE_MODE = 0,
	RECT_MODE,
};
/**
@brief    计算最佳跑位点的线程，支持CPU模式和GPU模式
@details  本类是独立于主线程的算法线程，提供全场多区域的最佳跑位点的计算。全局变量USE_GPU
为1时使用GPU进行跑位计算，为0时使用CPU进行跑位计算。为保持每块区域的跑位点计算能够保持稳定，
针对一个固定的或者变动极小的区域，需要在取点时注入一个固定的区域编号。
@note	注意，在每一帧当中，在使用对外接口getBestPoint函数前，要调用一次setSendPoint函数，设定
传球的点。一般此处为球当前所在的点*/

//C_shootRange返回值结构
typedef struct{
	float biggestAngle;
	float bestDir;
} C_shootParam;
//阻挡方向结构体，用于保存数据
typedef struct{
	float leftDir;
	float rightDir;
	float leftDist;
	float rightDist;
} C_blockDirection;

class CGPUBestAlgThread {
public:
	CGPUBestAlgThread();
	~CGPUBestAlgThread();

public:
	/**
	@brief	注册视觉信息并且开启算法线程*/
	void initialize(CVisionModule* _pVision);
	/**
	@brief	本模块的对外接口
	@param	leftUp 取点区域的左上点(x轴向正右方向的直角坐标系中)
	@param	rightDown 取点区域的右下点(x轴向正右方向的直角坐标系中)
	@param	pointKeeperNum 取点保持系统的编号，当前系统可识别区域编号0~9
	@param	reCalMode 取点区域为矩形还是圆形。当前系统只支持矩形，即此处传入RECT_MODE
	@param	forceReset 每次重新计算最佳跑位点，默认为false。传入true将不会使用取点保持系统*/
	CGeoPoint getBestPoint(const CGeoPoint leftUp = CGeoPoint(300,-200),const CGeoPoint rightDown = CGeoPoint(0,200), const int pointKeeperNum = 0, const ReCalModeT& reCalMode = RECT_MODE, bool forceReset = false);//pointKeeper支持同时多区域搜点时的最佳点保持功能
	
	/**
	@brief 全场所有点的势能值的生成*/	
	void generatePointValue();
	// 最佳 xx 点
	// ...

	/**
	@brief	判断某一区域前一帧的最佳跑位点是否仍有效*/
	bool isLastOneValid(const CGeoPoint& p);
	/**
	@brief	设定传球点
	@param	passPoint 当前传球点*/
	void setSendPoint(const CGeoPoint passPoint);
	/**
	@brief	取点的势能值的对外接口
	@param	p 要获取势能值的点*/
	double getPosPotential(const CGeoPoint p);

	// 以下为一些评价函数 yys
	void C_evaluateReceive_Geo(const CGeoPoint p, float& reverseReceiveP);

	C_shootParam C_shootRange_Geo(const CGeoPoint p);

	void C_evaluateShoot_Geo(const CGeoPoint p, float& reverseShootP);

	void C_evaluateOthers_Geo(const CGeoPoint p, float& reverseOtherP);

	float getPointPotential(int x,int y);

public:
	/**
	@brief	开启线程所用的函数*/
	static CThreadCreator::CallBackReturnType THREAD_CALLBACK doBestCalculation(CThreadCreator::CallBackParamType lpParam);

private:
	float (*_PointPotential)[GRIDY];	///<势能值数组
	CVisionModule* _pVision;			///<图像指针
	CGeoPoint pointKeeper[10];			///<取点保持数组
	int _lastCycle[10];					///<取点保持系统用到的前一帧帧号

	CGeoPoint sendPoint;				///<传球点，一般为球所在的点

	int halfLength; // 场地半长
	int halfWidth;  // 场地半宽
	int halfGoalWidth;
};

typedef NormalSingleton< CGPUBestAlgThread > GPUBestAlgThread;
#endif