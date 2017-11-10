#ifndef __SHOOT_BALL_H__
#define __SHOOT_BALL_H__
#include <skill/PlayerTask.h>

/**********************************************************
* Skill: ShootBall
* Description: 将ShootAgent放入skill中调试
* Author: zhyaic
* E-mail: zhyaic@gmail.com
* Created Date: 2012.5.17
***********************************************************/

class CShootBall :public CStatedTask {
public:
	CShootBall();
	virtual void plan(const CVisionModule* pVision);
	virtual bool isEmpty()const { return false; }
	virtual CPlayerCommand* execute(const CVisionModule* pVision);
protected:
	virtual void toStream(std::ostream& os) const { os << "Skill: ShootBall\n"; }

private:
	int _lastCycle;
	int _lastRunner;
	int _out_inter_cnt;
	bool _out_inter;
};

#endif //__SHOOT_BALL_H__