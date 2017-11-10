#ifndef __SHOOT_BALL_V2_H__
#define __SHOOT_BALL_V2_H__
#include <skill/PlayerTask.h>

/**********************************************************
* Skill: ShootBall
* Description: 只有touch和chase的shootball
* Author: dxh
* E-mail: dxiaohe@gmail.com
* Created Date: 2013.6.24
***********************************************************/

class CShootBallV2 :public CStatedTask {
public:
	CShootBallV2();
	virtual void plan(const CVisionModule* pVision);
	virtual bool isEmpty()const { return false; }
	virtual CPlayerCommand* execute(const CVisionModule* pVision);
protected:
	virtual void toStream(std::ostream& os) const { os << "Skill: ShootBall\n"; }

private:
	int _lastCycle;
	int _lastRunner;
};

#endif //__SHOOT_BALL_H__