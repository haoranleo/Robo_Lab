#ifndef _TEST_SKILL_PLAY_H_
#define _TEST_SKILL_PLAY_H_

#include "BasicPlay.h"

class CTestSkillPlay : public CBasicPlay{
public:
	CTestSkillPlay();
	virtual void executePlay(const COptionModule* pOption, const CVisionModule* pVision);
	virtual CBasicPlay* transmitPlay(const COptionModule* pOption, const CVisionModule* pVision);
	virtual const char* playName() const { return "TestSkillPlay"; }

private:
	void rescueAction(const COptionModule* pOption, const CVisionModule* pVision);

//	void testPTG(const COptionModule* pOption, const CVisionModule* pVision);

};
#endif // _TEST_SKILL_PLAY_H_