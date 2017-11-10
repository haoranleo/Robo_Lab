#ifndef _PLAY_BOOK_H_
#define _PLAY_BOOK_H_

class COptionModule;
class CVisionModule;
class CBasicPlay;
#include <vector>
using std::vector;
class CPlayBook{
public:
	CPlayBook(const COptionModule* pOption, const CVisionModule* pVision);
	CBasicPlay* getPlay();
	const char* currentPlayName() const;
private:
	bool handleSpecialPlays();
	CBasicPlay* _currentPlay; // 当前的play
	vector<CBasicPlay*> _allPlays; // 所有的play
	vector<CBasicPlay*> _applicablePlays; // 所有当前条件下适用的play
	const COptionModule* _pOption;
	const CVisionModule* _pVision;
};
#endif // _PLAY_BOOK_H_
