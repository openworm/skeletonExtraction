#ifndef timerlogH
#define timerlogH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/


using namespace std;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

#include <list>
#include <time.h>
#include <Windows.h>
#include <string>
#include <logger/logger.h>

//---------------------------------------------------------------------------
class TimePrec {
public:
	int milisecs;
	bool start;
	char* name;
};
class Timerlog {
public:
	Timerlog(char* name);
	Timerlog::~Timerlog();
	void addStart(char* name);
	void addEnd();
	void logExecutionTimes();
private:
	list<TimePrec> times;
	char* timerName;
};
//---------------------------------------------------------------------------

#include "timerlog.inl"

#endif

