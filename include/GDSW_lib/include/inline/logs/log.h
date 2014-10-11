#ifndef logH
#define logH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/

#include <vector>
#include <windows.h>
#include <tnt/tnt.h>

using namespace std;

#include <logger/logger.h>

using namespace std;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

//#include "m_math.h"
#include "mmath/mmath.h"
using namespace mmath;

#include "logs/timerlog.h"

//---------------------------------------------------------------------------
#define LOG_LEVEL_METHODSTARTEND 3
#define LOG_LEVEL_DUMP 8
#define LOG_LEVEL_ITERATIONS 5
#define LOG_LEVEL_V_PARAMS 9
#define LOG_LEVEL_C_PARAMS 4
#define LOG_LEVEL_WARNING 1
#define LOG_LEVEL_ALGORITHMS 6
#define LOG_LEVEL_NOTE 2
#define LOG_LEVEL_ERROR 0
//---------------------------------------------------------------------------
class Log {
private:
	#define floatlength 40
	#define translength 5
	#define intlength 3
	#define boolength 1

	#define logTime true

	//static Log* i_log;
public:
	Log();
	~Log();
	int logLevel;

//---------------------------------------------------------------------------
	void log(int level, const char * text);
	void log(int level, const char * text, float f);
	void log(int level, const char * text, BYTE i);
	void log(int level, const char * text, int i);
	void log(int level, const char * text, bool b);
	void log(int level, const char * text, CVector3 v);
	void log(int level, TNT::Array1D< float > vector);
	void log(int level, std::vector< int > vector);
	void log(int level, TNT::Array2D< float > matrix);
	void log(int level, TNT::Array2D< double > matrix);
	void log(int level, TNT::Array2D< bool > matrix);
	void log(int level, float *, int length);
	void logTransformation(int level, TNT::Array2D< float > matrix);
	void logTransformationPointer(int level, float * matrix);
//---------------------------------------------------------------------------
};

#ifdef _LOG
	static Log logg = Log();
#endif

#include "log.inl"

#endif

