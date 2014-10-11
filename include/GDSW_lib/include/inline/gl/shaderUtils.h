#ifndef c_shaderUtilsH
#define c_shaderUtilsH
//---------------------------------------------------------------------------
/*#ifdef _MMGR
	#include "c_newdeleteOriginal.h"
#endif*/

#include <fstream>
#include <gl/glew.h>
#include <gl/glut.h>
#include <vector>

#ifdef _LOG
	#include <logs/log.h>
#endif


//using namespace std;

/*#ifdef _MMGR
	#include "c_newdeleteDebug.h"
#endif*/

//---------------------------------------------------------------------------
unsigned long getFileLength(std::ifstream& file);
void PrintShaderLog(const GLhandleARB obj_in);
GLubyte* loadShaderSource(char* filename);
int getVariableIntValue(std::string key);
//---------------------------------------------------------------------------

#include "shaderUtils.inl"

#endif
