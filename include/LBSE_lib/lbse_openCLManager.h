#ifndef c_openCLManagerH
#define c_openCLManagerH

#include <CL\cl.h>
#include <CL\cl_gl.h>
#include <CL\cl_gl_ext.h>
#include <CL\cl_platform.h>
#include <fstream>
#include <vector>

#pragma warning(push, 0)

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#pragma pop

#include <mmath/mmath.h>
using namespace mmath;

#ifdef _LOG
	#include <logs/timerlog.h>
#endif

#include "PCT_lib/pct_eig3.h"

struct OpenCLContext {
	cl_context context;
	cl_device_id device_id;
	bool transferInteroperabilityMesh;
};

//---------------------------------------------------------------------------

class OpenCLManager {
public:
	std::string solutionDir;
	std::string projectDir;

	int FloorPow2(int n);
	unsigned long getFileLength2(std::ifstream& file);
	char * loadKernelSource(char* filename);
	void initializeInteropContext(OpenCLContext * oclc);
	void openCL_LaplaceContraction(CVector3 * output, float* input, unsigned int numOfVertices, float * Lcomplete, unsigned int maxNeigh,  int* neighbourhoods, float wL, float * wH);
	void openCL_LaplaceContractionInterop(int * ite, CVector3 * output, float* input, unsigned int numOfVertices, float * Ecomplete, unsigned int maxNeigh,  int* neighbourhoods, unsigned int maxNeigh2,  int* neighbourhoods2, float wL, float * wH, float sL, int numOfIte, int vboID, OpenCLContext oclc);
	void openCL_LaplaceContraction2ring(CVector3 * output, float* input, unsigned int numOfVertices, float * Lcomplete, unsigned int maxNeigh,  int* neighbourhoods, unsigned int maxNeigh2,  int* neighbourhoods2, float wL, float * wH);

	void openCL_LaplaceContractionJacobi(CVector3 * output, float* input, unsigned int numOfVertices, float * Asquared, float * B, int vboID, OpenCLContext oclc);

	void printCLInfo();
	int isExtensionSupported(const char* support_str, const char* ext_string, size_t ext_buffer_size);
};

#endif