#pragma once

#include <windows.h>
#include <vector>
#include <tnt.h>
#include <jama/jama_qr.h>
#include <jama/jama_lu.h>

#pragma warning(push, 0)

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/math/constants/constants.hpp>

#include <boost/serialization/vector.hpp>
#include <boost/unordered_map.hpp>

#pragma pop

#include <viennacl/vector.hpp>
#include <viennacl/compressed_matrix.hpp>
#include <viennacl/linalg/cg.hpp>
#include <viennacl/linalg/qr.hpp>
#include <viennacl/ocl/backend.hpp>

using namespace TNT;
using namespace stdext;
using namespace JAMA;

#ifdef _LOG
	#include <logs/log.h>
	#include <logs/timerlog.h>
#endif

#include <meshes/structure.h>
#include <meshes/MeshGraph.h>

#include "LBSE_lib/lbse_openCLManager.h"

namespace LaplacianScheme {
	static const int LS_GLOBAL_JAMA_COTANGENT = 0;                // | OK              | GLOBAL | CPU          | WHILE   | MESH | cotangent schema->funguje, akurat je pomala
	static const int LS_GLOBAL_JAMA_POINTCLOUD = 1;               // | NEJDE           | GLOBAL | CPU          | WHILE   | PCD  | lokalna Deleunayovka->ide, ale laplacian sa sprava divno     
	static const int LS_GLOBAL_CPU_COTANGENT_PCL = 2;             // | NEJDE           | GLOBAL | CPU          | WHILE   | PCD  | PCL vytvori triangulaciu z PC, na nej cotangent schema
	static const int LS_GLOBAL_VCL_COTANGENT = 3;                 // | OK              | GLOBAL | VIENNACL/GPU | WHILE   | MESH | zrata sa Laplacian cez cotangent schemu, potom sa rata system paralellne cez ViennaCL
	static const int LS_GLOBAL_VCLLSM_COTANGENT = 4;              // | OK              | GLOBAL | VIENNACL/GPU | WHILE   | MESH | Laplacian cez cotangent schemu, potom sa rata system paralellne cez ViennaCL, trochu rychlejsie
	static const int LS_LOCAL_JAMA_COTANGENT = 5;                 // | OK,NEKON KOSTRA | LOCAL  | CPU          | WHILE   | MESH | Laplacian sa rata lokalne, rata sa sekvencne po vertexoch na CPU
	static const int LS_LOCAL_OCL_COTANGENT = 6;                  // | OK,NEKON KOSTRA | LOCAL  | OPENCL / GPU | WHILE   | MESH | Laplacian sa rata lokalne, rata sa paralelne per vertexh na GPU
	static const int LS_LOCAL_OCL_QR_INTEROP_COTANGENT = 7;       // | OK,NEKON KOSTRA | LOCAL  | OPENCL / GPU | INTEROP | MESH | Laplacian sa rata lokalne, rata sa paralelne QR dekompozicia na GPU, interop na vozualizaciu
	static const int LS_GLOBAL_OCL_JACOBI_COTANGENT = 8;          // | OK,NEKON NUMERI | GLOBAL | OPENCL / GPU | WHILE   | MESH | globalny system sa riesi iteracnou matodou, ktora sa riesi paralelne per vertex na GPU
	static const int LS_GLOBAL_OCL_JACOBI_INTEROP_COTANGENT = 9;  // | OK,NEKON NUMERI | GLOBAL | OPENCL / GPU | INTEROP | MESH | globalny system sa riesi iteracnou matodou, ktora sa riesi paralelne per vertex na GPU, interop
	static const int LS_GLOBAL_VCLLSMSDF_COTANGENT = 10;          // | OK              | GLOBAL | VIENNACL/GPU | WHILE   | MESH | Laplacian cez cotangent schemu, potom sa rata system paralellne cez ViennaCL, vyuzitie SDF
	static const int LS_LOCAL_JAMASDF_COTANGENT = 11;             // | OK,NEKON KOSTRA | LOCAL  | CPU          | WHILE   | MESH | Laplacian sa rata lokalne, rata sa sekvencne po vertexoch na CPU, vyuzitie SDF
}

// main method for contraction of meshgraph for all the laplacian schemas
void contractMeshGraph(int laplacianScheme, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingExtent, float * curOneRingArea, int kneigh, vector<std::set<int>> globalNeighbourhoods, OpenCLContext oclc, CVector3 * sdfValues, PointCloudTriangulation::DeleunayTriangulator * pTriangulator, OpenCLManager openCLManager); //ModelController::CModel * sdfController

//float GLaplace(float h, CVector3 v1, CVector3 v2);

// neighbourhood operation for point cloud contraction
void findNearestKNeighbours(MeshGraph * pMesh, int idx, int kneigh, int * neighbours);
float getNeighbourhoodKernelWidth(MeshGraph * pMesh, int idx, int kneigh, int * neighbours);

// collapse degenerate faces into an edge
void collapseDegenerates(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping);

// creates Laplacian matrix
Array2D< float > calculateLaplacianMatrix(MeshGraph * pMesh);

/* 
	global methods for meshgraph contraction
*/
void contractMeshGraphCPUCotangent(MeshGraph * pMesh,  boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphCPUPointCloud(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingExtent, float * curOneRingArea,  int kneigh, vector<std::set<int>> globalNeighbourhoods, PointCloudTriangulation::DeleunayTriangulator * pTriangulator);
void contractMeshGraphGPUVCL(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphGPUVCL_LSM(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphCPUPCL(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphGPUVCL_LSM_SDF(MeshGraph * pMesh, CVector3 * sdfHalfVec, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphGPUVCL_LSM_SDF_C(MeshGraph * pMesh, CVector3 * sdfHalfVec, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphParallelCPU_SDF(MeshGraph * pMesh, CVector3 * sdfHalfVec, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);

/* 
	local parallel methods for meshgraph contraction, both CPU and OpenCL
*/
void contractMeshGraphParallelCPU(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea);
void contractMeshGraphParallelOpenCL(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea, OpenCLManager openCLManager);
void contractMeshGraphParallelOpenCL2ring(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, int it, BYTE * LImage, float sL, float * curOneRingArea, OpenCLManager openCLManager);
void contractMeshGraphParallelOpenCLInterop(int * ite, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, int numOfIte,  OpenCLContext oclc, OpenCLManager openCLManager);
void contractMeshGraphParallelOpenCLJacobi(MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, OpenCLContext oclc, OpenCLManager openCLManager);
void contractMeshGraphParallelOpenCLJacobiInterop(int * ite, MeshGraph * pMesh, boost::unordered_map<int, vector<int> > &mgDegeneratesMapping, float sL, float * curOneRingArea, int numOfIte, OpenCLContext oclc, OpenCLManager openCLManager);

/* 
	this methods are used to simulate parallel computation of Laplacian contraction on CPU
*/

float solveParallelWithJama(int idx, Array2D< float >  L_global, float* input, unsigned int numOfVertices,  MeshGraph * pMesh,  unsigned int maxNeigh,  int* neighbourhoods);
float solveParallelWithJama_SDF(int idx, float * sdfHalfVec, Array2D< float >  L_global, float* input, unsigned int numOfVertices,  MeshGraph * pMesh,  unsigned int maxNeigh,  int* neighbourhoods);
//---------------------------------------------------------------------------