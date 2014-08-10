#ifndef c_PCLManagerH
#define c_PCLManagerH
//---------------------------------------------------------------------------

//#include "c_config.h"
#include <meshes/MeshGraph.h>
#include "PCT_lib/pct_eig3.h"

#include "PCT_lib/pct_PointCloudTriangulation.h"

//#include <pcl/common/pca.h>
#include <jama/jama_svd.h>

#include <mmath/mmath.h>

/*#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>*/

#include <fade2d/Fade_2D.h>
#include <fade2d/Point2.h>

using namespace mmath;

//void greedyProjection(MeshGraph *in_mesh, MeshGraph *out_mesh);
//void getTangentPlanePCAUsingPCL(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3);

void getTangentPlanePCAWithSVD_orig(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3);
void getTangentPlanePCA_orig(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3);

/*
CVector2 Space2Plane_orig(CVector3 q, CVector3 o, CVector3 e1, CVector3 e2);
CVector3 Plane2Space_orig(CVector2 q, CVector3 o,CVector3 e1, CVector3 e2);
CVector3 ProjectPointIntoPlane_orig(CVector3 q, CVector3 o, CVector3 n);
Array2D<bool> computeLocalTrianglation_orig(int i, MeshGraph * pMesh, vector<std::set<int>> globalNeighbourhoods);
void findClosestNeighWithCentering_orig(int i, MeshGraph * pMesh, std::set<int> &neighs, CVector3 &cm, int * distances);
bool checkLocalNeighAngles_orig(int i, MeshGraph * pMesh, std::set<int> &neighs);
vector<std::set<int>> computeGlobalNeighbourhood_orig(MeshGraph * pMesh);
int getKNeigh_orig(MeshGraph * pMesh);*/

void computeTriangulationFromLocalDelaunays(MeshGraph * pMesh, PointCloudTriangulation::DeleunayTriangulator * pTriangulator);

//---------------------------------------------------------------------------

#endif