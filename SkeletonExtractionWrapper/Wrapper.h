
#include <meshes/structure.h>
#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>
#include <SDF_lib/sdf_extractor.h>
#include <SDF_lib/SDF_Library.h>

void loadModelToStructures(lbse::Extractor  *pLBSExtractor, structure::t3DModel * pModel, int * adaptVMdmax, int joiningTolerance);
void calculateSkinningData(GraphAlgorithms * pGa, ObjectSkeletonShaderData * pWormData, structure::t3DModel * pModel, SN::SkeletonNode * pSkeletonRoot, Array2D<float> * pDistanceMatrix, lbse::Extractor * pLBSExtractor, int numOfCtrlBones, int maxBoneMat);
void skeletonExtractionSDF(SN::SkeletonNode * pSkeletonRoot, lbse::Extractor * pLBSExtractor, sdf::Extractor * pSDFExtractor, int * adaptVMdmax);