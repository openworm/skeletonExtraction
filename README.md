PROJECTS IN VISUAL STUDIO SOLUTION (COMPONENTS UNDER DEVELOPMENT):

1) GDSW_lib [Geodesic Distance Skinning Weights]
- library computes geodesic distance on the mesh surface using Floyd-Warshall algorithm. 
- the geodesic distance is than used for computation of skinning weights
- it is a very trivial approach how skinning weights can be computed
- more complex approach might be needed in the future

:: main functions ::
void FloydWarshall(MeshGraph * pMesh, Array2D<float>& M)
- computes distances on the graph

2) LBSE_lib [Laplacian Based Skeleton Extraction]
- library uses Laplacian based skeleton contraction for extraction of the skeleton from input mesh
- the resulting skeleton is a tree structure

:: main functions ::
void computeSkeleton(t3DModel *pModel, int sourcePointID, SN::SkeletonNode * skeleton, int * ite, bool &recreateOperator, float modelMaxDim)
- computes skeleton from input mesh

3) PCT_lib [Point Cloud Triangulation]
- library computes triangulation from input point cloud
- the global triangulation is composed from local Delaunay triangulations

:: main functions ::
void computeLocalTriangulationFromPoints(int index, int numOfPoints, float * points, int &numOfIndices, int ** indices, std::vector<std::set<int>> globalNeighbourhoods, float * nor, bool visualization = false)
- computes local triangulation

void computeGlobalTriangulationFromPoints(int numOfPoints, float * points, int &numOfIndices, int ** indices, float ** normals, bool visualization = false);
- computes global triangulation

4) SDF_lib [Shape Diameter Function]
- not yet needed, but I have left it there for keeping compatibility of source files between my repo and this repo

PREPROCESSOR DEFINITIONS:

D_SCL_SECURE_NO_WARNINGS;_NLOG;_NMMGR;NOMINMAX;

HOW DOES IT WORK:

When a process wants to extract a skeleton from an input mesh, the process has to call "computeSkeleton" method from 
LBSE_lib. Next, for extraction of skinning weights, the GDSW_lib has to be used.

LIBRARIES NEEDED TO RUN EXTRACTION:

- Boost - used for serialization of the skeletons, can be removed later if serialization is not needed
- TNT - vectors and matrices used in Jama
- Jama - linear solver for TNT vectors and matrices
- ViennaCL - linear solver with OpenCL support...the solver can be parallelized on the GPU
- Fade2D - used for Delaunay triangulation
