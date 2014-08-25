###PROJECTS:###
*All projects are in this repository's [Visual Studio Solution](http://msdn.microsoft.com/en-us/library/bb165951(v=vs.80).aspx) [file](https://github.com/openworm/skeletonExtraction/blob/master/OpenWormSkeletonExtraction.sln).*

**1) GDSW_lib** [Geodesic Distance Skinning Weights]
- Library computes geodesic distance on the mesh surface using Floyd-Warshall algorithm. 
- The geodesic distance is than used for computation of skinning weights
- It is a very trivial approach how skinning weights can be computed
- A more complex approach might be needed in the future

*Main functions:*
```
void FloydWarshall(MeshGraph * pMesh, Array2D<float>& M)
```
- Computes distances on the graph

**2) LBSE_lib** [Laplacian-Based Skeleton Extraction]
- Library uses Laplacian based skeleton contraction for extraction of the skeleton from input mesh
- The resulting skeleton is a tree structure

*Main functions:*
```
void computeSkeleton(t3DModel *pModel, 
                     int sourcePointID, 
                     SN::SkeletonNode * skeleton, 
                     int * ite, 
                     bool &recreateOperator, 
                     float modelMaxDim)
```
- Computes skeleton from input mesh

**3) PCT_lib** [Point Cloud Triangulation]
- Library computes triangulation from input point cloud
- The global triangulation is composed from local Delaunay triangulations

*Main functions:*
```
void computeLocalTriangulationFromPoints(int index, int numOfPoints, 
                                         float * points, int &numOfIndices, 
                                         int ** indices, 
                                         std::vector<std::set<int>> globalNeighbourhoods, 
                                         float * nor, bool visualization = false)
```
- Computes local triangulation
```
void computeGlobalTriangulationFromPoints(int numOfPoints, 
                                          float * points, 
                                          int &numOfIndices, 
                                          int ** indices, 
                                          float ** normals, 
                                          bool visualization = false)
```
- computes global triangulation

**4) SDF_lib** [Shape Diameter Function]
- Not yet needed, but I have left it there for keeping compatibility of source files between my repo and this repo.

###PREPROCESSOR DEFINITIONS:###
```
D_SCL_SECURE_NO_WARNINGS;_NLOG;_NMMGR;NOMINMAX;
```
###HOW IT WORKS:###

When a process wants to extract a skeleton from an input mesh, the process has to call the `"computeSkeleton"` method from `LBSE_lib`. Next, for extraction of skinning weights, the `GDSW_lib` has to be used.

###LIBRARIES NEEDED TO RUN EXTRACTION:###

- Boost - used for serialization of the skeletons, can be removed later if serialization is not needed
- TNT - vectors and matrices used in Jama
- Jama - linear solver for TNT vectors and matrices
- ViennaCL - linear solver with OpenCL support...the solver can be parallelized on the GPU
- Fade2D - used for Delaunay triangulation
