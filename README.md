1/ Data from the simulation are loaded, for this there are 2 objects - cuticleLoader and MuscleLoader … for cuticle the cuticleLoader, position file and membrane file has to be set. Position file is taken from OW_DATA_VERSION data dir and membrane file is taken from OW_BINDPOSE_DATA_VERSION data dir.

g_core->openwormWrapper.cuticleLoader.ReadWormIteration(g_core->openwormWrapper.mesh, timestep);
Import::CalculatePerVertexNormals(g_core->openwormWrapper.mesh);

2/ All the cuticles in all the timesteps are loaded in a loop and exported into collada files (in order to be loaded by assimp, so SDF can be calculated). 

g_core->openwormWrapper.exporter.Export( g_core->openwormWrapper.mesh, config.projectDir + "Models\\OpenWorm\\Export\\export_"+OW_DATA_VERSION+"\\collada\\wormCollada_" + g_core->exportWrapper.PadNumber( g_core->openwormWrapper.timeStep, OW_PADDING) + ".dae");

3/ Next, wrapper is called from the script. It is loaded from the script in a loop, because there is a memory leak in SDF library. In can be run in 8 threads using 8 powershell scripts. By running this wrapper, skeletons and transformations between skeletons and a bindpose are calculated.
The output are quaternions, matrices and axis angles. They are rotations only encoding in the matrices.

g_core->openwormWrapper.skeletonExtractionSDF_calculateSDF(&(g_engine->oModel), &(g_core->lbseWrapper.oLBSExtractor));
g_core->openwormWrapper.skeletonExtractionSDF_extractSkeleton(g_core->defaultSkeletonRoot, &(g_core->lbseWrapper.oLBSExtractor), &(g_core->sdfWrapper.oSDFExtractor));			 

4) Postprocessing of the rotations. There are three steps in the postprocessing process.

a - interpolation is applied, SLERP is used for this
(this is performed to get rid of the shivering of the worm caused by floating inaccuracy)
SM::SmoothQuaternions(source, 130);  // 100-150 are good smoothing neighbourhoods

b - SDF is calculated, muscle stretching and gravitation force is applied and composed into the matrices
(muscle are stretching the cuticle, therefore the volume has to be restored)

c - rotations are perform in the skeleton chain, the translation of the skeleton root is ignored
(in each node, there is transformation for local node only and by multiplying of transformation matrices in skeleton chain we get global transformation matrix for each node, that can be directly used for skinning in shaders)
g_core->openwormWrapper.performSDFBasedTransformationsInSkeletonTree(&g_core->openwormWrapper.bindPoseSkeletonNode, &source[i], &matrices, sdf_loaded, 9.81, glm::vec3(0, 0, 1)); // b and c are both called in this method

d - the skeleton is anchored in some skeleton node
(for each timestep, the translation vector for anchored skeleton node is calculated - translation between bindpose position and the skinned one. Then this translation vector is added to each transformation matrix, do the skinning is applied and final skinned vertex is translated at the end by the vector)
g_core->openwormWrapper.anchorSkeletonDuringSkinning(&g_core->openwormWrapper.bindPoseSkeletonNode, &matrices, 15); // last parameter is the index of the node

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
###HOW DOES IT WORK:###

When a process wants to extract a skeleton from an input mesh, the process has to call the `"computeSkeleton"` method from `LBSE_lib`. Next, for extraction of skinning weights, the `GDSW_lib` has to be used.

###LIBRARIES NEEDED TO RUN EXTRACTION:###

- Boost - used for serialization of the skeletons, can be removed later if serialization is not needed
- TNT - vectors and matrices used in Jama
- Jama - linear solver for TNT vectors and matrices
- ViennaCL - linear solver with OpenCL support...the solver can be parallelized on the GPU
- Fade2D - used for Delaunay triangulation


###HOW TO POSTPROCESS QUATERNIONS, SMOOTH THEM AND ANCHOR THEM:###
```
 int timeStep = OW_START_WORM_ITERATION, timeIncrease = 1;
 //load skeleton
 string file = config.projectDir + "Models\\OpenWorm\\Export\\export_" + OW_DATA_VERSION + "\\animation_" + std::to_string(OW_SKINNING_NUM_BONES) + "\\skeleton_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_"+ g_core->exportWrapper.PadNumber(OW_BINDPOSE_TIMESTEP, 4) + ".skl";
 ifstream ifs(file);
 assert(ifs.good());
 boost::archive::xml_iarchive ia(ifs);
 ia >> BOOST_SERIALIZATION_NVP(g_core->openwormWrapper.bindPoseSkeletonNode);
 assignFathersForSkeletonTree(&g_core->openwormWrapper.bindPoseSkeletonNode);
 //load sdf 
 string sdf_file_31_0 = config.projectDir + "Models\\OpenWorm\\Export\\export_" + OW_DATA_VERSION + "\\animation_"+ std::to_string(OW_SKINNING_NUM_BONES)+"\\SDF__31S_00000.sdf";
 vector<float> sdf_loaded;
 g_core->openwormWrapper.loadSDFFromFile(sdf_file_31_0, sdf_loaded);
 //load quaternions
 vector<vector<glm::quat> > source;
 source.reserve(OW_MAX_WORM_ITERATIONS);
 while (timeStep < OW_MAX_WORM_ITERATIONS) {
	 //load skinning data
	 string quaternionFile = config.projectDir + "Models\\OpenWorm\\Export\\export_" + OW_DATA_VERSION + "\\animation_"+ std::to_string(OW_SKINNING_NUM_BONES)+"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) +"\\quaternion_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + g_core->exportWrapper.PadNumber(timeStep, 4) + ".qua";
	 ifstream isq(quaternionFile);
	 assert(isq.good());
	 source.push_back(vector<glm::quat>());
	 g_core->openwormWrapper.ImportQuaternion(source.back(), isq);

	 timeStep += timeIncrease;
 }				 
 //postprocess quaternions, smoth them with SLERP radius
 SM::SmoothQuaternions(source, 130);
 //output quaternions
 timeStep = OW_START_WORM_ITERATION;
 for (int i = 0; i < source.size(); i++) {
	 //save quaternions to file
	 string quaternionFile = config.projectDir + "Models\\OpenWorm\\Export\\export_" + OW_DATA_VERSION + "\\animation_"+ std::to_string(OW_SKINNING_NUM_BONES)+"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) +"\\postprocessed\\quaternion_post_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + g_core->exportWrapper.PadNumber(timeStep, 4) + ".qua";
	 ofstream outputQ(quaternionFile);

	 for (int q = 0; q < source[i].size(); q++) {
		 glm::quat quat = source[i][q];
		 outputQ << quat.w << " " << quat.x << " " << quat.y << " " << quat.z << std::endl;
	 }
	 //save matrix to file
	 string matrixFile = config.projectDir + "Models\\OpenWorm\\Export\\export_" + OW_DATA_VERSION + "\\animation_"+ std::to_string(OW_SKINNING_NUM_BONES)+"_bindpose"+ std::to_string(OW_BINDPOSE_TIMESTEP) +"\\anchored2\\matrix_anchored_" + std::to_string(OW_SKINNING_NUM_BONES) + "S_" + g_core->exportWrapper.PadNumber(timeStep, 4) + ".mat";
	 ofstream outputM(matrixFile);
	 vector<glm::mat4> matrices;
	 g_core->openwormWrapper.performSDFBasedTransformationsInSkeletonTree(&g_core->openwormWrapper.bindPoseSkeletonNode, &source[i], &matrices, sdf_loaded, 9.81, glm::vec3(0, 0, 1));
	 // anchor middle node of the skeleton, translate all the other nodes
	 g_core->openwormWrapper.anchorSkeletonDuringSkinning(&g_core->openwormWrapper.bindPoseSkeletonNode, &matrices, 22);

	 for (int m = 0; m < matrices.size(); m++) {
		 for (int j = 0; j < 4; j++) {
			 for (int k = 0; k < 4; k++) {
				 outputM << matrices[m][j][k] << " ";
			 }
			 outputM << std::endl;
		 }
		 outputM << std::endl;
	 }

	 timeStep += timeIncrease;
 }
```



