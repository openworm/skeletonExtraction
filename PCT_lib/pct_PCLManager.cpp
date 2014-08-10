//---------------------------------------------------------------------------
#include "pct_PCLManager.h"

#pragma unmanaged

/*int getKNeigh_orig(MeshGraph * pMesh){
	return min(max((int)(pMesh->numOfVertices * 0.02),8),12);
}*/

/*void greedyProjection(MeshGraph *in_mesh, MeshGraph *out_mesh){
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Generate pointcloud data
  cloud->width = in_mesh->numOfVertices;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
	  cloud->points[i].x = in_mesh->pVerts[i].x;
	  cloud->points[i].y = in_mesh->pVerts[i].y;
	  cloud->points[i].z = in_mesh->pVerts[i].z;
  }
  
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

  while (!viewer.wasStopped ())
  {
	  viewer.spinOnce ();
  }

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (300);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  
  pcl::io::saveOBJFile(config.projectDir+"\\Models\\obj\\export\\mesh.obj", triangles);

  int numOfPolygons = triangles.polygons.size();
  for (int i=0; i < numOfPolygons; i++){
	  pcl::Vertices polygon = triangles.polygons[i];
  }

  out_mesh->numOfVertices = cloud->width;
  //out_mesh->pVerts = new CVector3[cloud->width];
  out_mesh->E = Array2D<bool>(cloud->width, cloud->width, false);
//  out_mesh->faceAreaSum = -1;


  for (int i=0; i < numOfPolygons; i++){
	  pcl::Vertices polygon = triangles.polygons[i];
	  //out_mesh->pVerts[polygon.vertices[0]] = CVector3(cloud->points[polygon.vertices[0]].x, cloud->points[polygon.vertices[0]].y, cloud->points[polygon.vertices[0]].z);
	  //out_mesh->pVerts[polygon.vertices[1]] = CVector3(cloud->points[polygon.vertices[1]].x, cloud->points[polygon.vertices[1]].y, cloud->points[polygon.vertices[1]].z);
	  //out_mesh->pVerts[polygon.vertices[2]] = CVector3(cloud->points[polygon.vertices[2]].x, cloud->points[polygon.vertices[2]].y, cloud->points[polygon.vertices[2]].z);
	  out_mesh->E[polygon.vertices[0]][polygon.vertices[1]] = true;
	  out_mesh->E[polygon.vertices[1]][polygon.vertices[2]] = true;
	  out_mesh->E[polygon.vertices[2]][polygon.vertices[0]] = true;
  }
  
}*/

void getTangentPlanePCA_orig(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3){

	// get center of points
	CVector3 center(0,0,0);
	for (int i=0; i < numOfPoints; i++){
		center = center + points[i];
	}
	center = center / (float)numOfPoints;

	// create matrix C1 and C2
	Array2D<double> C1(3, numOfPoints, 0.0);
	for (int i=0; i < numOfPoints; i++){
		C1[0][i] = points[i].x - center.x;
		C1[1][i] = points[i].y - center.y;
		C1[2][i] = points[i].z - center.z;
	}

	Array2D<double> C2(numOfPoints, 3, 0.0);
	for (int i=0; i < numOfPoints; i++){
		C2[i][0] = points[i].x - center.x;
		C2[i][1] = points[i].y - center.y;
		C2[i][2] = points[i].z - center.z;
	}

	// compute covariance matrix C as C1 * C2
	Array2D<double> C = C1 * C2;

	/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. A is 3x3 symmetric matrix */
	double A[3][3];
	double V[3][3];
	double d[3];

	//Log::log(0, "matica C: ");
	//Log::log(0, C);

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			A[i][j] = C[i][j] / (double)(numOfPoints);

	eigen_decomposition(A, V, d);

	/*CVector3 ev0 = CVector3((float)V[0][0], (float)V[0][1], (float)V[0][2]);
	CVector3 ev1 = CVector3((float)V[1][0], (float)V[1][1], (float)V[1][2]);
	CVector3 ev2 = CVector3((float)V[2][0], (float)V[2][1], (float)V[2][2]);*/

	CVector3 ev0 = CVector3((float)V[0][0], (float)V[1][0], (float)V[2][0]);
	CVector3 ev1 = CVector3((float)V[0][1], (float)V[1][1], (float)V[2][1]);
	CVector3 ev2 = CVector3((float)V[0][2], (float)V[1][2], (float)V[2][2]);

	if (d[0] > d[1]){
		if (d[1] > d[2]){
			*ev_1 = ev0;
			*ev_2 = ev1;
			*ev_3 = ev2;
		} else {
			*ev_1 = ev0;
			*ev_2 = ev2;
			*ev_3 = ev1;
		}
	} else {
		if (d[0] > d[2]){
			*ev_1 = ev1;
			*ev_2 = ev0;
			*ev_3 = ev2;
		} else {
			*ev_1 = ev1;
			*ev_2 = ev2;
			*ev_3 = ev0;
		}
	}

	*ev_1 = Normalize(*ev_1);
	*ev_2 = Normalize(*ev_2);
	*ev_3 = Normalize(*ev_3);

	/**ev_1 = Normalize(CVector3(-ev_1->x, -ev_1->z, -ev_1->y));
	*ev_2 = Normalize(CVector3(-ev_2->x, -ev_2->z, -ev_2->y));
	*ev_3 = Normalize(CVector3(-ev_3->x, -ev_3->z, -ev_3->y));*/

	*n = Normalize(Cross(*ev_1, *ev_2));
}

void getTangentPlanePCAWithSVD_orig(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3){

	Array2D<float> P(numOfPoints, 3, 0.0);
	// get center of points
	CVector3 center(0,0,0);
	for (int i=0; i < numOfPoints; i++){
		center = center + points[i];
		P[i][0] = points[i].x;
		P[i][1] = points[i].y;
		P[i][2] = points[i].z;
	}
	center = center / (float)numOfPoints;

	// create matrix C1 and C2
	/*Array2D<double> C1(3, numOfPoints, 0.0);
	for (int i=0; i < numOfPoints; i++){
		C1[0][i] = points[i].x - center.x;
		C1[1][i] = points[i].y - center.y;
		C1[2][i] = points[i].z - center.z;
	}

	Array2D<double> C2(numOfPoints, 3, 0.0);
	for (int i=0; i < numOfPoints; i++){
		C2[i][0] = points[i].x - center.x;
		C2[i][1] = points[i].y - center.y;
		C2[i][2] = points[i].z - center.z;
	}

	// compute covariance matrix C as C1 * C2
	Array2D<double> C = C1 * C2;*/

	JAMA::SVD<float> svd(P);
	Array2D<float> U;
	svd.getU(U);
	Array2D<float> S;
	svd.getS(S);
	Array2D<float> C = U * S * S * U.transpose(U);

	/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. A is 3x3 symmetric matrix */
	double A[3][3];
	double V[3][3];
	double d[3];

	//Log::log(0, "matica C: ");
	//Log::log(0, C);

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			A[i][j] = C[i][j] / (double)(numOfPoints);

	eigen_decomposition(A, V, d);

	CVector3 ev0 = CVector3((float)V[0][0], (float)V[0][1], (float)V[0][2]);
	CVector3 ev1 = CVector3((float)V[1][0], (float)V[1][1], (float)V[1][2]);
	CVector3 ev2 = CVector3((float)V[2][0], (float)V[2][1], (float)V[2][2]);

	/*CVector3 ev0 = CVector3((float)V[0][0], (float)V[1][0], (float)V[2][0]);
	CVector3 ev1 = CVector3((float)V[0][1], (float)V[1][1], (float)V[2][1]);
	CVector3 ev2 = CVector3((float)V[0][2], (float)V[1][2], (float)V[2][2]);*/

	if (d[0] > d[1]){
		if (d[1] > d[2]){
			*ev_1 = ev0;
			*ev_2 = ev1;
			*ev_3 = ev2;
		} else {
			*ev_1 = ev0;
			*ev_2 = ev2;
			*ev_3 = ev1;
		}
	} else {
		if (d[0] > d[2]){
			*ev_1 = ev1;
			*ev_2 = ev0;
			*ev_3 = ev2;
		} else {
			*ev_1 = ev1;
			*ev_2 = ev2;
			*ev_3 = ev0;
		}
	}

	*ev_1 = Normalize(*ev_1);
	*ev_2 = Normalize(*ev_2);
	*ev_3 = Normalize(*ev_3);

	*n = Normalize(Cross(*ev_1, *ev_2));
}

/*void getTangentPlanePCAUsingPCL(int numOfPoints, CVector3 * points, CVector3 * n, CVector3 * ev_1, CVector3 * ev_2, CVector3 * ev_3){

		// PCA computed using PCL

		// create point cloud using neighs
		pcl::PointCloud<pcl::PointXYZ>::Ptr neighcloudPtr( new pcl::PointCloud<pcl::PointXYZ>() );
		neighcloudPtr->width = numOfPoints;
		neighcloudPtr->height = 1;
		neighcloudPtr->is_dense = true;
		neighcloudPtr->resize(numOfPoints);


		for (int i=0; i < numOfPoints; i++){
			pcl::PointXYZ point;
			point.x = points[i].x;
			point.y = points[i].y;
			point.z = points[i].z;
			neighcloudPtr->push_back(point);
		}
		pcl::PCA<pcl::PointXYZ> pca;

		pca.setInputCloud(neighcloudPtr);

		//Eigen::Vector4f pca_mean = pca.getMean();
		Eigen::Vector3f pca_val = pca.getEigenValues();
		Eigen::Matrix3f pca_vec = pca.getEigenVectors();
		
		if (pca_val(0) > pca_val(1)){
			if (pca_val(1) > pca_val(2)){
				// 0 a 1
				*ev_1 = CVector3(pca_vec(0,0), pca_vec(0,1), pca_vec(0,2));
				*ev_2 = CVector3(pca_vec(1,0), pca_vec(1,1), pca_vec(1,2));
				*ev_3 = CVector3(pca_vec(2,0), pca_vec(2,1), pca_vec(2,2));
			} else {
				// 0 a 2
				*ev_1 = CVector3(pca_vec(0,0), pca_vec(0,1), pca_vec(0,2));
				*ev_2 = CVector3(pca_vec(2,0), pca_vec(2,1), pca_vec(2,2));
				*ev_3 = CVector3(pca_vec(1,0), pca_vec(1,1), pca_vec(1,2));
			}
		} else {
			if (pca_val(0) > pca_val(2)){
				//1 a 0 
				*ev_1 = CVector3(pca_vec(1,0), pca_vec(1,1), pca_vec(1,2));
				*ev_2 = CVector3(pca_vec(0,0), pca_vec(0,1), pca_vec(0,2));
				*ev_3 = CVector3(pca_vec(2,0), pca_vec(2,1), pca_vec(2,2));
			} else {
				//1 a 2
				*ev_1 = CVector3(pca_vec(1,0), pca_vec(1,1), pca_vec(1,2));
				*ev_2 = CVector3(pca_vec(2,0), pca_vec(2,1), pca_vec(2,2));
				*ev_3 = CVector3(pca_vec(0,0), pca_vec(0,1), pca_vec(0,2));
			}
		}
		
		*ev_1 = Normalize(*ev_1);
		*ev_2 = Normalize(*ev_2);
		*ev_3 = Normalize(*ev_3);

		*n = Normalize(Cross(*ev_1, *ev_2));
}*/

/*Array2D<bool> computeLocalTrianglation_orig(int i, MeshGraph * pMesh, vector<std::set<int>> globalNeighbourhoods){
		/////////////// params /////////////
		bool doProjectionToLocalPlane = false;
		int k = 0;
		//float radiusNeigh = (pModel->modelbb.x_max -pModel->modelbb.x_min + pModel->modelbb.y_max + pModel->modelbb.y_min + pModel->modelbb.z_max - pModel->modelbb.z_min) / 10.0;

		/////////////// neighbourhood ///////////////

		std::set<int> neighs = globalNeighbourhoods[i];

//		CVector3 normalAva = CVector3(0,0,0);
//		for (std::set<int>::iterator it1=neighs.begin(); it1!=neighs.end(); ++it1){
//			for (std::set<int>::iterator it2=neighs.begin(); it2!=neighs.end(); ++it2){
//				CVector3 v1 = pMesh->pVerts[i] - pMesh->pVerts[*it1];
//				CVector3 v2 = pMesh->pVerts[i] - pMesh->pVerts[*it2];
//				CVector3 nor = Cross(v1, v2);
//
//				normalAva = normalAva + nor;
//			}
//		}
//
//		normalAva = normalAva / (float)neighs.size();
		

		// vyratame normalu tak, ze spravime vector (P - Pi) medzi vrcholom a kazdym vrcholom okolia
		// a potom cross medzi nimi a z toho priemerny vektor...

	
		//The projection of a point q = (x, y, z) onto a plane given by a point p = (a, b, c) and a normal n = (d, e, f) is
		//q_proj = q - dot(q - p, n) * n

		CVector2 * pointsInPlane = new CVector2[neighs.size()];
		CVector3 * neighPoints = new CVector3[neighs.size()];

//		cloud_neighs_projected->width    = neighs.size();
//		cloud_neighs_projected->height   = 1;
//		cloud_neighs_projected->is_dense = false;
//		cloud_neighs_projected->points.resize (cloud_neighs_projected->width * cloud_neighs_projected->height);

		// insert the point with index i - origin into neighs !!!
		neighs.insert(i);

		CVector3 p = pMesh->pVerts[i]; // origin
		CVector3 n;
		CVector3 ev1;
		CVector3 ev2;
		CVector3 ev3;

		Log::log(0, "okolie: ");

		k = 0;
		for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it){
			neighPoints[k] = pMesh->pVerts[*it];
			Log::log(0, "neighs: ", pMesh->pVerts[*it]);
			k++;
		}

		/////////////// PCA ///////////////

		getTangentPlanePCA_orig(neighs.size(), neighPoints, &n, &ev1, &ev2, &ev3);
		//getTangentPlanePCAWithSVD(neighs.size(), neighPoints, &n, &ev1, &ev2, &ev3);
		//getTangentPlanePCAUsingPCL(neighs.size(), neighPoints, &n, &ev1, &ev2, &ev3);


//		CVector3 p2 = p;
//		k = 0
//		while (Distance(p2, p) < 0.00001){
//			p2 = pMesh->pVerts[k];
//			k++;
//		}

//		CVector3 p2_proj = ProjectPointIntoPlane(p2, p, n);

		//Log::log(0, "normala tang. plochy: ", n);

		/////////////// PROJECTION INTO TANGENT PLANE ///////////////

		k=0;
		for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it)
		{
			CVector3 q(pMesh->pVerts[*it].x, pMesh->pVerts[*it].y, pMesh->pVerts[*it].z); // ten co chcem projektovat

			CVector3 q_proj = ProjectPointIntoPlane_orig(q, p, n);

//			if (i == 0){
//   				pMesh->pVerts[*it] = q_proj;
//			}

			CVector2 q_projPlane = Space2Plane_orig(q_proj, p, ev1, ev2);
			pointsInPlane[k] = q_projPlane;

//			if (i == 0){
//				pMesh->pVerts[*it] = Plane2Space(q_projPlane, p, ev1, ev2);
//			}
		
			k++;
		}

		// !!! ratat Laplacian v 3D, triangulacia sa spravi v 2D ale potom sa vrcholy adekvatne spoja v 3D
		// deleunay triangulation in a plane using projected point cloud of neighs

		/////////////// DELEUNAY IN 2D ///////////////

		using namespace GEOM_FADE2D;

		int numPoints = neighs.size();
		std::vector<Point2> vInputPoints;

		Fade_2D dt(numPoints);

		k = 0;
		  for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it){
			  Point2 point;
			  point.change((double)pointsInPlane[k].x, (double)pointsInPlane[k].y);
			  vInputPoints.push_back(point);
			  k++;
		  }
		  
	  dt.insert(vInputPoints);
	  
	  std::vector<Triangle2*> vAllDelaunayTriangles;
	  dt.getTrianglePointers(vAllDelaunayTriangles);


		 // create one ring area from neighIndex of processed vertex

		Array2D<bool> E_local = Array2D<bool>(pMesh->numOfVertices, pMesh->numOfVertices, false);

		for(std::vector<Triangle2*>::iterator it=vAllDelaunayTriangles.begin();it!=vAllDelaunayTriangles.end();++it){
		   Triangle2* t(*it);
		   Point2* p0=t->getCorner(0);
		   Point2* p1=t->getCorner(1);
		   Point2* p2=t->getCorner(2);

		   int i0 = -1;
		   int i1 = -1;
		   int i2 = -1;

		   k = 0;
		   for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it){
				if (Distance(pointsInPlane[k], CVector2(p0->x(),p0->y())) < 0.00001){
					i0 = *it;
				}
				if (Distance(pointsInPlane[k], CVector2(p1->x(),p1->y())) < 0.00001){
					i1 = *it;
				}
				if (Distance(pointsInPlane[k], CVector2(p2->x(),p2->y())) < 0.00001){
					i2 = *it;
				}
				k++;
		   }

		   if (i0 != -1 && i1 != -1 && i2 != -1){
			
			  E_local[i0][i1] = true;
			  E_local[i0][i2] = true;
			  E_local[i1][i0] = true;
			  E_local[i1][i2] = true;
			  E_local[i2][i0] = true;
			  E_local[i2][i1] = true;
		   }
			
//		   if (i0 != -1 && i1 != -1){
//			  E_local[i0][i1] = true;
//			  E_local[i1][i0] = true;
//		   }
//		   if (i0 != -1 && i2 != -1){
//			  E_local[i0][i2] = true;
//			  E_local[i2][i0] = true;
//		   }
//		   if (i2 != -1 && i1 != -1){
//			  E_local[i1][i2] = true;
//			  E_local[i2][i1] = true;
//		   }
			
		}

		//if (i == pModel->neighPCIndex){
			/*pMesh->neighVis[i].E_local_visualize = E_local.copy();
			pMesh->neighVis[i].visNormals = new CVector3[4];
			pMesh->neighVis[i].visNormals[0] = n;
			pMesh->neighVis[i].visNormals[1] = ev1;
			pMesh->neighVis[i].visNormals[2] = ev2;
			pMesh->neighVis[i].visNormals[3] = ev3;

			pMesh->neighVis[i].localNeighs = neighs;
			pMesh->neighVis[i].pointsInTangentPlane = pointsInPlane;
			pMesh->neighVis[i].isE_local_visualize = true;*/
		//}


		//Log::log(0, E_local);
		//return E_local;
//}
/*
CVector3 ProjectPointIntoPlane_orig(CVector3 q, CVector3 o, CVector3 n){
	CVector3 q_proj = q - n * Dot(q - o, n); // ten dot je distance
	return q_proj;
}

CVector2 Space2Plane_orig(CVector3 q, CVector3 o, CVector3 e1, CVector3 e2){

	Array2D< float > M = Array2D< float >(2, 3, 0.0f);
	M[0][0] = e1.x; M[1][0] = e2.x; 
	M[0][1] = e1.y; M[1][1] = e2.y;
	M[0][2] = e1.z; M[1][2] = e2.z;

	Array2D< float > S = Array2D< float >(3, 1, 0.0f);
	S[0][0] = q.x - o.x; S[0][1] = q.y - o.y; S[0][2] = q.z - o.z;

	Array2D< float > P = Array2D< float >(1, 2, 0.0f);

	P = M * S;

	CVector2 pp;
	pp.x = P[0][0];
	pp.y = P[0][1];

	return pp;
}

CVector3 Plane2Space_orig(CVector2 q, CVector3 o, CVector3 e1, CVector3 e2){

	Array2D< float > M = Array2D< float >(2, 3, 0.0f);
	M[0][0] = e1.x; M[1][0] = e2.x; 
	M[0][1] = e1.y; M[1][1] = e2.y;
	M[0][2] = e1.z; M[1][2] = e2.z;

	Array2D< float > S = Array2D< float >(3, 1, 0.0f);

	Array2D< float > P = Array2D< float >(1, 2, 0.0f);
	P[0][0] = q.x; P[0][1] = q.y;

	S = P * M;

	CVector3 ss;
	ss.x = S[0][0];
	ss.y = S[0][1];
	ss.z = S[0][2];

	return o + ss;
}
*/

/*
void findClosestNeighWithCentering_orig(int i, MeshGraph * pMesh, std::set<int> &neighs, CVector3 &cm, int * distances){
	float minDistance = FLT_MAX;
	int minIndex = -1;
	// factor, which penalyzes points near center of mass of the neighs
	float centerFactor = 1.0;
	for (int j=0; j<pMesh->numOfVertices; j++){
		if (i != j){


			CVector3 Vector1 = cm - pMesh->pVerts[i];
			CVector3 Vector2 = cm - pMesh->pVerts[j];

			float dotProduct = Dot(Vector1, Vector2);

			float vectorsMagnitude = Magnitude(Vector1) * Magnitude(Vector2) ;

			float v = dotProduct / vectorsMagnitude;
			if (v > 1.0)
				v = 1.0;
			if (v < -1.0)
				v = -1.0;

			centerFactor = (-v / 2.0) + 0.5;
			// if new vertex is oriented near cm, centerFactor is close to 0.1...if the vertex is on the other side, center Factor is close to 1.1
			//OrientedAngleBetweenVectors(cm - pMesh->pVerts[i], cm - pMesh->pVerts[j]);
			if (centerFactor == 0 && neighs.size() == 0 || vectorsMagnitude == 0) // if there is only one vertex, so cm = pVerts[i] and centerFactor = 0.0 ... we should change it to 1.0
				centerFactor = 1.0;

			if (centerFactor < 0.2 && centerFactor >= 0)
				centerFactor = 0.2;

			if (centerFactor > -0.2 && centerFactor < 0)
				centerFactor = -0.2;

			if (distances[j] * centerFactor < minDistance && (neighs.find(j) == neighs.end())){
				minDistance = distances[j] * centerFactor;
				minIndex = j;
			}
		}
	}
	neighs.insert(minIndex);
	// compute center of mass of neighbourhood
	cm = CVector3(0,0,0);
	for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it){
		cm = cm + pMesh->pVerts[*it];
	}
	cm = cm / neighs.size();
}*/



/*
// return true if we have to add next neigh vertex
bool checkLocalNeighAngles_orig(int i, MeshGraph * pMesh, std::set<int> &neighs){
	vector<int> vecNeighs;
	for (std::set<int>::iterator it=neighs.begin(); it!=neighs.end(); ++it){
		vecNeighs.push_back(*it);
	}
	// angles in one ring area from vertex i to others
	vector<float> angles = vector<float>(neighs.size());
	//vector<int> indices_index2order = vector<int>(neighs.size());
	vector<int> indices_order2index = vector<int>(neighs.size());
	int swapIndex;
	for (int j=0; j < neighs.size(); j++){
		float angle = OrientedAngleBetweenVectors(pMesh->pVerts[i] - pMesh->pVerts[vecNeighs[0]], pMesh->pVerts[i] - pMesh->pVerts[vecNeighs[j]]);
		angles[j] = angle;
		indices_order2index[j] = j;
	}

	// sort angles, get indices
	for (int j=0; j<neighs.size(); j++){
		float minAngle = FLT_MAX;
		for (int k=j; k<neighs.size(); k++){
			if (angles[k] < minAngle){
				minAngle = angles[k];
				swapIndex = k;
			}
		}
		float tmp = angles[j];
		angles[j] = angles[swapIndex];
		angles[swapIndex] = tmp;
		int tmp2 = indices_order2index[j];
		indices_order2index[j] = indices_order2index[swapIndex];
		indices_order2index[swapIndex] = tmp2;
	}

	// check if there is angle of 2 consequent vectors > 90
	for (int j=0; j<neighs.size(); j++){
		int idx = indices_order2index[j];
		int nextIdx = indices_order2index[(j + 1) % neighs.size()];
		//CVector3 vec0 = pMesh->pVerts[i] - pMesh->pVerts[vecNeighs[0]];
		CVector3 vec1 = pMesh->pVerts[i] - pMesh->pVerts[vecNeighs[idx]];
		CVector3 vec2 = pMesh->pVerts[i] - pMesh->pVerts[vecNeighs[nextIdx]];
		//float angle01 = AngleBetweenVectors(vec0, vec1);
		//float angle02 = AngleBetweenVectors(vec0, vec2);
		float angle = OrientedAngleBetweenVectors(vec1, vec2);

		angles[j] = angle;
		if (angles[j] > PI / 2)
			return true;
	}

	return false;
}*/
/*
vector<std::set<int>> computeGlobalNeighbourhood_orig(MeshGraph * pMesh){

	vector<std::set<int>> globalNeighbourhoods;

	int kneigh = getKNeigh_orig(pMesh);

	for (int i=0; i < pMesh->numOfVertices; i++){

		std::set<int> neighs;
		int * distances = new int[pMesh->numOfVertices];
		for (int j=0; j<pMesh->numOfVertices; j++){
			distances[j] = -1;
		}

		int neighIndex = -1;

		for (int j=0; j<pMesh->numOfVertices; j++){
			distances[j] = Distance(pMesh->pVerts[i], pMesh->pVerts[j]);
		}

		CVector3 cm = pMesh->pVerts[i];
		// find kneigh nearest neighbours
		for (int k=0; k < kneigh; k++){
			findClosestNeighWithCentering_orig(i, pMesh, neighs, cm, distances);
		}

		// if angles between 2 consequent neighs is more than 90degree, maximum add 5
		int numAngleLimit = 0;
		while (checkLocalNeighAngles_orig(i, pMesh, neighs) && numAngleLimit < 5){
			findClosestNeighWithCentering_orig(i, pMesh, neighs, cm, distances);
			numAngleLimit++;
		}

		//int * neighbours = new int[kneigh];
		//findNearestKNeighbours(pMesh, i, kneigh, neighbours);
		globalNeighbourhoods.push_back(neighs);
	}

	return globalNeighbourhoods;
}*/

void computeTriangulationFromLocalDelaunays(MeshGraph * pMesh, PointCloudTriangulation::DeleunayTriangulator * pTriangulator){

	int numOfTrians;
	int * indices = NULL;
	float * normals = NULL;
	float * verts = new float[pMesh->numOfVertices * 3];
	for (int i=0; i < pMesh->numOfVertices; i++){
		verts[i * 3] = pMesh->pVerts[i].x;
		verts[i * 3 + 1] = pMesh->pVerts[i].y;
		verts[i * 3 + 2] = pMesh->pVerts[i].z;
	}

	int nov = pMesh->numOfVertices;
	pTriangulator->computeGlobalTriangulationFromPoints(pMesh->numOfVertices, verts, numOfTrians, &indices, &normals, true);

	vector<int> testind;

	for (int j=0; j<numOfTrians; j++){
		testind.push_back(indices[j * 3]);
		testind.push_back(indices[j * 3 + 1]);
		testind.push_back(indices[j * 3 + 2]);
	}

	Array2D<bool> E_global = Array2D<bool>(pMesh->numOfVertices, pMesh->numOfVertices, false);

	for (int j=0; j<numOfTrians; j++){
		E_global[indices[j * 3]][indices[j * 3 + 1]] = true;
		E_global[indices[j * 3 + 1]][indices[j * 3]] = true;
		E_global[indices[j * 3 + 1]][indices[j * 3 + 2]] = true;
		E_global[indices[j * 3 + 2]][indices[j * 3 + 1]] = true;
		E_global[indices[j * 3]][indices[j * 3 + 2]] = true;
		E_global[indices[j * 3 + 2]][indices[j * 3]] = true;
	}

	pMesh->E = E_global.copy();

}

#pragma managed

#pragma package(smart_init)
