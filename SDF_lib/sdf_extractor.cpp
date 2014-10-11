//---------------------------------------------------------------------------
#include "sdf_extractor.h"

void sdf::Extractor::applyMedianMerge(MeshGraph * polyMesh, MeshGraph * pMesh, float threshold){

	vector<CVector3> polyMeshVerts;
	map<int, set<int> > polyMeshIndices;

	for (int i=0; i < pMesh->numOfVertices; i++){

		std::vector<int> neighs;
		// find lokal neighbors of vertex

		for (int j=0; j < pMesh->numOfVertices; j++){

			float dist = Distance(pMesh->pVerts[i], pMesh->pVerts[j]);

			if (dist < threshold){
				neighs.push_back(j);
			}
		}

		int k = neighs.size();

		// calculate median

		CVector3 median(0,0,0);

		for (int j=0; j<neighs.size(); j++){
			median = median + pMesh->pVerts[neighs[j]];
		}

		median = median / neighs.size();

		// move all vertices in neigh into median

		for (int j=0; j<neighs.size(); j++){
			pMesh->pVerts[neighs[j]] = median;
		}

	}

	/*for (int j=0; j < polyMeshVerts.size(); j++){
		vector<int> indices;
		polyMeshIndices.push_back(indices);
	}*/


	//polyMeshIndices.resize(polyMeshVerts.size());

/*#ifdef _LOG
	logg.log(0, "Matica E pMesh");
	logg.log(0, pMesh->E);
#endif*/

	for (int i=0; i < pMesh->numOfVertices; i++){
		bool add = true;
		for (int j=0; j < polyMeshVerts.size(); j++){
			if (Distance(pMesh->pVerts[i], polyMeshVerts[j]) < 0.00001){
				add = false;
				polyMeshIndices[j].insert(i);
			}
		}
		if (add){
			polyMeshVerts.push_back(pMesh->pVerts[i]);
			polyMeshIndices[polyMeshVerts.size() - 1].insert(i);
		}
	}

	polyMesh->numOfVertices = polyMeshVerts.size();
	polyMesh->pVerts = new CVector3[polyMesh->numOfVertices];
	for (int i=0; i<polyMesh->numOfVertices; i++){
		polyMesh->pVerts[i] = polyMeshVerts[i];
	}

	polyMesh->E = Array2D< bool >(polyMesh->numOfVertices, polyMesh->numOfVertices, false);
	polyMesh->numOfVBOlines = 0;

	for (int i=0; i<polyMesh->numOfVertices; i++){
		vector<int> indicesi;
		indicesi.assign( polyMeshIndices[i].begin(), polyMeshIndices[i].end() );

		/*#ifdef _LOG
				logg.log(0, "vector indexov z polyMesh");
				logg.log(0, "i: ", i);
				logg.log(0, indicesi);
		#endif*/

		for (int j=0; j<polyMesh->numOfVertices; j++){
			vector<int> indicesj;
			indicesj.assign( polyMeshIndices[j].begin(), polyMeshIndices[j].end() );

			for (int ii=0; ii<indicesi.size(); ii++){
				for (int jj=0; jj<indicesj.size(); jj++){
					int iii = indicesi[ii];
					int jjj = indicesj[jj];

					bool edgeee = pMesh->E[iii][jjj];

					if (edgeee){
						polyMesh->E[i][j] = true;
						polyMesh->numOfVBOlines++;
					}
				}
			}
		}
	}

	polyMesh->triangleIndices = new int[0];
	polyMesh->numOfFaces = 0;
}