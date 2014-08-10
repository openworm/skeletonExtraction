// Model.cpp : subor pre kontrolu modelov
#include "stdafx.h"
#include "Model.h"
#include "PointCloudTriangulation.h"

namespace ModelController
{
	CModel::CModel()
	{
		Assimp = new CAssimp();
		SDF_control = NULL;
		m_root = NULL;
		triangles = new LinkedList<Face>();
		points = new LinkedList<Vertex>();
		ResetSettings();
	}

	CModel::~CModel()
	{
		delete Assimp;
		delete SDF_control;

		/*
		/// -------------------------------------
		/// STRASNE DLHO TRVA ZATVORENIE PROGRAMU
		/// DOCASNE ZAKOMENTOVANE
		/// -------------------------------------

		// bacha na deletovanie v spravnom poradi
		if(m_root != NULL)
			delete m_root;

		// delete actual faces and vertices
		triangles->CompleteDelete();
		delete triangles;

		points->CompleteDelete();
		delete points;*/
	}

	// zapis info log do suboru
	void CModel::logInfo(std::string logString)
	{
		Assimp->logInfo(logString);
	}

	// zapis debug log do suboru
	void CModel::logDebug(std::string logString)
	{
		Assimp->logDebug(logString);
	}

	// nacita subor
	void CModel::LoadFile(std::string Filename)
	{
		ResetSettings();

		// ak znovu nacitavame, premaz povodne udaje
		if(m_root != NULL)
			delete m_root;

		if(SDF_control != NULL)
			delete SDF_control;

		// delete actual faces and vertices
		triangles->CompleteDelete();
		delete triangles;

		points->CompleteDelete();
		delete points;

		m_root = NULL;
		SDF_control = NULL;
		triangles = new LinkedList<Face>();
		points = new LinkedList<Vertex>();

		Assimp->Import3DFromFile(Filename);

		Assimp->LoadData(triangles, points);
		loaded = true;

		ComputeBoundary();
		CreateOctree();

		SetColors();
		ComputeSusedov();
	}

	// nacita priamo Assimp
	void CModel::LoadAssimp(aiScene* scene)
	{
		ResetSettings();

		// ak znovu nacitavame, premaz povodne udaje
		if(m_root != NULL)
			delete m_root;

		if(SDF_control != NULL)
			delete SDF_control;

		// delete actual faces and vertices
		triangles->CompleteDelete();
		delete triangles;

		points->CompleteDelete();
		delete points;

		m_root = NULL;
		SDF_control = NULL;
		triangles = new LinkedList<Face>();
		points = new LinkedList<Vertex>();

		Assimp->SetScene(scene);
		Assimp->LoadData(triangles, points);
		loaded = true;

		ComputeBoundary();
		CreateOctree();
		SetColors();
		ComputeSusedov();
	}

	/*float CModel::GetSDF(const struct aiFace* face, bool smoothed)
	{
		LinkedList<Face>::Cell<Face>* tmp = triangles->start;
		while(tmp != NULL)
		{
			if(tmp->data->assimp_ref == face)
			{
				if(smoothed)
					return tmp->data->diameter->smoothed;
				else
					return tmp->data->diameter->value;
			}
			tmp = tmp->next;
		}
		return 0;
	}*/

	float* CModel::GetSDF(int& size, bool smoothed)
	{
		LinkedList<Vertex>::Cell<Vertex>* tmp = points->start;
		size = points->GetSize();
		float* values = new float[size];
		for(int i = 0; i < size; i++)
		{
			float hodnota = 0;
			float total = float(tmp->data->susedia->GetSize());
			LinkedList<void>::Cell<void>* tm = tmp->data->susedia->start;
			while(tm != NULL)
			{
				if(smoothed)
					hodnota += ((Face*)tm->data)->diameter->smoothed;
				else
					hodnota += ((Face*)tm->data)->diameter->value;
				tm = tm->next;
			}
			hodnota = hodnota / total;
			values[i] = hodnota;
			tmp = tmp->next;
		}
		return values;
	}

	float* CModel::GetNormals(int& size)
	{
		LinkedList<Vertex>::Cell<Vertex>* tmp = points->start;
		size = points->GetSize();
		float* values = new float[size * 3];
		for(int i = 0; i < size; i++)
		{
			Vector4 hodnota = Vector4(0.0f, 0.0f, 0.0f, 0.0f);
			LinkedList<void>::Cell<void>* tm = tmp->data->susedia->start;
			while(tm != NULL)
			{
				hodnota = hodnota + ((Face*)tm->data)->normal;
				tm = tm->next;
			}
			hodnota.Normalize();
			values[i * 3 + 0] = hodnota.X;
			values[i * 3 + 1] = hodnota.Y;
			values[i * 3 + 2] = hodnota.Z;
			tmp = tmp->next;
		}
		return values;
	}

	void CModel::SetNewPositions(float* pos)
	{
		LinkedList<Vertex>::Cell<Vertex>* tmp = points->start;
		unsigned int size = points->GetSize();
		for(unsigned int i = 0; i < size; i++)
		{
			std::stringstream ss;
			ss << "Povodna suradnica vertexu c. ";
			ss << i;
			ss << " je X: ";
			ss << tmp->data->P.X;
			ss << ", Y: ";
			ss << tmp->data->P.Y;
			ss << ", Z: ";
			ss << tmp->data->P.Z;
			ss << "\n";
			ss << "Nova suradnica vertexu c. ";
			ss << i;
			ss << " je X: ";
			ss << pos[(i * 3) + 0];
			ss << ", Y: ";
			ss << pos[(i * 3) + 1];
			ss << ", Z: ";
			ss << pos[(i * 3) + 2];
			logInfo(ss.str());
			tmp->data->P.X = pos[(i * 3) + 0];
			tmp->data->P.Y = pos[(i * 3) + 1];
			tmp->data->P.Z = pos[(i * 3) + 2];
			tmp = tmp->next;
		}
		RecomputeNormals();

		if(m_root != NULL)
			delete m_root;

		m_root = NULL;
		CreateOctree();
	}

	void CModel::RecomputeNormals()
	{
		LinkedList<Face>::Cell<Face>* tmp = triangles->start;
		while(tmp != NULL)
		{
			tmp->data->ComputeNormal();
			tmp->data->diameter->value = 0.0f;
			tmp->data->diameter->smoothed = 0.0f;
			tmp->data->diameter->normalized1 = 0.0f;
			tmp->data->diameter->normalized2 = 0.0f;
			tmp = tmp->next;
		}

	}

	void CModel::DeleteIdenticalVertices()
	{
		float delta = b_size * 2 * 0.00001f;
		LinkedList<Vertex>* new_vertices = new LinkedList<Vertex>();
		LinkedList<Vertex>::Cell<Vertex>* tmp1 = points->start;
		while(tmp1 != NULL)
		{
			bool add = true;
			LinkedList<Vertex>::Cell<Vertex>* tmp2 = points->start;
			while(tmp2 != NULL)
			{
				if(tmp1 == tmp2)
					break;
				float distance = tmp1->data->P.Dist(tmp2->data->P);
				if(distance < delta)
				{
					add = false;
					break;
				}	
				tmp2 = tmp2->next;
			}
			if(add == true)
				new_vertices->InsertToEnd(tmp1->data);
			tmp1 = tmp1->next;
		}
		delete points;
		points = new_vertices;
	}

	void CModel::Triangulate ()
	{
		if(m_root != NULL)
			delete m_root;

		triangles->CompleteDelete();
		delete triangles;
		triangles = new LinkedList<Face>();

		DeleteIdenticalVertices();

		PointCloudTriangulation::DeleunayTriangulator *pTriangulator = new PointCloudTriangulation::DeleunayTriangulator();
		//pTriangulator->setKNeighParams(0.02, 8, 12);
		//pTriangulator->setCenterFactorParams(0.2, 0.5, 2.0);

		int numOfTriangles = 0;
		int *triangleverts = NULL;
		float *normals = NULL;
		unsigned int numOfVertices = points->GetSize();
		Vertex** tmp_points = new Vertex* [numOfVertices];

		// prekopcime do pola, koli lepsiemu pristupu
		LinkedList<Vertex>::Cell<Vertex>* tmp = points->start;
		unsigned int i = 0;
		while(tmp != NULL)
		{
			tmp_points[i] = tmp->data;
			tmp = tmp->next;
			i++;
		}

		// zoznam ktory davam do Madovej libky
		float * verts = new float[numOfVertices * 3];
		for(unsigned int ii = 0; ii < numOfVertices; ii++)
		{
			verts[ii * 3] = tmp_points[ii]->P.X;
			verts[ii * 3 + 1] = tmp_points[ii]->P.Y;
			verts[ii * 3 + 2] = tmp_points[ii]->P.Z;
		}

		for(unsigned int iii = 0; iii < numOfVertices; iii++)
		{
			delete tmp_points[iii]->susedia;
			tmp_points[iii]->susedia = new LinkedList<void>();
		}

		// triangulacia v Madovej libke
		pTriangulator->computeGlobalTriangulationFromPoints(numOfVertices, verts, numOfTriangles, &triangleverts, &normals, false);

		// kopcenie normal
		for (unsigned int l=0; l<numOfVertices; l++)
		{
			Vector4 normala = Vector4(normals[l * 3 + 0], normals[l * 3 + 1], normals[l * 3 + 2]);
			tmp_points[l]->SetNormal(normala);
		}

		// prelinkovanie struktur
		for (int l=0; l<numOfTriangles; l++)
		{
			int v1 = triangleverts[l * 3 + 0];
			int v2 = triangleverts[l * 3 + 1];
			int v3 = triangleverts[l * 3 + 2];		
			Face* novy_face = new Face(tmp_points[v1], tmp_points[v2], tmp_points[v3]);
			Vector4 t_nor = tmp_points[v1]->GetNormal() + tmp_points[v2]->GetNormal() + tmp_points[v3]->GetNormal();
			t_nor.Normalize();
			float theta = acos(novy_face->normal * t_nor);
			theta = theta * float(180.0 / M_PI);
			if(theta > 90.0f)
			{
				novy_face->v[0] = tmp_points[v3];
				novy_face->v[2] = tmp_points[v1];
			}
			novy_face->normal = t_nor;
			triangles->InsertToEnd(novy_face);
			tmp_points[v1]->susedia->InsertToEnd(novy_face);
			tmp_points[v2]->susedia->InsertToEnd(novy_face);
			tmp_points[v3]->susedia->InsertToEnd(novy_face);
		}

		loaded = true;

		// postprocessing ako Octree a pod.
		CreateOctree();
		SetColors();
		ComputeSusedov();

		delete pTriangulator;
		delete [] verts;
		delete [] triangleverts;
		delete [] normals;
		delete [] tmp_points;
	}
	// resetuje "show" nastavenia
	void CModel::ResetSettings()
	{
		loaded = false;
		draw_mode = 1;
		show_octree = false;
		show_normals = false;
		selected = NULL;

		b_size = 0.0;
		b_sf = 1.0;
		b_max = 0.0;
	}

	// vypocita rozmery modelu
	void CModel::ComputeBoundary()
	{
		float minx = 99999.0, miny = 99999.0, minz = 99999.0;
		float maxx = -99999.0, maxy = -99999.0, maxz = -99999.0;

		LinkedList<Vertex>::Cell<Vertex>* tmp = points->start;
		while(tmp != NULL)
		{
			if(tmp->data->P.X < minx)
				minx = tmp->data->P.X;
			if(tmp->data->P.Y < miny)
				miny = tmp->data->P.Y;
			if(tmp->data->P.Z < minz)
				minz = tmp->data->P.Z;

			if(tmp->data->P.X > maxx)
				maxx = tmp->data->P.X;
			if(tmp->data->P.Y > maxy)
				maxy = tmp->data->P.Y;
			if(tmp->data->P.Z > maxz)
				maxz = tmp->data->P.Z;

			tmp = tmp->next;
		}

		b_stred = Vector4((minx+maxx) / 2.0f, (miny+maxy) / 2.0f, (minz+maxz) / 2.0f);
		float sizex = 0;
		float sizey = 0;
		float sizez = 0;

		if(((minx<=0.0)&&(maxx<=0.0)) || ((minx>=0.0)&&(maxx>=0.0)))
			sizex = abs(minx+maxx);
		else
			sizex = abs(minx-maxx);

		if(((miny<=0.0)&&(maxy<=0.0)) || ((miny>=0.0)&&(maxy>=0.0)))
			sizey = abs(miny+maxy);
		else
			sizey = abs(miny-maxy);

		if(((minz<=0.0)&&(maxz<=0.0)) || ((minz>=0.0)&&(maxz>=0.0)))
			sizez = abs(minz+maxz);
		else
			sizez = abs(minz-maxz);
		b_size = max(max(sizex, sizey), sizez);

		b_sf = b_size / 10.0f;											// 1 / 10 velkosti modelu budu tie vektory
		b_max = sqrt(3.0f) * b_size;									// diagonala kocky
		b_size = b_size / 2.0f;

		SDF_control = new CSDFController(b_max, Assimp);
	}

	// vytvori Octree strukturu
	void CModel::CreateOctree()
	{
		m_root = new Octree(0, b_size, b_stred, NULL);

		unsigned int siz = triangles->GetSize();
		if(siz > 0)
		{
			Face** tria = new Face* [siz];
			LinkedList<Face>::Cell<Face>* tmp2 = triangles->start;
			int i = 0;
			while(tmp2 != NULL)
			{
				tria[i] = tmp2->data;
				tmp2 = tmp2->next;
				i++;
			}
			m_root->Build(tria, siz);
		}
		else
			m_root->Build(NULL, 0);
	}

	// nastavi farby pre picking
	void CModel::SetColors()
	{
		int col;
		int r = 0, g = 0, b = 0;
		LinkedList<Face>::Cell<Face>* tmp = triangles->start;
		while(tmp != NULL)
		{
			r = r + color_step;
			if(r >= 256) { r = r % 256; g = g + color_step;}
			if(g >= 256) { g = g % 256; b = b + color_step;}
			if(b >= 256) { b = b % 256; logInfo("MAXIMUM FARIEB PRE PICKING DOSIAHNUTE!!");}

			col = r + 256 * g + 256 * 256 * b;
			if(tmp->data != NULL)
				tmp->data->SetColor(col);
			//logInfo(MarshalString("farba: "+col+", RGB: " + r+" "+g+" "+b));

			tmp = tmp->next;
		}
	}

	// nastavi susedov pre facy
	void CModel::ComputeSusedov()
	{
		LinkedList<Face>::Cell<Face>* tmp = triangles->start;
		while(tmp != NULL)
		{
			tmp->data->ComputeSusedov();
			tmp = tmp->next;
		}
	}

	// pre spravne vycentrovanie pohladu ked sa nacita CModel
	void CModel::GetBoundary(float &siz, float &x, float &y, float &z)
	{
		siz = b_size;
		x = b_stred.X;
		y = b_stred.Y;
		z = b_stred.Z;
	}

	// vykresli CModel
	void CModel::DrawModel()
	{
	}

	void CModel::ColorToRGB(int color, unsigned char &R, unsigned char &G, unsigned char &B)
	{
		R = color % 256;
		int g = int((color - R) / 256);
		if(g >= 256)
		{
			G = g % 256;
			B = int((g - G) / 256);
		}
		else G = g;
	}

	void CModel::HLSToRGB(float SDF_value, unsigned char &R, unsigned char &G, unsigned char &B)
	{

	}

	void CModel::setDrawMode(int mode)
	{
		draw_mode = mode;
	}
	int CModel::getDrawMode()
	{
		return draw_mode;
	}

	int CModel::GetTriangleCount()
	{
		return triangles->GetSize();
	}

	void CModel::ProcessPick(int x, int y)
	{

	}
	void CModel::ComputeSDF()
	{
		SDF_control->Compute(triangles, m_root);
	}
}