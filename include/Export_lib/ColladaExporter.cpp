/************************************************************************************
*
* The MIT License (MIT)
* 
* Copyright (c) 2014 Michal Piovarèi
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
***********************************************************************************/

#include "ColladaExporter.h"
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>
#include <queue>
#include <ctime>

namespace Export {

	namespace {
		inline std::string toID(std::string str) {
			return "#" + str;
		}
		inline std::string toIDAppend(int number) {
			return "_" + std::to_string(number);
		}

		inline std::string GetCurrentDate() {
			time_t t = time(0);
			struct tm *timeInfo = localtime(&t);
			char buf[15];
			strftime(buf, sizeof(buf), "%Y-%m-%d", timeInfo);
			return buf;
		}
	}

#pragma region Constant strings

#pragma region COLLADA version
	const char COLLADA_SCHEMA[]          = "http://www.collada.org/2005/11/COLLADASchema";
	const char COLLADA_VERSION[]         = "1.4.1";
#pragma endregion

#pragma region Constants
	const char SEPARATOR[]               = " ";

	const char INV_BIND_MATRIX[]         = "INV_BIND_MATRIX";
	const char JOINT[]                   = "JOINT";
	const char FLOAT4x4[]                = "float4x4";
	const char FLOAT_ARRAY[]             = "float_array";
	const char NAME_VALUE[]              = "Name";
	const char NODE_VALUE[]              = "NODE";
	const char NORMAL[]                  = "NORMAL";
	const char POSITION[]                = "POSITION";
	const char TEXTCOORD[]               = "TEXCOORD";
	const char VERTEX[]                  = "VERTEX";
	const char WEIGHT[]                  = "WEIGHT";

	const char X[]                       = "X";
	const char Y[]                       = "Y";
	const char Z[]                       = "Z";
	const char S[]                       = "S";
	const char T[]                       = "T";

	const char X_UP_STR[]                = "X_UP";
	const char Y_UP_STR[]                = "Y_UP";
	const char Z_UP_STR[]                = "Z_UP";

	const char ROTATE_X[]                = "rotateX";
	const char ROTATE_Y[]                = "rotateY";
	const char ROTATE_Z[]                = "rotateZ";

	const char JOINT_ORIENT_X[]          = "jointOrientX";
	const char JOINT_ORIENT_Y[]          = "jointOrientY";
	const char JOINT_ORIENT_Z[]          = "jointOrientZ";
#pragma endregion

#pragma region File variables
	// IDs and variables used in exported file
	const char NORMALS_ID[]              = "normals";
	const char POSITIONS_ID[]            = "positions";
	const char SCENE_ID[]                = "VisualSceneNode";
	const char SKELETON_ID[]             = "skeleton_root";
	const char TEX_COORDS_ID[]           = "textureCoords";
	const char VERTICES_ID[]             = "verts";
	//model names
	const char DEFAULT_AUTHOR_NAME[]     = "open-worm";
	const char DEFAULT_NAME[]            = "worm";
	const char BIND_POSES_STR[]          = "-skin-bind_poses";
	const char JOINTS_STR[]              = "-skin-joints";
	const char MESH_STR[]                = "-mesh";
	const char SKIN_STR[]                = "-skin";
	const char WEIGHTS_STR[]             = "-skin-weights";
#pragma endregion

#pragma region XML elements
	const char ACCESSOR[]                = "accessor";
	const char ASSET[]                   = "asset";
	const char AUTHOR[]                  = "author";
	const char BIND_SHAPE_MATRIX[]       = "bind_shape_matrix";
	const char COLLADA[]                 = "COLLADA";
	const char CONTRIBUTOR[]             = "contributor";
	const char CONTROLLER[]              = "controller";
	const char CREATED[]                 = "created";
	const char FLOAT[]                   = "float";
	const char GEOMETRY[]                = "geometry";
	const char INPUT[]                   = "input";
	const char INSTANCE_CONTROLLER[]     = "instance_controller";
	const char INSTANCE_GEOMETRY[]       = "instance_geometry";
	const char INSTANCE_VISUAL_SCENE[]   = "instance_visual_scene";
	const char LIBRARY_CONTROLLERS[]     = "library_controllers";
	const char LIBRARY_GEOMETRIES[]      = "library_geometries";
	const char LIBRARY_VISUAL_SCENES[]   = "library_visual_scenes";
	const char MESH[]                    = "mesh";
	const char MODIFIED[]                = "modified";
	const char NAME_ARRAY[]              = "Name_array";
	const char NODE[]                    = "node";
	const char PARAM[]                   = "param";
	const char ROTATE[]                  = "rotate";
	const char SCENE[]                   = "scene";
	const char SKELETON[]                = "skeleton";
	const char SKIN[]                    = "skin";
	const char TECHNIQUE_COMMON[]        = "technique_common";
	const char TRANSLATE[]               = "translate";
	const char TRIANGLES[]               = "triangles";
	const char UP_AXIS[]                 = "Y_UP";
	const char VERTICES[]                = "vertices";
	const char V[]                       = "v";
	const char VISUAL_SCENE[]            = "visual_scene";
	const char VCOUNT[]                  = "vcount";
#pragma endregion

#pragma region XML attributes
	const char COUNT[]                   = "count";
	const char ID[]                      = "id";
	const char NAME[]                    = "name";
	const char OFFSET[]                  = "offset";
	const char SEMANTIC[]                = "semantic";
	const char SET[]                     = "set";
	const char SID[]                     = "sid";
	const char SOURCE[]                  = "source";
	const char STRIDE[]                  = "stride";
	const char TRANSFORM[]               = "TRANSFORM";
	const char TYPE[]                    = "type";
	const char URL[]                     = "url";
	const char VERSION[]                 = "version";
	const char XMLNS[]                   = "xmlns";
#pragma endregion

#pragma endregion

#pragma region Init

	ColladaExporter::ColladaExporter(void) : document(NULL), upAxisValue(Y_UP_STR), authorName(DEFAULT_AUTHOR_NAME)
	{
		SetModelName(DEFAULT_NAME);
		CleanDocument();
	}

	ColladaExporter::ColladaExporter(std::string modelName) : document(NULL), upAxisValue(Y_UP_STR), authorName(DEFAULT_AUTHOR_NAME) {
		SetModelName(modelName);
		CleanDocument();
	}

	ColladaExporter::~ColladaExporter(void)
	{
		CleanDocument();
	}

	void ColladaExporter::CleanDocument() {
		if (document) delete document;
		document = NULL;
		root = NULL;
		library_geometries = NULL;
		library_controllers = NULL;
		library_visual_scenes = NULL;
		visual_scene = NULL;
		meshCount = 0;
		skeletonCount = 0;
	}

#pragma endregion

#pragma region Setters

	void ColladaExporter::SetModelName(std::string modelName) {
		modelID          = modelName;

		bindPosesID      = modelName + BIND_POSES_STR;
		jointsID         = modelName + JOINTS_STR;
		meshID           = modelName + MESH_STR;
		skinID           = modelName + SKIN_STR;
		weightsID        = modelName + WEIGHTS_STR;
	}

	void ColladaExporter::SetUpAxis(Axis axis) {
		switch (axis)
		{
		case Axis::X_UP:
			upAxisValue = X_UP_STR;
			break;
		case Axis::Y_UP:
			upAxisValue = Y_UP_STR;
			break;
		case Axis::Z_UP:
			upAxisValue = Z_UP_STR;
			break;
		}
	}

#pragma endregion

#pragma region XML Header

	void ColladaExporter::GenerateHeader() {
		//create new document
		CleanDocument();
		document = new tinyxml2::XMLDocument();
		//create xml declaration
		tinyxml2::XMLDeclaration *declaration = document->NewDeclaration(NULL);
		document->InsertEndChild(declaration);
		//create collada root element
		root = document->NewElement(COLLADA);
		root->SetAttribute(XMLNS, COLLADA_SCHEMA);
		root->SetAttribute(VERSION, COLLADA_VERSION);
		document->InsertEndChild(root);
		//create header elements and their content
		tinyxml2::XMLElement *asset = document->NewElement(ASSET);

		tinyxml2::XMLElement *contributor = document->NewElement(CONTRIBUTOR);

		tinyxml2::XMLElement *author = document->NewElement(AUTHOR);
		tinyxml2::XMLText* authorText = document->NewText(authorName.c_str());
		author->InsertEndChild(authorText);

		std::string date = GetCurrentDate();
		tinyxml2::XMLElement *created = document->NewElement(CREATED);
		tinyxml2::XMLText* createdText = document->NewText(date.c_str());
		created->InsertEndChild(createdText);

		tinyxml2::XMLElement *modified = document->NewElement(MODIFIED);
		tinyxml2::XMLText* modifiedText = document->NewText(date.c_str());
		modified->InsertEndChild(modifiedText);

		tinyxml2::XMLElement *up_axis = document->NewElement(UP_AXIS);
		tinyxml2::XMLText* up_axisText = document->NewText(upAxisValue.c_str());
		up_axis->InsertEndChild(up_axisText);
		//link new elements
		contributor->InsertEndChild(author);
		asset->InsertEndChild(contributor);
		asset->InsertEndChild(created);
		asset->InsertEndChild(modified);
		asset->InsertEndChild(up_axis);
		root->InsertEndChild(asset);
	}

#pragma endregion

#pragma region COLLADA <source>

	template< typename T > tinyxml2::XMLElement* ColladaExporter::ExportVectorToSource(std::vector<T> &data, Parameters &params) {	
		tinyxml2::XMLElement *source = document->NewElement(SOURCE);
		source->SetAttribute(ID, params.id.c_str());
		if (!params.name.empty()) source->SetAttribute(NAME, params.name.c_str());

		std::string id = params.id + "-array";
		tinyxml2::XMLElement *data_array = document->NewElement(params.element.c_str());
		data_array->SetAttribute(ID, id.c_str());
		data_array->SetAttribute(COUNT, std::to_string(data.size()).c_str());
		//fill with data
		std::ostringstream oss;
		std::copy(data.begin(), data.end(), std::ostream_iterator<T>(oss, SEPARATOR));
		tinyxml2::XMLText *arrayText = document->NewText(oss.str().c_str());
		data_array->InsertEndChild(arrayText);

		source->InsertEndChild(data_array);

		tinyxml2::XMLElement *technique_common = document->NewElement(TECHNIQUE_COMMON);
		source->InsertEndChild(technique_common);

		id = toID(id);
		tinyxml2::XMLElement *accessor = document->NewElement(ACCESSOR);
		accessor->SetAttribute(SOURCE, id.c_str());
		accessor->SetAttribute(COUNT, std::to_string(data.size()/params.stride).c_str());
		accessor->SetAttribute(STRIDE, std::to_string(params.stride).c_str());
		technique_common->InsertEndChild(accessor);

		for (int i = 0; i < params.params.size(); i++) {
			tinyxml2::XMLElement *param = document->NewElement(PARAM);
			param->SetAttribute(NAME, params.params[i].first.c_str());
			param->SetAttribute(TYPE, params.params[i].second.c_str());
			accessor->InsertEndChild(param);
		}

		return source;
	}

#pragma endregion

#pragma region Mesh Export

	void ColladaExporter::ExportVertices(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		std::string meshCountStr = toIDAppend(meshCount);

		Parameters params;
		params.element = FLOAT_ARRAY;
		params.id = POSITIONS_ID + meshCountStr;
		params.name = POSITIONS_ID + meshCountStr;
		params.stride = 3;
		params.params.push_back(std::make_pair(X, FLOAT));
		params.params.push_back(std::make_pair(Y, FLOAT));
		params.params.push_back(std::make_pair(Z, FLOAT));
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->vertices, params);
		meshElement->InsertEndChild(source);
	}

	void ColladaExporter::ExportNormals(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		if (mesh->normals.empty()) return;
		std::string meshCountStr = toIDAppend(meshCount);

		Parameters params;
		params.element = FLOAT_ARRAY;
		params.id = NORMALS_ID + meshCountStr;
		params.name = NORMALS_ID + meshCountStr;
		params.stride = 3;
		params.params.push_back(std::make_pair(X, FLOAT));
		params.params.push_back(std::make_pair(Y, FLOAT));
		params.params.push_back(std::make_pair(Z, FLOAT));
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->normals, params);
		meshElement->InsertEndChild(source);
	}

	void ColladaExporter::ExportUVs(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		if (mesh->uvs.empty()) return;
		std::string meshCountStr = toIDAppend(meshCount);

		Parameters params;
		params.element = FLOAT_ARRAY;
		params.id = TEX_COORDS_ID + meshCountStr;
		params.name = TEX_COORDS_ID + meshCountStr;
		params.stride = 2;
		params.params.push_back(std::make_pair(S, FLOAT));
		params.params.push_back(std::make_pair(T, FLOAT));
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->uvs, params);
		meshElement->InsertEndChild(source);
	}

	void ColladaExporter::ExportTriangles(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		if (mesh->indices.empty()) return;
		std::string meshCountStr = toIDAppend(meshCount);
		bool normals = !mesh->normals.empty();
		bool uvs = !mesh->uvs.empty();

		tinyxml2::XMLElement *triangles = document->NewElement(TRIANGLES);
		triangles->SetAttribute(COUNT, std::to_string(mesh->indices.size()/3).c_str());
		meshElement->InsertEndChild(triangles);

		tinyxml2::XMLElement *input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, VERTEX);
		input->SetAttribute(SOURCE, toID(VERTICES_ID + meshCountStr).c_str());
		input->SetAttribute(OFFSET, "0");
		triangles->InsertEndChild(input);

		if (normals) {
			input = document->NewElement(INPUT);
			input->SetAttribute(SEMANTIC, NORMAL);
			input->SetAttribute(SOURCE, toID(NORMALS_ID + meshCountStr).c_str());
			input->SetAttribute(OFFSET, "1");
			triangles->InsertEndChild(input);
		}

		if (uvs) {
			input = document->NewElement(INPUT);
			input->SetAttribute(SEMANTIC, TEXTCOORD);
			input->SetAttribute(SOURCE, toID(TEX_COORDS_ID + meshCountStr).c_str());
			input->SetAttribute(OFFSET, normals ? "2" : "1");
			input->SetAttribute(SET, "1");
			triangles->InsertEndChild(input);
		}

		tinyxml2::XMLElement *p = document->NewElement("p");
		triangles->InsertEndChild(p);
		std::ostringstream oss;
		int count = 1 + (normals ? 1 : 0) + (uvs ? 1 : 0);
		for (int i = 0; i < mesh->indices.size(); i++) {
			int idx = mesh->indices[i];
			for (int j = 0; j < count; j++) {
				oss << idx << SEPARATOR;
			}
		}
		tinyxml2::XMLText *pText = document->NewText(oss.str().c_str());
		p->InsertEndChild(pText);
	}

	bool ColladaExporter::ExportMesh(meshes::IndexedFace *mesh) {
		if (!mesh || mesh->vertices.size() == 0) return false;
		std::string meshCountStr = toIDAppend(meshCount);

		//create library
		if (!library_geometries) {
			library_geometries = document->NewElement(LIBRARY_GEOMETRIES);
			root->InsertEndChild(library_geometries);
		}
		//create worm geometry
		tinyxml2::XMLElement *geometry = document->NewElement(GEOMETRY);
		geometry->SetAttribute(ID, (meshID + meshCountStr).c_str());
		geometry->SetAttribute(NAME, (meshID + meshCountStr).c_str());
		library_geometries->InsertEndChild(geometry);
		//create mesh and add mesh data
		tinyxml2::XMLElement *meshElement = document->NewElement(MESH);
		geometry->InsertEndChild(meshElement);
		ExportVertices(mesh, meshElement);
		ExportNormals(mesh, meshElement);
		ExportUVs(mesh, meshElement);
		//create vertices
		tinyxml2::XMLElement *vertices = document->NewElement(VERTICES);
		vertices->SetAttribute(ID, (VERTICES_ID + meshCountStr).c_str());
		tinyxml2::XMLElement *input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, POSITION);
		input->SetAttribute(SOURCE, toID(POSITIONS_ID + meshCountStr).c_str());
		vertices->InsertEndChild(input);

		meshElement->InsertEndChild(vertices);
		ExportTriangles(mesh, meshElement);

		meshCount++;
		return true;
	}

	void ColladaExporter::FinishMeshExport() {
		if (!library_visual_scenes) {
			library_visual_scenes = document->NewElement(LIBRARY_VISUAL_SCENES);
			root->InsertEndChild(library_visual_scenes);

			if (!visual_scene){
				visual_scene = document->NewElement(VISUAL_SCENE);
				visual_scene->SetAttribute(ID, SCENE_ID);
				visual_scene->SetAttribute(NAME, "untitled");
				library_visual_scenes->InsertEndChild(visual_scene);

				tinyxml2::XMLElement *scene = document->NewElement(SCENE);
				root->InsertEndChild(scene);

				tinyxml2::XMLElement *instance_visual_scene = document->NewElement(INSTANCE_VISUAL_SCENE);
				instance_visual_scene->SetAttribute(URL, toID(SCENE_ID).c_str());
				scene->InsertEndChild(instance_visual_scene);
			}

		}

		for (int i = 0; i < meshCount; i++) {
			CreateMeshInstanceNode(i);
		}

	}

#pragma endregion

#pragma region Skin Export

	void ColladaExporter::ExportSkeletonToJoints(SN::SkeletonNode *node, tinyxml2::XMLElement *element) {
		std::string meshCountStr = toIDAppend(meshCount);

		std::vector<std::string> jointNames;
		std::queue<SN::SkeletonNode*> queue;
		queue.push(node);
		while (!queue.empty()) {
			SN::SkeletonNode *aNode = queue.front();
			queue.pop();
			std::string str = "joint" + std::to_string(aNode->id);
			jointNames.push_back(str);
			for (int i = 0; i < aNode->nodes.size(); i++) {
				queue.push(aNode->nodes[i]);
			}
		}

		Parameters params;
		params.element = NAME_ARRAY;
		params.id = jointsID + meshCountStr;
		params.stride = 1;
		params.params.push_back(std::make_pair(JOINT, NAME_VALUE));
		tinyxml2::XMLElement *source = ExportVectorToSource(jointNames, params);
		element->InsertEndChild(source);
	}

	void ColladaExporter::ExportSkeletonToMatrices(SN::SkeletonNode *node, tinyxml2::XMLElement *element) {
		std::string meshCountStr = toIDAppend(meshCount);

		std::vector<float> bindMatrices;
		int count = 0;
		std::queue<SN::SkeletonNode*> queue;
		queue.push(node);
		while (!queue.empty()) {
			SN::SkeletonNode *aNode = queue.front();
			queue.pop();
			float matrixRow1[] = {1, 0, 0, -aNode->point.x};
			float matrixRow2[] = {0, 1, 0, -aNode->point.y};
			float matrixRow3[] = {0, 0, 1, -aNode->point.z};
			float matrixRow4[] = {0, 0, 0, 1};
			bindMatrices.insert(bindMatrices.end(), std::begin(matrixRow1), std::end(matrixRow1));
			bindMatrices.insert(bindMatrices.end(), std::begin(matrixRow2), std::end(matrixRow2));
			bindMatrices.insert(bindMatrices.end(), std::begin(matrixRow3), std::end(matrixRow3));
			bindMatrices.insert(bindMatrices.end(), std::begin(matrixRow4), std::end(matrixRow4));
			count++;
			for (int i = 0; i < aNode->nodes.size(); i++) {
				queue.push(aNode->nodes[i]);
			}
		}

		Parameters params;
		params.element = FLOAT_ARRAY;
		params.id = bindPosesID + meshCountStr;
		params.stride = 16;
		params.params.push_back(std::make_pair(TRANSFORM, FLOAT4x4));
		tinyxml2::XMLElement *source = ExportVectorToSource(bindMatrices, params);
		element->InsertEndChild(source);
	}

	void ColladaExporter::ExportWeights(std::vector<float> &weights, tinyxml2::XMLElement *element) {
		std::string meshCountStr = toIDAppend(meshCount);

		Parameters params;
		params.element = FLOAT_ARRAY;
		params.id = weightsID + meshCountStr;
		params.stride = 1;
		params.params.push_back(std::make_pair(WEIGHT, FLOAT));
		tinyxml2::XMLElement *source = ExportVectorToSource(weights, params);
		element->InsertEndChild(source);
	}

	void ColladaExporter::ExportVertexWeights(meshes::MeshSkin *mesh, tinyxml2::XMLElement *element) {
		std::string meshCountStr = toIDAppend(meshCount);

		std::string influencePairs = "";
		for (int i = 0; i < mesh->jointIDs.size(); i++) {
			influencePairs += std::to_string(mesh->jointIDs[i]) + " " + std::to_string(i) + " ";
		}
		int vertices = mesh->vertices.size()/3;
		tinyxml2::XMLElement *vertex_weights = document->NewElement("vertex_weights");
		vertex_weights->SetAttribute(COUNT, std::to_string(vertices).c_str());
		element->InsertEndChild(vertex_weights);
		//set input as pair joint-weight
		tinyxml2::XMLElement *input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, JOINT);
		input->SetAttribute(SOURCE, toID(jointsID + meshCountStr).c_str());
		input->SetAttribute(OFFSET, "0");
		vertex_weights->InsertEndChild(input);
		input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, WEIGHT);
		input->SetAttribute(SOURCE, toID(weightsID + meshCountStr).c_str());
		input->SetAttribute(OFFSET, "1");
		vertex_weights->InsertEndChild(input);
		//add number of pairs for each vertex
		std::ostringstream oss;
		std::copy(mesh->influences.begin(), mesh->influences.end(), std::ostream_iterator<int>(oss, SEPARATOR));
		tinyxml2::XMLElement *vcount = document->NewElement(VCOUNT);
		tinyxml2::XMLText *vcountText = document->NewText(oss.str().c_str());
		vcount->InsertEndChild(vcountText);
		vertex_weights->InsertEndChild(vcount);
		tinyxml2::XMLElement *v = document->NewElement(V);
		tinyxml2::XMLText *vText = document->NewText(influencePairs.c_str());
		v->InsertEndChild(vText);
		vertex_weights->InsertEndChild(v);
	}

	void ColladaExporter::ExportSkin(meshes::MeshSkin *mesh, SN::SkeletonNode *skeleton) {
		std::string meshCountStr = toIDAppend(meshCount);
		//create library of controllers
		if (!library_controllers) {
			library_controllers = document->NewElement(LIBRARY_CONTROLLERS);
			root->InsertEndChild(library_controllers);
		}
		//create first controller
		tinyxml2::XMLElement *controller = document->NewElement(CONTROLLER);
		controller->SetAttribute(ID, (skinID + meshCountStr).c_str());
		controller->SetAttribute(NAME, (skinID + meshCountStr).c_str());
		library_controllers->InsertEndChild(controller);
		//add skin to the controller
		tinyxml2::XMLElement *skin = document->NewElement(SKIN);
		skin->SetAttribute(SOURCE, toID(meshID + meshCountStr).c_str());
		controller->InsertEndChild(skin);
		//bind shape matrix for the skin
		tinyxml2::XMLElement *bind_shape_matrix = document->NewElement(BIND_SHAPE_MATRIX);
		tinyxml2::XMLText *bind_shape_matrixText = document->NewText("1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		bind_shape_matrix->InsertEndChild(bind_shape_matrixText);
		skin->InsertEndChild(bind_shape_matrix);
		//add list of all skeleton joints
		ExportSkeletonToJoints(skeleton, skin);
		//add inverse bind matrices for joints
		ExportSkeletonToMatrices(skeleton, skin);
		//add weight
		ExportWeights(mesh->skinWeights, skin);
		//add joints
		tinyxml2::XMLElement *joints = document->NewElement("joints");
		skin->InsertEndChild(joints);
		tinyxml2::XMLElement *input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, JOINT);
		input->SetAttribute(SOURCE, toID(jointsID + meshCountStr).c_str());
		joints->InsertEndChild(input);
		input = document->NewElement(INPUT);
		input->SetAttribute(SEMANTIC, INV_BIND_MATRIX);
		input->SetAttribute(SOURCE, toID(bindPosesID + meshCountStr).c_str());
		joints->InsertEndChild(input);
		//associate weights with vertices and joints
		ExportVertexWeights(mesh, skin);

		meshCount++;
	}

#pragma endregion

#pragma region Skeleton Export

	void ColladaExporter::ExportSkeleton(SN::SkeletonNode *aNode, tinyxml2::XMLElement *nodeRoot) {
		std::string skeletonCountStr = toIDAppend(skeletonCount);

		std::string name = "joint" + std::to_string(aNode->id);
		tinyxml2::XMLElement *node = document->NewElement(NODE);
		if (aNode->father == NULL) node->SetAttribute(ID, (SKELETON_ID + skeletonCountStr).c_str());
		node->SetAttribute(NAME, name.c_str());
		node->SetAttribute(SID, name.c_str());
		node->SetAttribute(TYPE, JOINT);
		nodeRoot->InsertEndChild(node);
		//add node translation relative to parent node
		mmath::CVector3 P = aNode->point;
		if (aNode->father) P = P - aNode->father->point;

		std::string translateStr = std::to_string(P.x) + " " + std::to_string(P.y) + " " + std::to_string(P.z);
		tinyxml2::XMLElement *translate = document->NewElement(TRANSLATE);
		translate->SetAttribute(SID, TRANSLATE);
		tinyxml2::XMLText *translateText = document->NewText(translateStr.c_str());
		translate->InsertEndChild(translateText);
		node->InsertEndChild(translate);
		//orientation Z
		/*tinyxml2::XMLElement *rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, JOINT_ORIENT_Z);
		tinyxml2::XMLText *rotateText = document->NewText("0 0 1 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);
		//orientation Y
		rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, JOINT_ORIENT_Y);
		rotateText = document->NewText("0 1 0 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);
		//orientation X
		rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, JOINT_ORIENT_X);
		rotateText = document->NewText("1 0 0 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);*/
		//add child nodes
		for (int i = 0; i < aNode->nodes.size(); i++) {
			ExportSkeleton(aNode->nodes[i], node);
		}
	}

	void ColladaExporter::ExportSkeleton(SN::SkeletonNode *skeleton) {
		if (!skeleton) return;
		//create skeleton
		if (!library_visual_scenes) {
			library_visual_scenes = document->NewElement(LIBRARY_VISUAL_SCENES);
			root->InsertEndChild(library_visual_scenes);

			if (!visual_scene){
				visual_scene = document->NewElement(VISUAL_SCENE);
			
				visual_scene->SetAttribute(ID, SCENE_ID);
				visual_scene->SetAttribute(NAME, "bind");
				library_visual_scenes->InsertEndChild(visual_scene);
				//create scene
				tinyxml2::XMLElement *scene = document->NewElement(SCENE);
				root->InsertEndChild(scene);
			
				tinyxml2::XMLElement *instance_visual_scene = document->NewElement(INSTANCE_VISUAL_SCENE);
				instance_visual_scene->SetAttribute(URL, toID(SCENE_ID).c_str());
				scene->InsertEndChild(instance_visual_scene);
			}
		}
		//export skeleton
		ExportSkeleton(skeleton, visual_scene);

		skeletonCount++;
	}

	void ColladaExporter::BindMeshWithSkeleton() {
		std::vector<int> meshSkeletonMap;
		meshSkeletonMap.push_back(0);
		BindMeshesWithSkeletons(meshSkeletonMap);
	}


	void ColladaExporter::CreateMeshInstanceNode(int index) {
		std::string meshCountStr = toIDAppend(index);
		tinyxml2::XMLElement *node = document->NewElement(NODE);
		node->SetAttribute(ID, (modelID + meshCountStr).c_str());
		node->SetAttribute(NAME, (modelID + meshCountStr).c_str());
		visual_scene->InsertEndChild(node);

		tinyxml2::XMLElement *rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, ROTATE_Z);
		tinyxml2::XMLText *rotateText = document->NewText("0 0 1 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);

		rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, ROTATE_Y);
		rotateText = document->NewText("0 1 0 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);

		rotate = document->NewElement(ROTATE);
		rotate->SetAttribute(SID, ROTATE_X);
		rotateText = document->NewText("1 0 0 0");
		rotate->InsertEndChild(rotateText);
		node->InsertEndChild(rotate);

		tinyxml2::XMLElement *instance_geometry = document->NewElement(INSTANCE_GEOMETRY);
		instance_geometry->SetAttribute(URL, toID(meshID + meshCountStr).c_str());
		node->InsertEndChild(instance_geometry);
		//add material as child of instance geometry
	}

	void ColladaExporter::CreateSkinedMeshInstanceNode(int meshIndex, int skeletonIndex) {
		std::string meshCountStr = toIDAppend(meshIndex);
		std::string skeletonIDStr = toIDAppend(skeletonIndex);
		//bind mesh and skeleton
		tinyxml2::XMLElement *node = document->NewElement(NODE);
		node->SetAttribute(ID, (modelID + meshCountStr).c_str());
		node->SetAttribute(NAME, (modelID + meshCountStr).c_str());
		node->SetAttribute(TYPE, NODE_VALUE);
		visual_scene->InsertEndChild(node);
		tinyxml2::XMLElement *instane_controller = document->NewElement(INSTANCE_CONTROLLER);
		instane_controller->SetAttribute(URL, toID(skinID + meshCountStr).c_str());
		node->InsertEndChild(instane_controller);
		tinyxml2::XMLElement *skeletonElement = document->NewElement(SKELETON);
		instane_controller->InsertEndChild(skeletonElement);
		tinyxml2::XMLText *skeletonText = document->NewText(toID(SKELETON_ID + skeletonIDStr).c_str());
		skeletonElement->InsertEndChild(skeletonText);
		//material can be binded here
	}

	void ColladaExporter::BindMeshesWithSkeletons(std::vector<int> &meshSkeletonMap) {
		for (int i = 0; i < meshCount; i++) {
			if (meshSkeletonMap[i] == -1) {
				CreateMeshInstanceNode(i);
			} else {
				CreateSkinedMeshInstanceNode(i, meshSkeletonMap[i]);
			}
		}
	}

#pragma endregion

#pragma region File Output

	bool ColladaExporter::SaveToFile(std::string fileName) {
		int error_id = document->SaveFile(fileName.c_str());
		return !error_id;
	}

#pragma endregion

#pragma region Export

	bool ColladaExporter::Export(meshes::IndexedFace *mesh, std::string fileName) {
		if (!mesh) return false;
		GenerateHeader();
		ExportMesh(mesh);
		FinishMeshExport();
		return SaveToFile(fileName);
	}

	bool ColladaExporter::Export(meshes::IndexedFace *mesh, SN::SkeletonNode *skeleton, std::string fileName) {
		if (!mesh || !skeleton) return false;
		GenerateHeader();
		ExportMesh(mesh);
		ExportSkeleton(skeleton);
		FinishMeshExport();
		return SaveToFile(fileName);
	}

	bool ColladaExporter::Export(meshes::MeshSkin *mesh, SN::SkeletonNode *skeleton, std::string fileName) {
		if (!mesh || !skeleton) return false;
		GenerateHeader();
		ExportMesh(mesh);
		meshCount = 0;
		ExportSkin(mesh, skeleton);
		ExportSkeleton(skeleton);
		BindMeshWithSkeleton();
		return SaveToFile(fileName);
	}

	bool ColladaExporter::Export(std::vector<meshes::IndexedFace*> &meshes, std::string fileName) {
		if (meshes.empty()) return false;
		GenerateHeader();
		for (int i = 0; i < meshes.size(); i++) {
			ExportMesh(meshes[i]);
		}
		FinishMeshExport();
		return SaveToFile(fileName);
	}

	bool ColladaExporter::Export(std::vector<meshes::MeshSkin*> &meshes, std::vector<SN::SkeletonNode*> &skeletons, std::vector<int> &meshSkeletonMap, std::string fileName) {
		if (meshes.empty()) return false;
		GenerateHeader();
		for (int i = 0; i < meshes.size(); i++) {
			ExportMesh(meshes[i]);
		}
		//reset meshcount to count ids for skin
		meshCount = 0;
		for (int i = 0; i < meshes.size(); i++) {
			ExportSkin(meshes[i], skeletons[meshSkeletonMap[i]]);
		}
		for (int i = 0; i < skeletons.size(); i++) {
			ExportSkeleton(skeletons[i]);
		}
		BindMeshesWithSkeletons(meshSkeletonMap);
		return SaveToFile(fileName);
	}

#pragma endregion

}