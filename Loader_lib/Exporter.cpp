#include "Exporter.h"
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>


namespace Loader {
	Exporter::Exporter(void) : document(NULL)
	{
	}


	Exporter::~Exporter(void)
	{
		CleanDocument();
	}

	void Exporter::CleanDocument() {
		if (document) delete document;
	}

	void Exporter::GenerateHeader() {
		//create new document
		CleanDocument();
		document = new tinyxml2::XMLDocument();
		//create xml declaration
		tinyxml2::XMLDeclaration *declaration = document->NewDeclaration(NULL);
		document->InsertEndChild(declaration);
		//create collada root element
		root = document->NewElement("COLLADA");
		root->SetAttribute("xmlns", "http://www.collada.org/2005/11/COLLADASchema");
		root->SetAttribute("version", "1.4.1");
		document->InsertEndChild(root);
		//create header elements and their content
		tinyxml2::XMLElement *asset = document->NewElement("asset");

		tinyxml2::XMLElement *contributor = document->NewElement("contributor");

		tinyxml2::XMLElement *author = document->NewElement("author");
		tinyxml2::XMLText* authorText = document->NewText("misop");
		author->InsertEndChild(authorText);

		tinyxml2::XMLElement *created = document->NewElement("created");
		tinyxml2::XMLText* createdText = document->NewText("2014-08-23T22:29:59Z");
		created->InsertEndChild(createdText);

		tinyxml2::XMLElement *modified = document->NewElement("modified");
		tinyxml2::XMLText* modifiedText = document->NewText("2014-08-23T22:29:59Z");
		modified->InsertEndChild(modifiedText);

		tinyxml2::XMLElement *up_axis = document->NewElement("up_axis");
		tinyxml2::XMLText* up_axisText = document->NewText("Y_UP");
		up_axis->InsertEndChild(up_axisText);
		//link new elements
		contributor->InsertEndChild(author);
		asset->InsertEndChild(contributor);
		asset->InsertEndChild(created);
		asset->InsertEndChild(modified);
		asset->InsertEndChild(up_axis);
		root->InsertEndChild(asset);
	}

	tinyxml2::XMLElement* Exporter::ExportVectorToSource(std::vector<float> &data, Parameters &params) {	
		tinyxml2::XMLElement *source = document->NewElement("source");
		source->SetAttribute("id", params.id.c_str());
		source->SetAttribute("name", params.name.c_str());

		std::string id = params.id + "_array";
		tinyxml2::XMLElement *float_array = document->NewElement("float_array");
		float_array->SetAttribute("id", id.c_str());
		float_array->SetAttribute("count", std::to_string(data.size()).c_str());
		//fill with data
		std::ostringstream oss;
		std::copy(data.begin(), data.end(), std::ostream_iterator<float>(oss, " "));
		tinyxml2::XMLText *arrayText = document->NewText(oss.str().c_str());
		float_array->InsertEndChild(arrayText);

		source->InsertEndChild(float_array);

		tinyxml2::XMLElement *technique_common = document->NewElement("technique_common");
		source->InsertEndChild(technique_common);

		id = "#" + id;
		tinyxml2::XMLElement *accessor = document->NewElement("accessor");
		accessor->SetAttribute("source", id.c_str());
		accessor->SetAttribute("count", std::to_string(data.size()/params.elements).c_str());
		accessor->SetAttribute("stride", std::to_string(params.elements).c_str());
		technique_common->InsertEndChild(accessor);

		for (int i = 0; i < params.elements; i++) {
			tinyxml2::XMLElement *param = document->NewElement("param");
			param->SetAttribute("name", params.params[i].c_str());
			param->SetAttribute("type", "float");
			accessor->InsertEndChild(param);
		}

		return source;
	}

	void Exporter::ExportVertices(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {	
		Parameters params;
		params.id = "positions";
		params.name = "Vertices";
		params.elements = 3;
		params.params.push_back("X");
		params.params.push_back("Y");
		params.params.push_back("Z");
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->vertices, params);
		meshElement->InsertEndChild(source);
	}

	bool Exporter::ExportNormals(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		if (mesh->normals.empty()) return false;
		Parameters params;
		params.id = "normals";
		params.name = "Normals";
		params.elements = 3;
		params.params.push_back("X");
		params.params.push_back("Y");
		params.params.push_back("Z");
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->normals, params);
		meshElement->InsertEndChild(source);

		return true;
	}

	bool Exporter::ExportUVs(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement) {
		if (mesh->uvs.empty()) return false;
		Parameters params;
		params.id = "textureCoords";
		params.name = "TextureCoords";
		params.elements = 2;
		params.params.push_back("S");
		params.params.push_back("T");
		tinyxml2::XMLElement *source = ExportVectorToSource(mesh->uvs, params);
		meshElement->InsertEndChild(source);

		return true;
	}

	void Exporter::ExportTriangles(meshes::IndexedFace *mesh, tinyxml2::XMLElement *meshElement, bool normals, bool uvs) {
		if (mesh->indices.empty()) return;

		tinyxml2::XMLElement *triangles = document->NewElement("triangles");
		triangles->SetAttribute("count", std::to_string(mesh->indices.size()/3).c_str());
		meshElement->InsertEndChild(triangles);

		tinyxml2::XMLElement *input = document->NewElement("input");
		input->SetAttribute("semantic", "VERTEX");
		input->SetAttribute("source", "#verts");
		input->SetAttribute("offset", "0");
		triangles->InsertEndChild(input);

		if (normals) {
			input = document->NewElement("input");
			input->SetAttribute("semantic", "NORMAL");
			input->SetAttribute("source", "#normals");
			input->SetAttribute("offset", "1");
			triangles->InsertEndChild(input);
		}

		if (uvs) {
			input = document->NewElement("input");
			input->SetAttribute("semantic", "TEXCOORD");
			input->SetAttribute("source", "#textureCoords");
			input->SetAttribute("offset", normals ? "2" : "1");
			input->SetAttribute("set", "1");
			triangles->InsertEndChild(input);
		}
		
		tinyxml2::XMLElement *p = document->NewElement("p");
		triangles->InsertEndChild(p);
		//TODO make it work with normals and uvs
		std::ostringstream oss;
		int count = 1 + (normals ? 1 : 0) + (uvs ? 1 : 0);
		for (int i = 0; i < mesh->indices.size(); i++) {
			int idx = mesh->indices[i];
			for (int j = 0; j < count; j++) {
				oss << idx << " ";
			}
		}
		tinyxml2::XMLText *pText = document->NewText(oss.str().c_str());
		p->InsertEndChild(pText);
	}

	bool Exporter::ExportMesh(meshes::IndexedFace *mesh) {
		if (mesh->vertices.size() == 0) return false;
		//create library
		tinyxml2::XMLElement *library = document->NewElement("library_geometries");
		root->InsertEndChild(library);
		//create worm geometry
		tinyxml2::XMLElement *geometry = document->NewElement("geometry");
		geometry->SetAttribute("id", "worm_id");
		geometry->SetAttribute("name", "worm");
		library->InsertEndChild(geometry);
		//create mesh and add mesh data
		tinyxml2::XMLElement *meshElement = document->NewElement("mesh");
		geometry->InsertEndChild(meshElement);
		ExportVertices(mesh, meshElement);
		bool normals = ExportNormals(mesh, meshElement);
		bool uvs = ExportUVs(mesh, meshElement);
		//create vertices
		tinyxml2::XMLElement *vertices = document->NewElement("vertices");
		vertices->SetAttribute("id", "verts");
		tinyxml2::XMLElement *input = document->NewElement("input");
		input->SetAttribute("semantic", "POSITION");
		input->SetAttribute("source", "#positions");
		vertices->InsertEndChild(input);

		meshElement->InsertEndChild(vertices);
		ExportTriangles(mesh, meshElement, normals, uvs);

		return true;
	}

	bool Exporter::SaveToFile(std::string fileName) {
		int error_id = document->SaveFile(fileName.c_str());
		return !error_id;
	}
}