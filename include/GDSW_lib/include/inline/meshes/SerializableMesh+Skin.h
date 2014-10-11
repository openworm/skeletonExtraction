#pragma once
#include <meshes/Mesh+Skin.h>
#include <string>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#pragma warning(push, 0)
#include <boost\archive\basic_xml_iarchive.hpp>
#include <boost\archive\xml_iarchive.hpp>
#include <boost\archive\xml_oarchive.hpp>
#pragma pop


namespace meshes {
	struct SerializableMeshSkin : MeshSkin
	{
		friend class ::boost::serialization::access;

		SerializableMeshSkin() : MeshSkin() {};
		SerializableMeshSkin(MeshSkin *mesh) {
			vertices = mesh->vertices;
			normals = mesh->normals;
			tangents = mesh->tangents;
			uvs = mesh->uvs;
			indices = mesh->indices;
			fdata = mesh->fdata;
			idata = mesh->idata;
			skinWeights = mesh->skinWeights;
			jointIDs = mesh->jointIDs;
			influences = mesh->influences;
		};
	protected:
		// When the class Archive corresponds to an output archive, the
		// & operator is defined similar to <<.  Likewise, when the class Archive
		// is a type of input archive the & operator is defined similar to >>.
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(vertices);
			ar & BOOST_SERIALIZATION_NVP(normals);
			ar & BOOST_SERIALIZATION_NVP(tangents);
			ar & BOOST_SERIALIZATION_NVP(uvs);
			ar & BOOST_SERIALIZATION_NVP(indices);
			ar & BOOST_SERIALIZATION_NVP(fdata);
			ar & BOOST_SERIALIZATION_NVP(idata);
			ar & BOOST_SERIALIZATION_NVP(skinWeights);
			ar & BOOST_SERIALIZATION_NVP(jointIDs);
			ar & BOOST_SERIALIZATION_NVP(influences);
		}
	};
}

BOOST_CLASS_VERSION(meshes::SerializableMeshSkin, 0)