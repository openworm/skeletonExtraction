// Vertex.h : subor pre manipulaciu s 3D bodom
#pragma once
#include "SDF_lib/Structures/Math/Vector4.h"
#include "SDF_lib/Structures/Generic/LinkedList.h"

namespace MeshStructures
{
	using namespace MathStructures;
	using namespace GenericStructures;

	class Vertex
	{
	public:
		Vertex(float x, float y, float z);
		Vertex(Vector4 position);
		~Vertex();
		void SetNormal(Vector4 normala);
		Vector4 GetNormal();


		Vector4 P;
		LinkedList<void>*		susedia;					// susedia
	private:
		Vector4					normal;
	};
}