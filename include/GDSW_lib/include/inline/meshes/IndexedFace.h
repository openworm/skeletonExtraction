#pragma once
#include <vector>
#include <mmath\mmath.h>

namespace meshes {
        struct IndexedFace
        {
                std::vector<float> vertices;
                std::vector<float> normals;
                std::vector<float> tangents;
                std::vector<float> uvs;
                std::vector<int> indices;
                std::vector<float> fdata;
                std::vector<int> idata;
                IndexedFace() {};
                IndexedFace(const std::vector<float> &vertices, const std::vector<int> &indices) : vertices(vertices), indices(indices) {};
                mmath::CVector3 GetVertex(int idx) {
                        int location = idx*3;
                        return mmath::CVector3(vertices[location + 0], vertices[location + 1], vertices[location + 2]);
                }
                mmath::CVector3 GetTriangle(int idx) {
                        int location = idx*3;
						return mmath::CVector3(indices[location + 0], indices[location + 1], indices[location + 2]);
                }
                mmath::CVector2 GetUVs(int idx) {
                        int location = idx*2;
						return mmath::CVector2(uvs[location + 0], uvs[location + 1]);
                }
        };
}
