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

#include "QuaternionsBetweenWormSkeletons.h"
#include <queue>
#include <mmath\mmath.h>
#include <glm\glm.hpp>
#include <glm\custom\aditions.h>

namespace SM {
	namespace {
		void TranslateSkeletonToOrigin(SN::SkeletonNode* skeleton);
		void TranslateSkeleton(SN::SkeletonNode* skeleton, mmath::CVector3 translate);
		void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest, glm::mat3 &M);

		void TranslateSkeletonToOrigin(SN::SkeletonNode* skeleton) {
			mmath::CVector3 translate = skeleton->point;
			TranslateSkeleton(skeleton, translate);
		}
		void TranslateSkeleton(SN::SkeletonNode* skeleton, mmath::CVector3 translate) {
			std::queue<SN::SkeletonNode*> queue;
			queue.push(skeleton);

			while (!queue.empty()) {
				SN::SkeletonNode *node = queue.front();
				queue.pop();

				node->point = node->point - translate;

				for (int i = 0; i < node->nodes.size(); i++) {
					queue.push(node->nodes[i]);
				}
			}
		}
		void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest, glm::mat3 &M) {
			//get source direction
			mmath::CVector3 sourceParent = source->father->point;
			mmath::CVector3 sourcePoint = source->point;
			mmath::CVector3 sourceVec = sourcePoint - sourceParent;
			glm::vec3 s = glm::normalize(glm::vec3(sourceVec.x, sourceVec.y, sourceVec.z));
			//get destination direction			
			mmath::CVector3 destParent = dest->father->point;
			mmath::CVector3 destPoint = dest->point;
			mmath::CVector3 destVec = destPoint - destParent;
			glm::vec3 d = glm::normalize(glm::vec3(destVec.x, destVec.y, destVec.z));
			//apply transformation matrix from previous calculations
			s = M * s;
			//calculate quaternion
			glm::quat q = glm::QuatBetweenVectors(s, d);
			dest->matrices.qRotation = mmath::Quaternion(q.w, q.x, q.y, q.z);
			//adjust transformation matrix
			M = glm::mat3_cast(q) * M;
			//if there is a child node calculate its quaternions
			if (!(source->nodes.empty() || dest->nodes.empty())) {
				CalculateWormTransformationMatrices((skl::SkeletonNode*)source->nodes[0], (skl::SkeletonNode*)dest->nodes[0], M);
			}
		}
	}

	void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest) {
		//check if skeletons exist and have at least one son
		if (!source || !dest || source->nodes.empty() || dest->nodes.empty()) return;
		//remember original positions
		mmath::CVector3 offsetSource = source->point;
		mmath::CVector3 offsetDest = dest->point;
		//translate skeletons to origin
		TranslateSkeletonToOrigin(source);
		TranslateSkeletonToOrigin(dest);
		//calculate quaternion transformations
		glm::mat3 M(1);
		CalculateWormTransformationMatrices((skl::SkeletonNode*)source->nodes[0], (skl::SkeletonNode*)dest->nodes[0], M);
		//translate skeletons back
		TranslateSkeleton(source, offsetSource);
		TranslateSkeleton(dest, offsetDest);
	}
}