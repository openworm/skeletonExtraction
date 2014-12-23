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
#include <fstream>

namespace SM {
	typedef std::vector<skl::SkeletonNode*> LinearSkeleton;
	namespace {
		void TranslateSkeletonToOrigin(SN::SkeletonNode* skeleton);
		void TranslateSkeleton(SN::SkeletonNode* skeleton, mmath::CVector3 translate);
		void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest, glm::mat3 &M);
		int CalculateSimplifiedSkeletonMapping(int &idx, skl::SkeletonNode *skeleton, std::map<int, int> &map, float threshold, glm::quat error);
		void LinearizeSkeletons(std::vector<skl::SkeletonNode*> &skeletons, std::vector<LinearSkeleton> &linearSkeletons);
		void LinearizeWorms(std::vector<skl::SkeletonNode*> &skeletons, std::vector<LinearSkeleton> &linearSkeletons);

		float HalfCos2Angle(float cos) {
			return acos(cos) * 2;
		}

		void TranslateSkeletonToOrigin(SN::SkeletonNode* skeleton) {
			mmath::CVector3 translate = skeleton->point * -1.0;
			TranslateSkeleton(skeleton, translate);
		}
		void TranslateSkeleton(SN::SkeletonNode* skeleton, mmath::CVector3 translate) {
			std::queue<SN::SkeletonNode*> queue;
			queue.push(skeleton);

			while (!queue.empty()) {
				SN::SkeletonNode *node = queue.front();
				queue.pop();

				node->point = node->point + translate;

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
			dest->matrices.vTranslation = CVector3(0,0,0);
			dest->matrices.currentAffine = Array2D<float>(4,4,0.0);
			dest->matrices.computeAffineTransformation();
			//adjust transformation matrix
			M = glm::mat3_cast(q) * M;
			//if there is a child node calculate its quaternions
			if (!(source->nodes.empty() || dest->nodes.empty())) {
				CalculateWormTransformationMatrices((skl::SkeletonNode*)source->nodes[0], (skl::SkeletonNode*)dest->nodes[0], M);
			}
		}

		int CalculateSimplifiedSkeletonMapping(int &idx, skl::SkeletonNode *skeleton, std::map<int, int> &map, float threshold, glm::quat error) {
			//TODO not finished yet
			skl::SkeletonNode *node = skeleton;
			int savedCounter = 0;

			while (node->nodes.size() < 2) {
				glm::quat quat(node->matrices.qRotation.s, node->matrices.qRotation.i, node->matrices.qRotation.j, node->matrices.qRotation.k);
				error = quat * error;
				map[node->id] = idx;
				if (HalfCos2Angle(error.w) < threshold) {
					savedCounter++;
				} else {
					idx++;
				}
			}
			//create functions for each branch node and reset error
			for (int i = 0; i < node->nodes.size(); i++) {
				savedCounter += CalculateSimplifiedSkeletonMapping(idx, (skl::SkeletonNode*)node->nodes[i], map, threshold, glm::quat());
			}

			return savedCounter;
		}

		void LinearizeSkeletons(std::vector<skl::SkeletonNode*> &skeletons, std::vector<LinearSkeleton> &linearSkeletons) {
			linearSkeletons.resize(skeletons.size());
			for (int i = 0; i < skeletons.size(); i++) {
				linearSkeletons[i].clear();
				std::queue<skl::SkeletonNode*> queue;
				queue.push(skeletons[i]);

				while (!queue.empty()) {
					skl::SkeletonNode *node = queue.front();
					queue.pop();

					linearSkeletons[i].push_back(node);

					for (int j = 0; j < node->nodes.size(); j++) {
						queue.push((skl::SkeletonNode*)node->nodes[j]);
					}
				}
			}
		}

		void LinearizeWorms(std::vector<skl::SkeletonNode*> &skeletons, std::vector<LinearSkeleton> &linearSkeletons) {
			linearSkeletons.resize(skeletons.size());
			for (int i = 0; i < skeletons.size(); i++) {
				linearSkeletons[i].clear();
				skl::SkeletonNode *node = skeletons[i];

				while (node != NULL) {
					linearSkeletons[i].push_back(node);

					if (node->nodes.size() == 1)
						node = (skl::SkeletonNode*)node->nodes[0];
					else if (node->nodes.empty())
						node = NULL;
					else {
						std::string errorMessage = "Input skeleton is not a worm. Node #" + std::to_string(node->id) + " contains " + std::to_string(node->nodes.size()) + " child nodes!";
						std::exception error(errorMessage.c_str());
						throw error;
					}
				}
			}
		}
	}

	void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest) {
		//check if skeletons exist and have at least one son
		if (!source || !dest || source->nodes.empty() || dest->nodes.empty()) return;
		//remember original positions
		// set dest root translation to zero
		dest->matrices.vTranslation = CVector3(0,0,0);
		dest->matrices.computeAffineTransformation();

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

	int CalculateSimplifiedWormMapping(std::vector<SkeletonRotation> &skeletonRotations, std::vector<int> &map, float threshold) {
		glm::quat error;
		threshold = glm::radians(threshold);
		map.clear();
		map.reserve(skeletonRotations.size());
		int idx = 0;
		int savedCounter = 0;

		for (int i = 0; i < skeletonRotations.size(); i++) {
			error = glm::normalize(skeletonRotations[i].rotation * error);
			float angle = HalfCos2Angle(error.w);

			if (angle <= threshold) {
				//percieved change is too small
				map.push_back(-1);
				savedCounter++;
			} else {
				//percieved change is big enough reset the error and change transformation
				skeletonRotations[i].rotation = error;
				map.push_back(idx);
				error = glm::quat();
				idx++;
			}
		}

		return savedCounter;
	}

	int CalculateSimplifiedWormMapping(std::vector<glm::quat> &skeletonRotations, std::vector<bool> &map, float threshold) {
		glm::quat error;
		threshold = glm::radians(threshold);
		map.clear();
		map.reserve(skeletonRotations.size());
		int savedCounter = 0;

		for (int i = 0; i < skeletonRotations.size(); i++) {
			error = glm::normalize(skeletonRotations[i] * error);
			float angle = HalfCos2Angle(error.w);

			if (angle <= threshold) {
				//percieved change is too small set no rotation
				skeletonRotations[i] = glm::quat();
				map.push_back(false);
				savedCounter++;
			} else {
				//percieved change is big enough reset the error and apply error to transformation
				skeletonRotations[i] = error;
				map.push_back(true);
				error = glm::quat();
			}
		}

		return savedCounter;
	}

	int CalculateSimplifiedWormMapping(skl::SkeletonNode *source, float threshold) {
		glm::quat error;
		threshold = glm::radians(threshold);
		skl::SkeletonNode *node = source;
		int savedCounter = 0;

		while (node != NULL) {
			glm::quat rotation(node->matrices.qRotation.s, node->matrices.qRotation.i, node->matrices.qRotation.j, node->matrices.qRotation.k);
			error = glm::normalize(rotation * error);
			float angle = HalfCos2Angle(error.w);

			if (angle <= threshold) {
				//percieved change is too small
				node->matrices.qRotation = Quaternion();
				savedCounter++;
			} else {
				//percieved change is big enough reset the error and update transformation
				node->matrices.qRotation = Quaternion(error.w, error.x, error.y, error.z);
				error = glm::quat();
			}

			//check if skeleton is actually a worm, if not raise exception
			if (node->nodes.size() > 1) {
				std::string errorMessage = "Input skeleton is not a worm. Node #" + std::to_string(node->id) + " contains " + std::to_string(node->nodes.size()) + " child nodes!";
				std::exception error(errorMessage.c_str());
				throw error;
			}
			//otherwise continue
			if (node->nodes.size() == 1) {
				node = (skl::SkeletonNode*)node->nodes[0];
			} else {
				node = NULL;
			}
		}

		return savedCounter;
	}

	int CalculateSimplifiedWormMapping(std::vector<SkeletonRotation> &source, std::vector<SkeletonRotation> &dest, std::vector<int> &map, float threshold) {
		float error = 0;
		threshold = glm::radians(threshold);
		map.clear();
		map.reserve(dest.size());
		int idx = 0;
		int savedCounter = 0;

		for (int i = 0; i < source.size(); i++) {
			glm::vec3 sourceV = source[i].rotation * glm::vec3(1, 0, 0);
			glm::vec3 destV = dest[i].rotation * glm::vec3(1, 0, 0);
			glm::quat quat = glm::QuatBetweenVectors(sourceV, destV);
			float angle = HalfCos2Angle(quat.w) + error;

			if (angle <= threshold) {
				//percieved change is too small
				map.push_back(-1);
				error = angle;
				savedCounter++;
			} else {
				//percieved change is big enough reset the error
				map.push_back(idx);
				error = 0;
				idx++;
			}
		}

		return savedCounter;
	}

	int CalculateSimplifiedSkeletonMapping(skl::SkeletonNode *skeleton, std::map<int, int> &map, float threshold) {
		return -1;
		int savedCounter = 0;
		int idx = 0;
		for (int i = 0; i < skeleton->nodes.size(); i++) {
			savedCounter += CalculateSimplifiedSkeletonMapping(idx, (skl::SkeletonNode*)skeleton->nodes[i], map, glm::radians(threshold), glm::quat());
		}
		return savedCounter;
	}

	void TransformWormSkeleton(skl::SkeletonNode *skeleton, std::vector<glm::mat4> &transformations) {
		skl::SkeletonNode *node = skeleton;

		for (int i = 0; i < transformations.size(); i++) {
			if (node == NULL) return;

			glm::vec4 P(node->point.x, node->point.y, node->point.z, 1);
			P = transformations[i] * P;
			node->point = CVector3(P.x, P.y, P.z);

			if (node->nodes.size() == 1) {
				node = (skl::SkeletonNode*)node->nodes[0];
			} else if (node->nodes.size() == 0) {
				node = NULL;
			} else {
				std::string errorMessage = "Input skeleton is not a worm. Node #" + std::to_string(node->id) + " contains " + std::to_string(node->nodes.size()) + " child nodes!";
				std::exception error(errorMessage.c_str());
				throw error;
			}
		}
	}

	float SkTED(std::vector<skl::SkeletonNode*> &source, std::vector<skl::SkeletonNode*> &proposed, int n, float d) {
		if (source.size() == 0 || source.size() != proposed.size() || source.size() < 2*n + 1)
			return 0;

		float values = 0;
		float SkTED = 0;
		std::vector<LinearSkeleton> linearSource;
		LinearizeSkeletons(source, linearSource);
		std::vector<LinearSkeleton> linearProposed;
		LinearizeSkeletons(proposed, linearProposed);

		//for each node (they are the same count during the animation)
		for (int i = 0; i < linearSource[0].size(); i++) {
			//calculated tel of each frame
			std::vector<float> tel;
			for (int f = 0; f < source.size() - 1; f++) {
				CVector3 temp = linearSource[f][i]->point - linearSource[f+1][i]->point;
				tel.push_back(sqrtf(temp.x*temp.x + temp.y*temp.y + temp.z*temp.z + d*d));
			}
			//calculated tel_p of each frame
			std::vector<float> tel_p;
			for (int f = 0; f < proposed.size() - 1; f++) {
				CVector3 temp = linearProposed[f][i]->point - linearProposed[f+1][i]->point;
				tel_p.push_back(sqrtf(temp.x*temp.x + temp.y*temp.y + temp.z*temp.z + d*d));
			}
			//calculated s for each frame
			std::vector<float> s(source.size(), 0);
			for (int f = n; f < source.size() - n; f++) {
				float s_v = 0;
				float count = 0;
				for (int ff = -n; ff < n; ff++) {
					s_v += tel[f + ff];
					count++;
				}
				s_v /= count;
				s[f] = s_v;
			}
			//calculate ted for each frame
			for (int f = n; f < source.size() - n; f++) {
				SkTED += (fabsf(tel[f] - tel_p[f]) / s[f]);
				values++;
			}
		}

		SkTED = SkTED / values;
		return SkTED;
	}

	void ExportToFile(std::vector<skl::SkeletonNode*> &source, std::string fileName, float frameSpeed) {
		if (source.empty() || fileName.empty()) return;

		std::vector<LinearSkeleton> linearSources;
		LinearizeWorms(source, linearSources);

		std::ofstream of(fileName);
		//store # of skeletal nodes, number of frames and frame speed
		of << linearSources[0].size() << std::endl;
		of << linearSources.size() << std::endl;
		of << frameSpeed << std::endl;
		//store absolute positions (x, y, z) of each skeletal node for each frame
		for (int f = 0; f < linearSources.size(); f++) {
			for (int i = 0; i < linearSources[f].size(); i++) {
				of << linearSources[f][i]->point.x << " ";
				of << linearSources[f][i]->point.y << " ";
				of << linearSources[f][i]->point.z << std::endl;
			}
		}
	}

	int FindIncorrectSkeletons(std::vector<skl::SkeletonNode*> &source, std::vector<bool> &valid, std::vector<float> &ds, float threshold) {
		if (source.empty() || source.size() < 3) return 0;

		std::vector<LinearSkeleton> linearSources;
		LinearizeWorms(source, linearSources);
		int counter = 0;

		float lastD = 0;

		for (int i = 0; i < linearSources[0].size(); i++) {
			lastD += Length(linearSources[1][i]->point - linearSources[0][i]->point);
		}
		//first two frames are always good
		valid.push_back(true);
		valid.push_back(true);

		ds.push_back(0);
		ds.push_back(lastD);

		bool good = true;
		for (int f = 2; f < linearSources.size(); f++) {
			float d = 0;

			bool error = false;
			for (int i = 0; i < linearSources[i].size(); i++) {
				d += Length(linearSources[f][i]->point - linearSources[f-1][i]->point);
			}

			if (d > threshold) {
				good = !good;
			}

			if (!good) counter++;
			valid.push_back(good);
			ds.push_back(d);
		}

		return counter;
	}
}