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

#include "MatrixExport.h"
#include <queue>
#include <fstream>

namespace Export {
	namespace {
		void ExportQRotation(skl::SkeletonNode *node, std::ofstream &output) {
			output << node->matrices.qRotation.s << " " << node->matrices.qRotation.i << " " << node->matrices.qRotation.j << " " << node->matrices.qRotation.k << std::endl;
		}

		void ExportAxisAngle(skl::SkeletonNode *node, std::ofstream &output) {
			CVector4 axisAngle = QuaternionToAxisAngle(node->matrices.qRotation);
			output << axisAngle.s * (180.0 / PI) << " " << axisAngle.i << " " << axisAngle.j << " " << axisAngle.k << std::endl;
		}

		void ExportAffineMatrix(skl::SkeletonNode *node, std::ofstream &output) {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					output << node->matrices.currentAffine[i][j] << " ";
				}
				output << std::endl;
			}
			output << std::endl;
		}

		void SaveToFile(skl::SkeletonNode *skeleton, std::string fileName, void ExportFunction(skl::SkeletonNode*, std::ofstream&)) {
			std::ofstream output(fileName, std::ofstream::out);
			std::queue<SN::SkeletonNode*> queue;
			queue.push(skeleton);

			while (!queue.empty()) {
				skl::SkeletonNode *node = (skl::SkeletonNode*)queue.front();
				queue.pop();

				ExportFunction(node, output);

				for (int i = 0; i < node->nodes.size(); i++) {
					queue.push(node->nodes[i]);
				}
			}

			output.close();
		}
	}

	void SaveQuaternionsToFile(skl::SkeletonNode *skeleton, std::string fileName) {
		SaveToFile(skeleton, fileName, ExportQRotation);
	}

	void SaveAxisAngleToFile(skl::SkeletonNode *skeleton, std::string fileName){
		SaveToFile(skeleton, fileName, ExportAxisAngle);
	}

	void SaveMatricesToFile(skl::SkeletonNode *skeleton, std::string fileName) {
		SaveToFile(skeleton, fileName, ExportAffineMatrix);
	}
}