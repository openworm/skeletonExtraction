#pragma once

#include <inline/SkeletonNode/s_skeletonNode.h>

namespace SM {
	/// <summary>
	/// Calculates quaternions transforming source skeleton to destination skeleton.
	/// </summary>
	/// <param name="source">Source skeleton</param>
	/// <param name="dest">Destination skeleton</param>
	/// <returns>Sets destination skeleton matrices.qRotation to a quaternion that when applied would transform destination skeleton to source skeleton</returns>
	void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest);
}