#pragma once
#include <vector>
#include <map>
#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtx\quaternion.hpp>

#include <inline/SkeletonNode/s_skeletonNode.h>

namespace SM {
	typedef struct SkeletonRotation {
		int id;
		glm::quat rotation;
		SkeletonRotation(int id, glm::quat rotation) : id(id), rotation(rotation) { };
	} SkeletonRotation;
	/// <summary>
	/// Calculates quaternions transforming source skeleton to destination skeleton.
	/// </summary>
	/// <param name="source">Source skeleton</param>
	/// <param name="dest">Destination skeleton</param>
	/// <returns>Sets destination skeleton matrices.qRotation to a quaternion that when applied would transform destination skeleton to source skeleton</returns>
	void CalculateWormTransformationMatrices(skl::SkeletonNode *source, skl::SkeletonNode *dest);
	/// <summary>
	/// Calculates mapping which simplifies rotations not percievable by human eye on a linear skeleton
	/// </summary>
	/// <param name="skeletonRotations">Source skeleton rotations as quaternions and id of skeletal node</param>
	/// <param name="map">Output map of removed rotations</param>
	/// <param name="threshold">Threshold for change in angle not percievable by human eye. Measured in degrees. Varies with size of the model.</param>
	/// <returns>Return simplified maping and number of removed rotations.</returns>
	int CalculateSimplifiedWormMapping(std::vector<SkeletonRotation> &skeletonRotations, std::vector<int> &map, float threshold);
	/// <summary>
	/// Calculates mapping which simplifies rotations not percievable by human eye on a linear skeleton and simplifies quaternions.
	/// </summary>
	/// <param name="skeletonRotations">Source skeleton rotations as quaternions</param>
	/// <param name="map">Output map of removed rotations</param>
	/// <param name="threshold">Threshold for change in angle not percievable by human eye. Measured in degrees. Varies with size of the model.</param>
	/// <returns>Return simplified maping, recalculated transformations and number of removed rotations.</returns>
	int CalculateSimplifiedWormMapping(std::vector<glm::quat> &skeletonRotations, std::vector<bool> &map, float threshold);
	/// <summary>
	/// Calculates simplified transformation directly on the skeleton
	/// </summary>
	/// <param name="skeletonRotations">Source skeleton which rotations will be compressed</param>
	/// <param name="threshold">Threshold for change in angle not percievable by human eye. Measured in degrees. Varies with size of the model.</param>
	/// <returns>Simplifies rotations in the input skeleton and return number of simplified rotations.</returns>
	int CalculateSimplifiedWormMapping(skl::SkeletonNode *source, float threshold);
	/// <summary>
	/// Calculates mapping which simplifies rotations not percievable by human eye between two linear skeletons.
	/// Both skeletons must have the same number of nodes.
	/// </summary>
	/// <param name="source">Source skeleton rotations as quaternions and id of skeletal node</param>
	/// <param name="dest">Destination skeleton rotations as quaternions and id of skeletal node</param>
	/// <param name="map">Output map of removed rotations</param>
	/// <param name="threshold">Threshold for change in angle not percievable by human eye. Measured in degrees. Varies with size of the model.</param>
	/// <returns>Return simplified maping and number of removed rotations.</returns>
	int CalculateSimplifiedWormMapping(std::vector<SkeletonRotation> &source, std::vector<SkeletonRotation> &dest, std::vector<int> &map, float threshold);
	/// <summary>
	/// Calculates mapping which simplifies rotations not percievable by human eye.
	/// </summary>
	/// <param name="skeleton">Source skeleton</param>
	/// <param name="map">Output map of removed rotations</param>
	/// <param name="threshold">Threshold for change in angle not percievable by human eye. Measured in degrees. Varies with size of the model.</param>
	/// <returns>Return simplified maping and number of removed rotations.</returns>
	int CalculateSimplifiedSkeletonMapping(skl::SkeletonNode *skeleton, std::map<int, int> &map, float threshold);
	/// <summary>
	/// Transform worm skeleton with precalculated transformation matrices
	/// </summary>
	/// <param name="skeleton">Source skeleton</param>
	/// <param name="transformations">Vector of rotations applied on each skeletal node</param>
	/// <returns>Returns modified source skeleton by the transformations in the input vector</returns>
	void TransformWormSkeleton(skl::SkeletonNode *skeleton, std::vector<glm::mat4> &transformations);
	/// <summary>
	/// Calculates SkTED error for each frame of the animation
	/// </summary>
	/// <param name="source">Source skeletons</param>
	/// <param name="proposed">Proposed skeletons</param>
	/// <param name="n">Range of frames arround each frame to evaluate SkTED.</param>
	/// <param name="d">Temporal distance between frames.</param>
	/// <returns>Return measured SkTED error.</returns>
	float SkTED(std::vector<skl::SkeletonNode*> &source, std::vector<skl::SkeletonNode*> &proposed, int n = 2, float d = 0.016666667);
	/// <summary>
	/// Exports skeletal animation to file. First skeleton is in bind pose rest are animation frames.
	/// </summary>
	/// <param name="source">Source skeletons</param>
	/// <param name="fileName">File name where the skeletal structure would be saved</param>
	/// <param name="frameSpeed">Speed of one frame.</param>
	/// <returns>Saves skeleton to file</returns>
	void ExportToFile(std::vector<skl::SkeletonNode*> &source, std::string filName, float frameSpeed = 0.016666667);
	/// <summary>
	/// Finds incorrect skeletal structures in an animation. Incorrect structure causes a jump in the animation
	/// </summary>
	/// <param name="source">Source skeletons</param>
	/// <param name="valid">Output vector of valid skeletons</param>
	/// <param name="threshold">threshold of change in % from last change</param>
	/// <returns>Number of incorrect skeletons</returns>
	int FindIncorrectSkeletons(std::vector<skl::SkeletonNode*> &source, std::vector<bool> &valid, std::vector<float> &ds, float threshold);
}