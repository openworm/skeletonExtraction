#pragma once

#include <meshes/structure.h>
#include <SkeletonNode/SkeletonNode.h>

#include <LBSE_lib/lbse_extractor.h>
#include <LBSE_lib/lbse_skeleton.h>

#include <SDF_lib/sdf_extractor.h>
#include <SDF_lib/Helpers/SDFSettings.h>

#include <Import_lib/WormLoader.h>
#include <Export_lib/ColladaExporter.h>
#include <Export_lib/MatrixExport.h>

#include <SM_lib/QuaternionsBetweenWormSkeletons.h>

#include "openworm_wrapper.h"

void coreFunc(int timeStep);
