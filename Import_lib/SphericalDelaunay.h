/*
 *  SphereDelaunay.h
 *  gtm
 *
 *  Created by J. Andreas Baerentzen on 04/12/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */
#pragma once

#include <vector>
#include <list>
#include <glm\glm.hpp>

void Delaunay_on_sphere(const std::vector<glm::vec3>& vertices, std::vector<glm::ivec3>& triangles_out);