// (c) 2010 Geom e.U. Bernhard Kornberger, Graz/Austria. All rights reserved.
//
// This file is part of the Fade2D library. You can use it for your personal
// non-commercial research. Licensees holding a commercial license may use this 
// file in accordance with the Commercial License Agreement provided 
// with the Software.
//
// This software is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING 
// THE WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Please contact the author if any conditions of this licensing are not clear 
// to you.
// 
// Author: Bernhard Kornberger, bkorn (at) geom.at 
// http://www.geom.at

#pragma once
#include <set>

#include "common.h"
#if GEOM_PSEUDO3D==GEOM_TRUE 
	namespace GEOM_FADE25D {
#elif GEOM_PSEUDO3D==GEOM_FALSE
	namespace GEOM_FADE2D {
#else
	#error GEOM_PSEUDO3D is not defined 
#endif 

class Point2; // FWD
class ConstraintGraph2; // FWD


enum ConstraintInsertionStrategy
{
	CIS_KEEP_DELAUNAY,
	CIS_IGNORE_DELAUNAY
};


class ConstraintSegment2
{
public:
	ConstraintSegment2(Point2* p0_,Point2* p1_,ConstraintInsertionStrategy cis_);
	~ConstraintSegment2();
	CLASS_DECLSPEC
	Point2* getSrc() const;
	CLASS_DECLSPEC
	Point2* getTrg() const;
	CLASS_DECLSPEC
	bool isAlive() const;
	ConstraintInsertionStrategy getCIS() const;
	CLASS_DECLSPEC
	bool operator<(const ConstraintSegment2& pOther) const;

/** \brief Split a constraint segment
* 
* Version 1: For internal use. 
*/
	CLASS_DECLSPEC
	bool split(Point2* pSplit);
/** \brief Split a constraint segment
* 
* Version 2: For development purposes 
*/
	CLASS_DECLSPEC
	bool splitAndRemovePrev(Point2* pSplit);
/** \brief Add an owner
* 
* Sets a specific ConstraintGraph2 as owner of the current ConstraintSegment2.
* Mostly for internal use.   
*/
	void addOwner(ConstraintGraph2* pOwner);
/** \brief Remove an owner
* Removes a specific ConstraintGraph2 as owner of the current ConstraintSegment2.
* Mostly for internal use. 
*/
	void removeOwner(ConstraintGraph2* pOwner);
	CLASS_DECLSPEC
/** \brief Get all children
* Recursively retrieve all children of the current ConstraintSegment2.
*/
	void getChildrenRec(std::vector<ConstraintSegment2*>& vChildConstraintSegments);
	CLASS_DECLSPEC
/** \brief Get the children and the split point
* Retrieve the two direct children of the current ConstraintSegment2 as well as the split point. 
*/
	void getChildrenAndSplitPoint(ConstraintSegment2*& pCSeg0,ConstraintSegment2*& pCSeg1,Point2*& pSplitPoint);
	CLASS_DECLSPEC
	friend std::ostream &operator<<(std::ostream &stream, const ConstraintSegment2& cSeg);
/** \brief Return the number of owners
* Returns the number of ConstraintGraphs where the current ConstraintSegment2 is a member
*/
	CLASS_DECLSPEC
	size_t getNumberOfOwners() const;

	ConstraintGraph2* getPolygonalOwner() const;

protected:
	Point2 *p0,*p1;
	ConstraintInsertionStrategy cis;
	std::set<ConstraintGraph2*> sOwners;
	std::vector<ConstraintSegment2*> vChildren;		
};

} // NAMESPACE
