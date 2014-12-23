#include "SkeletonSheetExtraction.h"
#include <poly2tri\poly2tri.h>

using namespace mmath;

/*namespace Sheet {
	namespace {
		const float FLOAT_PREC_DELTA = 0.000001;
		const float CUP_ANGLE_FLOAT_PREC_DELTA = 0.00001;

		void CalculateMappingAnglesForVertex(CVector3 v, float &firaw, float &fi, float &teta, float &t, float &normt, CVector3 &displacement, CVector3 start, CVector3 end, CVector3 skeletonBoneCenter, bool tangentSpace, CVector3 &tst) {
			CVector3 s = start - end;

			t = s.x * (v.x - end.x) + s.y * (v.y - end.y) + s.z * (v.z - end.z);
			t = t / (s.x * s.x + s.y * s.y + s.z * s.z);

			float tCut = t;

			if (t < 0){
				tCut = 0;
			}
			if (t > 1){
				tCut = 1;
			}

			CVector3 p = end + s * tCut;		

			// calcualtion of fi
			// 1) make ortogonla projection of vector skeletonBoneCenter - p on the plane perpendicular to the bone, reference vector
			// 2) angle between this projection of p - v and reference is angle fi

			CVector3 ref = p - skeletonBoneCenter;
			CVector3 normal_fi_plane = Normalize(s);
			CVector3 ref_projection_fi = Normalize(ref - normal_fi_plane * Dot(ref, normal_fi_plane)); // projection of reference vector (angle fi = 0)
			displacement = p - v;	
			CVector3 dir_projection_fi = Normalize(displacement - normal_fi_plane * Dot(displacement, normal_fi_plane)); // projection of our displacement vector

			CVector3 normal_teta_plane = Cross(normal_fi_plane, ref_projection_fi);

			// TODO: prerobit s fkciou oriented angle...

			fi = AngleBetweenVectors(ref_projection_fi, dir_projection_fi);
			float det = DeterminantInPlane(ref_projection_fi, dir_projection_fi, normal_fi_plane);

			float fpd = FLOAT_PREC_DELTA;

			if (Distance(v, start) < fpd * Magnitude(skeletonBoneCenter) || Distance(v, end) < fpd * Magnitude(skeletonBoneCenter)){ // if it is the same vertex as start / end point
				teta = PI / 2.0;
			} else {
				teta = AngleBetweenVectors(dir_projection_fi, displacement);
			}

			// if sdm is mapped in bone tangent space, rotate displacement vector into tangent space
			if (tangentSpace){

				CVector3 tsn = Normalize(dir_projection_fi);
				if (t < 0 || t > 1){
					tst = Normalize(CVector3(-sin(fi) * sin(teta), cos(fi) * sin(teta), cos(teta)));
					if (Dot(tst, s) < 0) // if segment orientation is opposit of tangent
						tst = tst * -1.0;
				}
				else
					tst = Normalize(s);
				CVector3 tsb = Cross(tst, tsn);

				CVector3 tangentDisplacement;

				tangentDisplacement.x = Dot(displacement, tst);
				tangentDisplacement.y = Dot(displacement, tsb);
				tangentDisplacement.z = Dot(displacement, tsn);

				displacement = tangentDisplacement;
			}

			float cafpd = CUP_ANGLE_FLOAT_PREC_DELTA; 
			if (abs(teta - PI/2.0) < cafpd)
				teta = PI / 2.0;

			if (det < 0)
				fi = 2 * PI - fi;
			if (fi <= PI)
				firaw = fi;
			else 
				firaw = fi - 2 * PI;
		}

		void polyg2tri(std::vector<CVector2> &points, std::vector<int> &o_triangles) {
			int id = 0;
			std::vector<p2t::Point *> polyline;
			polyline.reserve(points.size());
			for (int i = 0; i < points.size(); i++) {
				polyline.push_back(new p2t::Point(points[i].x, points[i].y, id));
				id++;
			}
			p2t::CDT cdt(polyline);
			cdt.Triangulate();
			std::vector<p2t::Triangle*> triangles = cdt.GetTriangles();
			for (int i = 0; i < triangles.size(); i++) {
				p2t::Triangle &t = *triangles[i];
				p2t::Point& a = *t.GetPoint(0);
				p2t::Point& b = *t.GetPoint(1);
				p2t::Point& c = *t.GetPoint(2);
				o_triangles.push_back(a.id);
				o_triangles.push_back(b.id);
				o_triangles.push_back(c.id);
			}
			for (std::vector<p2t::Point*>::iterator it = polyline.begin(); it != polyline.end(); ++it) {
				delete *it;
			}
		}
	}

	SkeletonSheetExtraction::SkeletonSheetExtraction(void)
	{
	}


	SkeletonSheetExtraction::~SkeletonSheetExtraction(void)
	{
	}

	void ExtractSkeletonSheet(std::vector<mmath::CVector3> &vertices, const mmath::CVector3 &start,  const mmath::CVector3 &end, std::vector<mmath::CVector3> &sheetPoints, std::vector<int> &sheetTriangulation) {
		float THRESHOLD = 5;
		//capsule arround bone -> find fi and theta for each vertex
		std::vector<CVector2> angles;
		for (int i = 0; i < vertices.size(); i++) {
			//calculate fi and theta
			float fi = 0, theta = 0;
			angles.push_back(CVector2(fi, theta));
		}
		//merge according to fi and theta
		std::vector<int> vertexMap;
		std::vector<CVector2> delaunayPoints;
		for (int i = 0; i < vertices.size(); i++) {
			bool add = true;
			for (int j = 0; j < delaunayPoints.size(); j++) {
				//also checking for cross capsule closness
				if (mmath::Distance(angles[i], delaunayPoints[j]) < THRESHOLD ||
					mmath::Distance(angles[i], delaunayPoints[j] - CVector2(0, 2*M_PI)) < THRESHOLD) {
						add = false;
						break;
				}
			}
			if (add) {
				delaunayPoints.push_back(angles[i]);
				vertexMap.push_back(i);
			}
		}
		//execute delaunay triangulation in texture space
		polyg2tri(delaunayPoints, sheetTriangulation);
		//project vertices back to 3D
		for (int i = 0; i < vertexMap.size(); i++) {
			sheetPoints.push_back(vertices[vertexMap[i]]);
		}
	}
}*/