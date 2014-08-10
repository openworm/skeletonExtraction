#ifndef Delaunay_H
#define Delaunay_H

#include <iostream>
#include <stdlib.h> // for C qsort 
#include <cmath>
#include <time.h> // for random

const double EPSILON = 0.000001;

struct ITRIANGLE{
  int p1, p2, p3;
};

struct IEDGE{
  int p1, p2;
};

struct XYZ{
  double x, y, z;
};

void outputtriangle(int &nv, XYZ p[], ITRIANGLE v[], int &ntri);
int XYZCompare(const void *v1, const void *v2);
int Triangulate(int nv, XYZ pxyz[], ITRIANGLE v[], int &ntri);
int CircumCircle(double, double, double, double, double, double, double, 
double, double&, double&, double&);

#endif


