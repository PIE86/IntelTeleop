#ifndef ELLIPSOID_GEN
#define ELLIPSOID_GEN

#include "../lib/MinVolEllipse.h"

#ifndef STRUCT_POINT3D
#define STRUCT_POINT3D

struct Point3D
{
  double x;
  double y;
  double z;
  Point3D ( double _x, double _y, double _z ): x(_x), y(_y), z(_z) {}
  Point3D Add( Point3D p2) {return Point3D( x+p2.x, y+p2.y, z+p2.z );}
  Point3D Multipy ( double n) {}
  Point3D MidPoint( Point3D p2) {return Point3D(  (x+p2.x)/2, (y+p2.y)2, (z+p2.z)2 );}

};

#endif

void GetMinVollEllipsoid(std::vector<Point3D>, double tolerance);


#endif
