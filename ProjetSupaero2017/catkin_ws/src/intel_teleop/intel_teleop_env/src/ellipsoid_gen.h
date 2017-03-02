#ifndef ELLIPSOID_GEN
#define ELLIPSOID_GEN

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <sstream>

#include "../lib/MinVolEllipse_emxAPI.h"
#include "../lib/MinVolEllipse.h"
#include "../lib/rtGetInf.h"
#include "../lib/rtGetNaN.h"
#include "../lib/rt_nonfinite.h"
#include "../lib/rtwtypes.h"


#ifndef STRUCT_POINT3D
  #define STRUCT_POINT3D

  struct Point3D
  {
    double x;
    double y;
    double z;
    Point3D (): x(0.0), y(0.0), z(0.0) {}
    Point3D ( double _x, double _y, double _z ): x(_x), y(_y), z(_z) {}

  };
#endif

bool GetMinVollEllipsoid(std::vector<Point3D>, double tolerance);

#endif
