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


#ifndef STRUCT_ELLIPSOID
  #define STRUCT_ELLIPSOID

  struct Ellipsoid
  {
    std::vector<double> centre;
    std::vector< std::vector<double> > A;
    Ellipsoid ( std::vector<double> _centre, std::vector< std::vector<double> > _A) : centre(_centre), A(_A) {}
  };
#endif

Ellipsoid GetMinVolEllipsoid(std::vector< std::vector<double> > points, double tolerance);

#endif
