#include "ellipsoid_gen.h"

using namespace std;

// doesnt return anything but that's todo depending on other files
bool GetMinVollEllipsoid(std::vector<Point3D> points, double tolerance) {
  printf("get min volume ellipsoid...\n");
  int nb = points.size();
  double data[3][nb];
  for(int i = 0; i < nb; i++) {
    data[0][i] = points[i].x;
    data[1][i] = points[i].y;
    data[2][i] = points[i].z;
  }
  emxArray_real_T * temp = emxCreateWrapper_real_T(*data, 3, nb);

  printf("%i\n", temp->size[0]);
  printf("%i\n", temp->size[1]);

  // results
  double c[3];
  double A[9];

  // Call the entry-point 'MinVolEllipse'.
  MinVolEllipse(temp, tolerance, A, c);

  printf("c: ");
  for(int i=0; i<3; i++) {
    printf("%f ", c[i]);
  }
  printf("\n");
  printf("A: \n");
  for(int i=0; i<9; i++) {
    printf("%f ", A[i]);
  }
  printf("\n---\n");
  emxDestroyArray_real_T(temp);

  return 1;
}
