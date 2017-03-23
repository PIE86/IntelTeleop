#include <iostream>
#include <Eigen/Dense>

#include "ellipsoid_gen.h"

using namespace std;
using namespace Eigen;

// doesnt return anything but that's todo depending on other files
Ellipsoid GetMinVolEllipsoid(std::vector< vector<double> > points, double tolerance) {
  printf("get min volume ellipsoid...\n");
  int nb = points.size();
  double data[3][nb];
  for(int i = 0; i < nb; i++) {
    data[0][i] = points[i][0];
    data[1][i] = points[i][1];
    data[2][i] = points[i][2];
  }
  emxArray_real_T * temp = emxCreateWrapper_real_T(*data, 3, nb);

  printf("%i\n", temp->size[0]);
  printf("%i\n", temp->size[1]);

  // results
  double c[3];
  double A[9];

  vector< vector<double> > matrixA(3, vector<double>(3));
  vector<double> vectorC(3);

  // Call the entry-point 'MinVolEllipse'.
  MinVolEllipse(temp, tolerance, A, c);

  printf("c: ");
  for(int i=0; i<3; i++) {
    printf("%f ", c[i]);
    vectorC[i] = c[i];
  }
  printf("\n");
  printf("A: \n");
  for(int i=0; i<3; i++) {
    for(int j=0; j<3; j++) {
      printf("%f ", A[3*i+j]);
      matrixA[i][j] = A[3*i+j];
    }
  }
  printf("\n---\n");
  emxDestroyArray_real_T(temp);

  // get eigenvalues of A
  // Matrix3f B;
  // B << A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8], A[9];
  // cout << "B:\n" << B << endl;
  // SelfAdjointEigenSolver<Matrix3f> eigensolver(B);
  // if (eigensolver.info() != Success) abort();
  // cout << "Eigenvalues:\n" << eigensolver.eigenvalues() << endl;

  // vector<double> abc;
  // for( int i = 0; i < 3; i++ ) {
  //   double temp = 1 / sqrt( eigensolver.eigenvalues()[i] );
  //   printf("abc : %f\n", temp);
  //   abc.push_back( temp );

  // }

  return Ellipsoid(vectorC, matrixA);
}
