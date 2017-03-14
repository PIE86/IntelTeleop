//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MinVolEllipse.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Feb-2017 18:00:05
//

// Include Files
#include "rt_nonfinite.h"
#include "MinVolEllipse.h"
#include "MinVolEllipse_emxutil.h"
#include "norm.h"
#include "diag.h"
#include "inv.h"

// Function Definitions

//
// [A , c] = MinVolEllipse(P, tolerance)
//  Finds the minimum volume enclsing ellipsoid (MVEE) of a set of data
//  points stored in matrix P. The following optimization problem is solved:
//
//  minimize       log(det(A))
//  subject to     (P_i - c)' * A * (P_i - c) <= 1
//
//  in variables A and c, where P_i is the i-th column of the matrix P.
//  The solver is based on Khachiyan Algorithm, and the final solution
//  is different from the optimal value by the pre-spesified amount of 'tolerance'.
//
//  inputs:
// ---------
//  P : (d x N) dimnesional matrix containing N points in R^d.
//  tolerance : error in the solution with respect to the optimal value.
//
//  outputs:
// ---------
//  A : (d x d) matrix of the ellipse equation in the 'center form':
//  (x-c)' * A * (x-c) = 1
//  c : 'd' dimensional vector as the center of the ellipse.
//
//  example:
//  --------
//       P = rand(5,100);
//       [A, c] = MinVolEllipse(P, .01)
//
//       To reduce the computation time, work with the boundary points only:
//
//       K = convhulln(P');
//       K = unique(K(:));
//       Q = P(:,K);
//       [A, c] = MinVolEllipse(Q, .01)
//
//
//  Nima Moshtagh (nima@seas.upenn.edu)
//  University of Pennsylvania
//
//  December 2005
//  UPDATE: Jan 2009
// Arguments    : const emxArray_real_T *P
//                double tolerance
//                double A[9]
//                double c[3]
// Return Type  : void
//
void MinVolEllipse(const emxArray_real_T *P, double tolerance, double A[9],
                   double c[3])
{
  emxArray_real_T *Q;
  int i0;
  int ib;
  int br;
  int ixstart;
  emxArray_real_T *u;
  double err;
  double mtmp;
  emxArray_real_T *M;
  emxArray_real_T *U;
  emxArray_real_T *y;
  emxArray_real_T *b;
  emxArray_real_T *b_y;
  emxArray_real_T *c_y;
  emxArray_real_T *b_M;
  int n;
  int k;
  unsigned int unnamed_idx_1;
  int ic;
  int ar;
  int ia;
  double X[16];
  double b_b[16];
  unsigned int unnamed_idx_0;
  boolean_T exitg1;
  double step_size;
  emxArray_real_T *d_y;
  emxArray_real_T *c_b;
  double e_y[9];
  double f_y[3];
  double g_y[3];
  double h_y[9];
  double b_c;
  emxInit_real_T(&Q, 2);

  // %%%%%%%%%%%%%%%%%%%% Solving the Dual problem%%%%%%%%%%%%%%%%%%%%%%%%%%%5
  //  ---------------------------------
  //  data points
  //  -----------------------------------
  i0 = Q->size[0] * Q->size[1];
  Q->size[0] = 4;
  Q->size[1] = P->size[1];
  emxEnsureCapacity((emxArray__common *)Q, i0, (int)sizeof(double));
  ib = P->size[1] << 2;
  for (i0 = 0; i0 < ib; i0++) {
    Q->data[i0] = 0.0;
  }

  if (1 > P->size[1]) {
    ib = -1;
  } else {
    ib = P->size[1] - 1;
  }

  for (i0 = 0; i0 <= ib; i0++) {
    for (br = 0; br < 3; br++) {
      Q->data[br + Q->size[0] * i0] = P->data[br + P->size[0] * i0];
    }
  }

  ixstart = P->size[1];
  for (i0 = 0; i0 < ixstart; i0++) {
    Q->data[3 + Q->size[0] * i0] = 1.0;
  }

  emxInit_real_T1(&u, 1);

  //  initializations
  //  -----------------------------------
  err = 1.0;
  mtmp = 1.0 / (double)P->size[1];
  i0 = u->size[0];
  u->size[0] = P->size[1];
  emxEnsureCapacity((emxArray__common *)u, i0, (int)sizeof(double));
  ib = P->size[1];
  for (i0 = 0; i0 < ib; i0++) {
    u->data[i0] = mtmp;
  }

  //  1st iteration
  //  Khachiyan Algorithm
  //  -----------------------------------
  emxInit_real_T1(&M, 1);
  emxInit_real_T(&U, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&b, 2);
  emxInit_real_T(&b_y, 2);
  emxInit_real_T(&c_y, 2);
  emxInit_real_T1(&b_M, 1);
  while (err > tolerance) {
    diag(u, U);
    if ((Q->size[1] == 1) || (U->size[0] == 1)) {
      i0 = y->size[0] * y->size[1];
      y->size[0] = 4;
      y->size[1] = U->size[1];
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      for (i0 = 0; i0 < 4; i0++) {
        ib = U->size[1];
        for (br = 0; br < ib; br++) {
          y->data[i0 + y->size[0] * br] = 0.0;
          n = Q->size[1];
          for (ixstart = 0; ixstart < n; ixstart++) {
            y->data[i0 + y->size[0] * br] += Q->data[i0 + Q->size[0] * ixstart] *
              U->data[ixstart + U->size[0] * br];
          }
        }
      }
    } else {
      k = Q->size[1];
      unnamed_idx_1 = (unsigned int)U->size[1];
      i0 = y->size[0] * y->size[1];
      y->size[0] = 4;
      y->size[1] = (int)unnamed_idx_1;
      y->size[0] = 4;
      emxEnsureCapacity((emxArray__common *)y, i0, (int)sizeof(double));
      ib = y->size[1];
      for (i0 = 0; i0 < ib; i0++) {
        for (br = 0; br < 4; br++) {
          y->data[br + y->size[0] * i0] = 0.0;
        }
      }

      if (U->size[1] == 0) {
      } else {
        ixstart = (U->size[1] - 1) << 2;
        for (n = 0; n <= ixstart; n += 4) {
          for (ic = n; ic + 1 <= n + 4; ic++) {
            y->data[ic] = 0.0;
          }
        }

        br = 0;
        for (n = 0; n <= ixstart; n += 4) {
          ar = 0;
          i0 = br + k;
          for (ib = br; ib + 1 <= i0; ib++) {
            if (U->data[ib] != 0.0) {
              ia = ar;
              for (ic = n; ic + 1 <= n + 4; ic++) {
                ia++;
                y->data[ic] += U->data[ib] * Q->data[ia - 1];
              }
            }

            ar += 4;
          }

          br += k;
        }
      }
    }

    i0 = b->size[0] * b->size[1];
    b->size[0] = Q->size[1];
    b->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)b, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 4; i0++) {
      ib = Q->size[1];
      for (br = 0; br < ib; br++) {
        b->data[br + b->size[0] * i0] = Q->data[i0 + Q->size[0] * br];
      }
    }

    if ((y->size[1] == 1) || (b->size[0] == 1)) {
      for (i0 = 0; i0 < 4; i0++) {
        for (br = 0; br < 4; br++) {
          X[i0 + (br << 2)] = 0.0;
          ib = y->size[1];
          for (ixstart = 0; ixstart < ib; ixstart++) {
            X[i0 + (br << 2)] += y->data[i0 + y->size[0] * ixstart] * b->
              data[ixstart + b->size[0] * br];
          }
        }
      }
    } else {
      k = y->size[1];
      memset(&X[0], 0, sizeof(double) << 4);
      for (n = 0; n <= 13; n += 4) {
        for (ic = n; ic + 1 <= n + 4; ic++) {
          X[ic] = 0.0;
        }
      }

      br = 0;
      for (n = 0; n <= 13; n += 4) {
        ar = 0;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (b->data[ib] != 0.0) {
            ia = ar;
            for (ic = n; ic + 1 <= n + 4; ic++) {
              ia++;
              X[ic] += b->data[ib] * y->data[ia - 1];
            }
          }

          ar += 4;
        }

        br += k;
      }
    }

    //  X = \sum_i ( u_i * q_i * q_i')  is a (d+1)x(d+1) matrix
    i0 = b->size[0] * b->size[1];
    b->size[0] = Q->size[1];
    b->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)b, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 4; i0++) {
      ib = Q->size[1];
      for (br = 0; br < ib; br++) {
        b->data[br + b->size[0] * i0] = Q->data[i0 + Q->size[0] * br];
      }
    }

    inv(X, b_b);
    unnamed_idx_0 = (unsigned int)b->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[0] = (int)unnamed_idx_0;
    b_y->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    k = b->size[0];
    i0 = b_y->size[0] * b_y->size[1];
    b_y->size[1] = 4;
    emxEnsureCapacity((emxArray__common *)b_y, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 4; i0++) {
      ib = b_y->size[0];
      for (br = 0; br < ib; br++) {
        b_y->data[br + b_y->size[0] * i0] = 0.0;
      }
    }

    if (b->size[0] == 0) {
    } else {
      ixstart = b->size[0] * 3;
      n = 0;
      while ((k > 0) && (n <= ixstart)) {
        i0 = n + k;
        for (ic = n; ic + 1 <= i0; ic++) {
          b_y->data[ic] = 0.0;
        }

        n += k;
      }

      br = 0;
      n = 0;
      while ((k > 0) && (n <= ixstart)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 4; ib++) {
          if (b_b[ib] != 0.0) {
            ia = ar;
            i0 = n + k;
            for (ic = n; ic + 1 <= i0; ic++) {
              ia++;
              b_y->data[ic] += b_b[ib] * b->data[ia - 1];
            }
          }

          ar += k;
        }

        br += 4;
        n += k;
      }
    }

    unnamed_idx_0 = (unsigned int)b_y->size[0];
    unnamed_idx_1 = (unsigned int)Q->size[1];
    i0 = c_y->size[0] * c_y->size[1];
    c_y->size[0] = (int)unnamed_idx_0;
    c_y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)c_y, i0, (int)sizeof(double));
    k = b_y->size[0];
    i0 = c_y->size[0] * c_y->size[1];
    emxEnsureCapacity((emxArray__common *)c_y, i0, (int)sizeof(double));
    ib = c_y->size[1];
    for (i0 = 0; i0 < ib; i0++) {
      n = c_y->size[0];
      for (br = 0; br < n; br++) {
        c_y->data[br + c_y->size[0] * i0] = 0.0;
      }
    }

    if ((b_y->size[0] == 0) || (Q->size[1] == 0)) {
    } else {
      ixstart = b_y->size[0] * (Q->size[1] - 1);
      n = 0;
      while ((k > 0) && (n <= ixstart)) {
        i0 = n + k;
        for (ic = n; ic + 1 <= i0; ic++) {
          c_y->data[ic] = 0.0;
        }

        n += k;
      }

      br = 0;
      n = 0;
      while ((k > 0) && (n <= ixstart)) {
        ar = 0;
        for (ib = br; ib + 1 <= br + 4; ib++) {
          if (Q->data[ib] != 0.0) {
            ia = ar;
            i0 = n + k;
            for (ic = n; ic + 1 <= i0; ic++) {
              ia++;
              c_y->data[ic] += Q->data[ib] * b_y->data[ia - 1];
            }
          }

          ar += k;
        }

        br += 4;
        n += k;
      }
    }

    b_diag(c_y, M);

    //  M the diagonal vector of an NxN matrix
    ixstart = 1;
    n = M->size[0];
    mtmp = M->data[0];
    ar = 0;
    if (M->size[0] > 1) {
      if (rtIsNaN(M->data[0])) {
        br = 1;
        exitg1 = false;
        while ((!exitg1) && (br + 1 <= n)) {
          ixstart = br + 1;
          if (!rtIsNaN(M->data[br])) {
            mtmp = M->data[br];
            ar = br;
            exitg1 = true;
          } else {
            br++;
          }
        }
      }

      if (ixstart < M->size[0]) {
        while (ixstart + 1 <= n) {
          if (M->data[ixstart] > mtmp) {
            mtmp = M->data[ixstart];
            ar = ixstart;
          }

          ixstart++;
        }
      }
    }

    step_size = ((mtmp - 3.0) - 1.0) / (4.0 * (mtmp - 1.0));
    i0 = M->size[0];
    M->size[0] = u->size[0];
    emxEnsureCapacity((emxArray__common *)M, i0, (int)sizeof(double));
    ib = u->size[0];
    for (i0 = 0; i0 < ib; i0++) {
      M->data[i0] = (1.0 - step_size) * u->data[i0];
    }

    M->data[ar] += step_size;
    i0 = b_M->size[0];
    b_M->size[0] = M->size[0];
    emxEnsureCapacity((emxArray__common *)b_M, i0, (int)sizeof(double));
    ib = M->size[0];
    for (i0 = 0; i0 < ib; i0++) {
      b_M->data[i0] = M->data[i0] - u->data[i0];
    }

    err = norm(b_M);
    i0 = u->size[0];
    u->size[0] = M->size[0];
    emxEnsureCapacity((emxArray__common *)u, i0, (int)sizeof(double));
    ib = M->size[0];
    for (i0 = 0; i0 < ib; i0++) {
      u->data[i0] = M->data[i0];
    }
  }

  emxFree_real_T(&b_M);
  emxFree_real_T(&c_y);
  emxFree_real_T(&b_y);
  emxFree_real_T(&b);
  emxFree_real_T(&y);
  emxFree_real_T(&M);
  emxFree_real_T(&Q);

  // %%%%%%%%%%%%%%%%%% Computing the Ellipse parameters%%%%%%%%%%%%%%%%%%%%%%
  //  Finds the ellipse equation in the 'center form':
  //  (x-c)' * A * (x-c) = 1
  //  It computes a dxd matrix 'A' and a d dimensional vector 'c' as the center
  //  of the ellipse.
  diag(u, U);

  //  the A matrix for the ellipse
  //  --------------------------------------------
  emxInit_real_T(&d_y, 2);
  if ((P->size[1] == 1) || (U->size[0] == 1)) {
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = 3;
    d_y->size[1] = U->size[1];
    emxEnsureCapacity((emxArray__common *)d_y, i0, (int)sizeof(double));
    for (i0 = 0; i0 < 3; i0++) {
      ib = U->size[1];
      for (br = 0; br < ib; br++) {
        d_y->data[i0 + d_y->size[0] * br] = 0.0;
        n = P->size[1];
        for (ixstart = 0; ixstart < n; ixstart++) {
          d_y->data[i0 + d_y->size[0] * br] += P->data[i0 + P->size[0] * ixstart]
            * U->data[ixstart + U->size[0] * br];
        }
      }
    }
  } else {
    k = P->size[1];
    unnamed_idx_1 = (unsigned int)U->size[1];
    i0 = d_y->size[0] * d_y->size[1];
    d_y->size[0] = 3;
    d_y->size[1] = (int)unnamed_idx_1;
    d_y->size[0] = 3;
    emxEnsureCapacity((emxArray__common *)d_y, i0, (int)sizeof(double));
    ib = d_y->size[1];
    for (i0 = 0; i0 < ib; i0++) {
      for (br = 0; br < 3; br++) {
        d_y->data[br + d_y->size[0] * i0] = 0.0;
      }
    }

    if (U->size[1] == 0) {
    } else {
      ixstart = 3 * (U->size[1] - 1);
      for (n = 0; n <= ixstart; n += 3) {
        for (ic = n; ic + 1 <= n + 3; ic++) {
          d_y->data[ic] = 0.0;
        }
      }

      br = 0;
      for (n = 0; n <= ixstart; n += 3) {
        ar = 0;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (U->data[ib] != 0.0) {
            ia = ar;
            for (ic = n; ic + 1 <= n + 3; ic++) {
              ia++;
              d_y->data[ic] += U->data[ib] * P->data[ia - 1];
            }
          }

          ar += 3;
        }

        br += k;
      }
    }
  }

  emxFree_real_T(&U);
  emxInit_real_T(&c_b, 2);
  i0 = c_b->size[0] * c_b->size[1];
  c_b->size[0] = P->size[1];
  c_b->size[1] = 3;
  emxEnsureCapacity((emxArray__common *)c_b, i0, (int)sizeof(double));
  for (i0 = 0; i0 < 3; i0++) {
    ib = P->size[1];
    for (br = 0; br < ib; br++) {
      c_b->data[br + c_b->size[0] * i0] = P->data[i0 + P->size[0] * br];
    }
  }

  if ((d_y->size[1] == 1) || (c_b->size[0] == 1)) {
    for (i0 = 0; i0 < 3; i0++) {
      for (br = 0; br < 3; br++) {
        e_y[i0 + 3 * br] = 0.0;
        ib = d_y->size[1];
        for (ixstart = 0; ixstart < ib; ixstart++) {
          e_y[i0 + 3 * br] += d_y->data[i0 + d_y->size[0] * ixstart] * c_b->
            data[ixstart + c_b->size[0] * br];
        }
      }
    }
  } else {
    k = d_y->size[1];
    memset(&e_y[0], 0, 9U * sizeof(double));
    for (n = 0; n <= 7; n += 3) {
      for (ic = n; ic + 1 <= n + 3; ic++) {
        e_y[ic] = 0.0;
      }
    }

    br = 0;
    for (n = 0; n <= 7; n += 3) {
      ar = 0;
      i0 = br + k;
      for (ib = br; ib + 1 <= i0; ib++) {
        if (c_b->data[ib] != 0.0) {
          ia = ar;
          for (ic = n; ic + 1 <= n + 3; ic++) {
            ia++;
            e_y[ic] += c_b->data[ib] * d_y->data[ia - 1];
          }
        }

        ar += 3;
      }

      br += k;
    }
  }

  emxFree_real_T(&c_b);
  emxFree_real_T(&d_y);
  if ((P->size[1] == 1) || (u->size[0] == 1)) {
    for (i0 = 0; i0 < 3; i0++) {
      f_y[i0] = 0.0;
      ib = P->size[1];
      for (br = 0; br < ib; br++) {
        mtmp = f_y[i0] + P->data[i0 + P->size[0] * br] * u->data[br];
        f_y[i0] = mtmp;
      }
    }
  } else {
    for (ic = 0; ic < 3; ic++) {
      f_y[ic] = 0.0;
    }

    ar = 0;
    for (ib = 0; ib + 1 <= P->size[1]; ib++) {
      if (u->data[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 3; ic++) {
          ia++;
          mtmp = f_y[ic] + u->data[ib] * P->data[ia - 1];
          f_y[ic] = mtmp;
        }
      }

      ar += 3;
    }
  }

  if ((P->size[1] == 1) || (u->size[0] == 1)) {
    for (i0 = 0; i0 < 3; i0++) {
      g_y[i0] = 0.0;
      ib = P->size[1];
      for (br = 0; br < ib; br++) {
        mtmp = g_y[i0] + P->data[i0 + P->size[0] * br] * u->data[br];
        g_y[i0] = mtmp;
      }
    }
  } else {
    for (ic = 0; ic < 3; ic++) {
      g_y[ic] = 0.0;
    }

    ar = 0;
    for (ib = 0; ib + 1 <= P->size[1]; ib++) {
      if (u->data[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 3; ic++) {
          ia++;
          mtmp = g_y[ic] + u->data[ib] * P->data[ia - 1];
          g_y[ic] = mtmp;
        }
      }

      ar += 3;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (br = 0; br < 3; br++) {
      h_y[i0 + 3 * br] = e_y[i0 + 3 * br] - f_y[i0] * g_y[br];
    }
  }

  b_inv(h_y, A);
  for (i0 = 0; i0 < 9; i0++) {
    A[i0] *= 0.33333333333333331;
  }

  //  center of the ellipse
  //  --------------------------------------------
  if ((P->size[1] == 1) || (u->size[0] == 1)) {
    for (i0 = 0; i0 < 3; i0++) {
      c[i0] = 0.0;
      ib = P->size[1];
      for (br = 0; br < ib; br++) {
        b_c = c[i0] + P->data[i0 + P->size[0] * br] * u->data[br];
        c[i0] = b_c;
      }
    }
  } else {
    for (ic = 0; ic < 3; ic++) {
      c[ic] = 0.0;
    }

    ar = 0;
    for (ib = 0; ib + 1 <= P->size[1]; ib++) {
      if (u->data[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic < 3; ic++) {
          ia++;
          b_c = c[ic] + u->data[ib] * P->data[ia - 1];
          c[ic] = b_c;
        }
      }

      ar += 3;
    }
  }

  emxFree_real_T(&u);
}

//
// File trailer for MinVolEllipse.cpp
//
// [EOF]
//
