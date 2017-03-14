//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: diag.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Feb-2017 18:00:05
//

// Include Files
#include "rt_nonfinite.h"
#include "MinVolEllipse.h"
#include "diag.h"
#include "MinVolEllipse_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *v
//                emxArray_real_T *d
// Return Type  : void
//
void b_diag(const emxArray_real_T *v, emxArray_real_T *d)
{
  int j;
  int dlen;
  int stride;
  if ((v->size[0] == 1) && (v->size[1] == 1)) {
    j = d->size[0];
    d->size[0] = 1;
    emxEnsureCapacity((emxArray__common *)d, j, (int)sizeof(double));
    d->data[0] = v->data[0];
  } else {
    if (0 < v->size[1]) {
      if (v->size[0] <= v->size[1]) {
        dlen = v->size[0];
      } else {
        dlen = v->size[1];
      }

      stride = v->size[0] + 1;
    } else {
      dlen = 0;
      stride = 0;
    }

    j = d->size[0];
    d->size[0] = dlen;
    emxEnsureCapacity((emxArray__common *)d, j, (int)sizeof(double));
    for (j = 0; j + 1 <= dlen; j++) {
      d->data[j] = v->data[j * stride];
    }
  }
}

//
// Arguments    : const emxArray_real_T *v
//                emxArray_real_T *d
// Return Type  : void
//
void diag(const emxArray_real_T *v, emxArray_real_T *d)
{
  int j;
  int unnamed_idx_1;
  int i1;
  j = v->size[0];
  unnamed_idx_1 = v->size[0];
  i1 = d->size[0] * d->size[1];
  d->size[0] = j;
  d->size[1] = unnamed_idx_1;
  emxEnsureCapacity((emxArray__common *)d, i1, (int)sizeof(double));
  j *= unnamed_idx_1;
  for (i1 = 0; i1 < j; i1++) {
    d->data[i1] = 0.0;
  }

  for (j = 0; j + 1 <= v->size[0]; j++) {
    d->data[j + d->size[0] * j] = v->data[j];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
