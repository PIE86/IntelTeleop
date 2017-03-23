//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MinVolEllipse_emxAPI.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Feb-2017 18:00:05
//
#ifndef __MINVOLELLIPSE_EMXAPI_H__
#define __MINVOLELLIPSE_EMXAPI_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "MinVolEllipse_types.h"

// Function Declarations
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for MinVolEllipse_emxAPI.h
//
// [EOF]
//
