/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_MinVolEllipse_api.h
 *
 * MATLAB Coder version            : 3.0
 * C/C++ source code generated on  : 27-Feb-2017 18:00:05
 */

#ifndef ___CODER_MINVOLELLIPSE_API_H__
#define ___CODER_MINVOLELLIPSE_API_H__

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_MinVolEllipse_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void MinVolEllipse(emxArray_real_T *P, real_T tolerance, real_T A[9],
  real_T c[3]);
extern void MinVolEllipse_api(const mxArray *prhs[2], const mxArray *plhs[2]);
extern void MinVolEllipse_atexit(void);
extern void MinVolEllipse_initialize(void);
extern void MinVolEllipse_terminate(void);
extern void MinVolEllipse_xil_terminate(void);

#endif

/*
 * File trailer for _coder_MinVolEllipse_api.h
 *
 * [EOF]
 */
