//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 27-Feb-2017 18:00:05
//
#ifndef __INV_H__
#define __INV_H__

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "MinVolEllipse_types.h"

// Function Declarations
extern void b_inv(const double x[9], double y[9]);
extern void inv(const double x[16], double y[16]);

#endif

//
// File trailer for inv.h
//
// [EOF]
//
