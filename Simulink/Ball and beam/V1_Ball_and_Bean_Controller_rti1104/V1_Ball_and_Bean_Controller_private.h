/*
 * V1_Ball_and_Bean_Controller_private.h
 *
 * Code generation for model "V1_Ball_and_Bean_Controller".
 *
 * Model version              : 1.100
 * Simulink Coder version : 8.6 (R2014a) 27-Dec-2013
 * C source code generated on : Tue Sep 13 12:08:08 2022
 *
 * Target selection: rti1104.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */
#ifndef RTW_HEADER_V1_Ball_and_Bean_Controller_private_h_
#define RTW_HEADER_V1_Ball_and_Bean_Controller_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#endif                                 /* __RTWTYPES_H__ */

extern int slaveDSPPwmStopFlagCh1;
extern int slaveDSPPwmStopFlagCh2;
extern int slaveDSPPwmStopFlagCh3;
extern int slaveDSPPwmStopFlagCh4;

/* private model entry point functions */
extern void V1_Ball_and_Bean_Controller_derivatives(void);

#endif                                 /* RTW_HEADER_V1_Ball_and_Bean_Controller_private_h_ */
