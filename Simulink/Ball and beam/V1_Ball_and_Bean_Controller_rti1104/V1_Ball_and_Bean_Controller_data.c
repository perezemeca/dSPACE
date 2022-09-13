/*
 * V1_Ball_and_Bean_Controller_data.c
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
#include "V1_Ball_and_Bean_Controller.h"
#include "V1_Ball_and_Bean_Controller_private.h"

/* Block parameters (auto storage) */
P_V1_Ball_and_Bean_Controller_T V1_Ball_and_Bean_Controller_P = {
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Constant'
                                        */
  0.001,                               /* Expression: 0.001
                                        * Referenced by: '<Root>/Pulse HC-SR04'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Stop PWM 1..4'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Integrator'
                                        */
  0.1,                                 /* Expression: 0.1
                                        * Referenced by: '<Root>/Ki'
                                        */
  0.254,                               /* Expression: 0.254
                                        * Referenced by: '<Root>/Set Point Sensor'
                                        */
  -5.8,                                /* Computed Parameter: TransferFcn_A
                                        * Referenced by: '<Root>/Transfer Fcn'
                                        */
  7.8,                                 /* Computed Parameter: TransferFcn_C
                                        * Referenced by: '<Root>/Transfer Fcn'
                                        */
  0.083333333333333329,                /* Expression: 0.0095/0.114
                                        * Referenced by: '<Root>/Gain'
                                        */
  -5.8,                                /* Computed Parameter: Derivativefilter_A
                                        * Referenced by: '<Root>/Derivative filter'
                                        */
  -33.64,                              /* Computed Parameter: Derivativefilter_C
                                        * Referenced by: '<Root>/Derivative filter'
                                        */
  5.8,                                 /* Computed Parameter: Derivativefilter_D
                                        * Referenced by: '<Root>/Derivative filter'
                                        */
  0.2,                                 /* Expression: 0.2
                                        * Referenced by: '<Root>/Kd'
                                        */
  0.5,                                 /* Expression: 0.5
                                        * Referenced by: '<Root>/Kp'
                                        */
  0.0522,                              /* Expression: 0.0522
                                        * Referenced by: '<Root>/Servo Beam'
                                        */
  0.0617,                              /* Expression: 0.0617
                                        * Referenced by: '<Root>/Saturation'
                                        */
  0.0427,                              /* Expression: 0.0427
                                        * Referenced by: '<Root>/Saturation'
                                        */
  0.0775,                              /* Expression: 0.0775
                                        * Referenced by: '<Root>/Servo Base'
                                        */
  0.0,                                 /* Expression: 0
                                        * Referenced by: '<Root>/Duty cicle PWM 4'
                                        */
  1.0                                  /* Expression: 1
                                        * Referenced by: '<Root>/Stop PWM 4'
                                        */
};
