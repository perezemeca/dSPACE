/*
 * V1_Ball_and_Bean_Controller.c
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
#include "V1_Ball_and_Bean_Controller_trc_ptr.h"
#include "V1_Ball_and_Bean_Controller.h"
#include "V1_Ball_and_Bean_Controller_private.h"

/* Block signals (auto storage) */
B_V1_Ball_and_Bean_Controller_T V1_Ball_and_Bean_Controller_B;

/* Continuous states */
X_V1_Ball_and_Bean_Controller_T V1_Ball_and_Bean_Controller_X;

/* Block states (auto storage) */
DW_V1_Ball_and_Bean_Controlle_T V1_Ball_and_Bean_Controller_DW;

/* Real-time model */
RT_MODEL_V1_Ball_and_Bean_Con_T V1_Ball_and_Bean_Controller_M_;
RT_MODEL_V1_Ball_and_Bean_Con_T *const V1_Ball_and_Bean_Controller_M =
  &V1_Ball_and_Bean_Controller_M_;

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  V1_Ball_and_Bean_Controller_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  V1_Ball_and_Bean_Controller_output();
  V1_Ball_and_Bean_Controller_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  V1_Ball_and_Bean_Controller_output();
  V1_Ball_and_Bean_Controller_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  V1_Ball_and_Bean_Controller_output();
  V1_Ball_and_Bean_Controller_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model output function */
void V1_Ball_and_Bean_Controller_output(void)
{
  real_T u0;
  real_T u1;
  real_T u2;
  if (rtmIsMajorTimeStep(V1_Ball_and_Bean_Controller_M)) {
    /* set solver stop time */
    if (!(V1_Ball_and_Bean_Controller_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&V1_Ball_and_Bean_Controller_M->solverInfo,
                            ((V1_Ball_and_Bean_Controller_M->Timing.clockTickH0
        + 1) * V1_Ball_and_Bean_Controller_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&V1_Ball_and_Bean_Controller_M->solverInfo,
                            ((V1_Ball_and_Bean_Controller_M->Timing.clockTick0 +
        1) * V1_Ball_and_Bean_Controller_M->Timing.stepSize0 +
        V1_Ball_and_Bean_Controller_M->Timing.clockTickH0 *
        V1_Ball_and_Bean_Controller_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(V1_Ball_and_Bean_Controller_M)) {
    V1_Ball_and_Bean_Controller_M->Timing.t[0] = rtsiGetT
      (&V1_Ball_and_Bean_Controller_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(V1_Ball_and_Bean_Controller_M)) {
    /* Constant: '<Root>/Constant' */
    V1_Ball_and_Bean_Controller_B.Constant =
      V1_Ball_and_Bean_Controller_P.Constant_Value;

    /* S-Function (rti_commonblock): '<S1>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
    V1_Ball_and_Bean_Controller_DW.SFunction1_IWORK[0] =
      V1_Ball_and_Bean_Controller_P.StopPWM14_Value;

    /* write the duty cycle down */
    ds1104_slave_dsp_pwm_duty_write(0, rti_slv1104_fcn_index[6],
      V1_Ball_and_Bean_Controller_P.PulseHCSR04_Value);

    /* set outputs to TTL-level or retrigger PWM generation */
    if (V1_Ball_and_Bean_Controller_DW.SFunction1_IWORK[0] == 1 ) {
      /*  if (flag == RUN)||(flag == UNDEF) */
      if ((slaveDSPPwmStopFlagCh1 == 1)||(slaveDSPPwmStopFlagCh1 == 2))/* PWM Stop signal from Input Port -> set outputs to TTL-level */
      {
        slaveDSPPwmStopFlagCh1 = 0;
        ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH1_MSK,
          SLVDSP1104_PWM_TTL_LOW);
      }
    } else if (V1_Ball_and_Bean_Controller_DW.SFunction1_IWORK[0] == 0 ) {
      /* PWM Stop signal disabled -> trigger PWM generation once */
      if ((slaveDSPPwmStopFlagCh1 == 0)||(slaveDSPPwmStopFlagCh1 == 2)) {
        slaveDSPPwmStopFlagCh1 = 1;
        ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH1_MSK);
      }
    }
  }

  /* Integrator: '<Root>/Integrator' */
  V1_Ball_and_Bean_Controller_B.Integrator =
    V1_Ball_and_Bean_Controller_X.Integrator_CSTATE;

  /* Gain: '<Root>/Ki' */
  V1_Ball_and_Bean_Controller_B.Ki = V1_Ball_and_Bean_Controller_P.Ki_Gain *
    V1_Ball_and_Bean_Controller_B.Integrator;

  /* TransferFcn: '<Root>/Transfer Fcn' */
  V1_Ball_and_Bean_Controller_B.TransferFcn = 0.0;
  V1_Ball_and_Bean_Controller_B.TransferFcn +=
    V1_Ball_and_Bean_Controller_P.TransferFcn_C *
    V1_Ball_and_Bean_Controller_X.TransferFcn_CSTATE;

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Set Point Sensor'
   */
  V1_Ball_and_Bean_Controller_B.Sum =
    V1_Ball_and_Bean_Controller_P.SetPointSensor_Value -
    V1_Ball_and_Bean_Controller_B.TransferFcn;

  /* Gain: '<Root>/Gain' */
  V1_Ball_and_Bean_Controller_B.Gain = V1_Ball_and_Bean_Controller_P.Gain_Gain *
    V1_Ball_and_Bean_Controller_B.Sum;

  /* TransferFcn: '<Root>/Derivative filter' */
  V1_Ball_and_Bean_Controller_B.Derivativefilter = 0.0;
  V1_Ball_and_Bean_Controller_B.Derivativefilter +=
    V1_Ball_and_Bean_Controller_P.Derivativefilter_C *
    V1_Ball_and_Bean_Controller_X.Derivativefilter_CSTATE;
  V1_Ball_and_Bean_Controller_B.Derivativefilter +=
    V1_Ball_and_Bean_Controller_P.Derivativefilter_D *
    V1_Ball_and_Bean_Controller_B.Gain;

  /* Gain: '<Root>/Kd' */
  V1_Ball_and_Bean_Controller_B.Kd = V1_Ball_and_Bean_Controller_P.Kd_Gain *
    V1_Ball_and_Bean_Controller_B.Derivativefilter;

  /* Gain: '<Root>/Kp' */
  V1_Ball_and_Bean_Controller_B.Kp = V1_Ball_and_Bean_Controller_P.Kp_Gain *
    V1_Ball_and_Bean_Controller_B.Gain;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Servo Beam'
   */
  V1_Ball_and_Bean_Controller_B.Sum1 =
    ((V1_Ball_and_Bean_Controller_P.ServoBeam_Value -
      V1_Ball_and_Bean_Controller_B.Ki) - V1_Ball_and_Bean_Controller_B.Kd) -
    V1_Ball_and_Bean_Controller_B.Kp;

  /* Saturate: '<Root>/Saturation' */
  u0 = V1_Ball_and_Bean_Controller_B.Sum1;
  u1 = V1_Ball_and_Bean_Controller_P.Saturation_LowerSat;
  u2 = V1_Ball_and_Bean_Controller_P.Saturation_UpperSat;
  if (u0 > u2) {
    V1_Ball_and_Bean_Controller_B.Saturation = u2;
  } else if (u0 < u1) {
    V1_Ball_and_Bean_Controller_B.Saturation = u1;
  } else {
    V1_Ball_and_Bean_Controller_B.Saturation = u0;
  }

  /* End of Saturate: '<Root>/Saturation' */
  if (rtmIsMajorTimeStep(V1_Ball_and_Bean_Controller_M)) {
    /* S-Function (rti_commonblock): '<S1>/S-Function2' */
    /* This comment workarounds a code generation problem */

    /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
    V1_Ball_and_Bean_Controller_DW.SFunction2_IWORK[1] =
      V1_Ball_and_Bean_Controller_P.StopPWM14_Value;

    /* write the duty cycle down */
    ds1104_slave_dsp_pwm_duty_write(0, rti_slv1104_fcn_index[7],
      V1_Ball_and_Bean_Controller_B.Saturation);

    /* set outputs to TTL-level or retrigger PWM generation */
    if (V1_Ball_and_Bean_Controller_DW.SFunction2_IWORK[1] == 1 ) {
      /*  if (flag == RUN)||(flag == UNDEF) */
      if ((slaveDSPPwmStopFlagCh2 == 1)||(slaveDSPPwmStopFlagCh2 == 2))/* PWM Stop signal from Input Port -> set outputs to TTL-level */
      {
        slaveDSPPwmStopFlagCh2 = 0;
        ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH2_MSK,
          SLVDSP1104_PWM_TTL_LOW);
      }
    } else if (V1_Ball_and_Bean_Controller_DW.SFunction2_IWORK[1] == 0 ) {
      /* PWM Stop signal disabled -> trigger PWM generation once */
      if ((slaveDSPPwmStopFlagCh2 == 0)||(slaveDSPPwmStopFlagCh2 == 2)) {
        slaveDSPPwmStopFlagCh2 = 1;
        ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH2_MSK);
      }
    }

    /* S-Function (rti_commonblock): '<S1>/S-Function3' */
    /* This comment workarounds a code generation problem */

    /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
    V1_Ball_and_Bean_Controller_DW.SFunction3_IWORK[2] =
      V1_Ball_and_Bean_Controller_P.StopPWM14_Value;

    /* write the duty cycle down */
    ds1104_slave_dsp_pwm_duty_write(0, rti_slv1104_fcn_index[8],
      V1_Ball_and_Bean_Controller_P.ServoBase_Value);

    /* set outputs to TTL-level or retrigger PWM generation */
    if (V1_Ball_and_Bean_Controller_DW.SFunction3_IWORK[2] == 1 ) {
      /*  if (flag == RUN)||(flag == UNDEF) */
      if ((slaveDSPPwmStopFlagCh3 == 1)||(slaveDSPPwmStopFlagCh3 == 2))/* PWM Stop signal from Input Port -> set outputs to TTL-level */
      {
        slaveDSPPwmStopFlagCh3 = 0;
        ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH3_MSK,
          SLVDSP1104_PWM_TTL_LOW);
      }
    } else if (V1_Ball_and_Bean_Controller_DW.SFunction3_IWORK[2] == 0 ) {
      /* PWM Stop signal disabled -> trigger PWM generation once */
      if ((slaveDSPPwmStopFlagCh3 == 0)||(slaveDSPPwmStopFlagCh3 == 2)) {
        slaveDSPPwmStopFlagCh3 = 1;
        ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH3_MSK);
      }
    }

    /* S-Function (rti_commonblock): '<S1>/S-Function4' */
    /* This comment workarounds a code generation problem */

    /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
    V1_Ball_and_Bean_Controller_DW.SFunction4_IWORK[3] =
      V1_Ball_and_Bean_Controller_P.StopPWM4_Value;

    /* write the duty cycle down */
    ds1104_slave_dsp_pwm_duty_write(0, rti_slv1104_fcn_index[9],
      V1_Ball_and_Bean_Controller_P.DutyciclePWM4_Value);

    /* set outputs to TTL-level or retrigger PWM generation */
    if (V1_Ball_and_Bean_Controller_DW.SFunction4_IWORK[3] == 1 ) {
      /*  if (flag == RUN)||(flag == UNDEF) */
      if ((slaveDSPPwmStopFlagCh4 == 1)||(slaveDSPPwmStopFlagCh4 == 2))/* PWM Stop signal from Input Port -> set outputs to TTL-level */
      {
        slaveDSPPwmStopFlagCh4 = 0;
        ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH4_MSK,
          SLVDSP1104_PWM_TTL_LOW);
      }
    } else if (V1_Ball_and_Bean_Controller_DW.SFunction4_IWORK[3] == 0 ) {
      /* PWM Stop signal disabled -> trigger PWM generation once */
      if ((slaveDSPPwmStopFlagCh4 == 0)||(slaveDSPPwmStopFlagCh4 == 2)) {
        slaveDSPPwmStopFlagCh4 = 1;
        ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH4_MSK);
      }
    }

    /* S-Function (rti_commonblock): '<S2>/S-Function1' */
    /* This comment workarounds a code generation problem */

    /* dSPACE I/O Board DS1104 #1 Unit:PWM2D */
    {
      UInt16 pwm2d_ch1stat;
      ds1104_slave_dsp_pwm2d_read(2, rti_slv1104_fcn_index[19],
        &V1_Ball_and_Bean_Controller_B.SFunction1[1],
        &V1_Ball_and_Bean_Controller_B.SFunction1[0], &pwm2d_ch1stat);
      if (V1_Ball_and_Bean_Controller_B.SFunction1[1] != 0.0) {
        V1_Ball_and_Bean_Controller_B.SFunction1[1] = 1.0/
          V1_Ball_and_Bean_Controller_B.SFunction1[1];
      } else {
        V1_Ball_and_Bean_Controller_B.SFunction1[1] = 0.0;
      }
    }

    /* S-Function (rti_commonblock): '<S2>/S-Function2' */
    /* This comment workarounds a code generation problem */

    /* S-Function (rti_commonblock): '<S2>/S-Function3' */
    /* This comment workarounds a code generation problem */

    /* S-Function (rti_commonblock): '<S2>/S-Function4' */
    /* This comment workarounds a code generation problem */
  }
}

/* Model update function */
void V1_Ball_and_Bean_Controller_update(void)
{
  if (rtmIsMajorTimeStep(V1_Ball_and_Bean_Controller_M)) {
    rt_ertODEUpdateContinuousStates(&V1_Ball_and_Bean_Controller_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++V1_Ball_and_Bean_Controller_M->Timing.clockTick0)) {
    ++V1_Ball_and_Bean_Controller_M->Timing.clockTickH0;
  }

  V1_Ball_and_Bean_Controller_M->Timing.t[0] = rtsiGetSolverStopTime
    (&V1_Ball_and_Bean_Controller_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.0001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++V1_Ball_and_Bean_Controller_M->Timing.clockTick1)) {
      ++V1_Ball_and_Bean_Controller_M->Timing.clockTickH1;
    }

    V1_Ball_and_Bean_Controller_M->Timing.t[1] =
      V1_Ball_and_Bean_Controller_M->Timing.clockTick1 *
      V1_Ball_and_Bean_Controller_M->Timing.stepSize1 +
      V1_Ball_and_Bean_Controller_M->Timing.clockTickH1 *
      V1_Ball_and_Bean_Controller_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void V1_Ball_and_Bean_Controller_derivatives(void)
{
  XDot_V1_Ball_and_Bean_Control_T *_rtXdot;
  _rtXdot = ((XDot_V1_Ball_and_Bean_Control_T *)
             V1_Ball_and_Bean_Controller_M->ModelData.derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE = V1_Ball_and_Bean_Controller_B.Gain;

  /* Derivatives for TransferFcn: '<Root>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += V1_Ball_and_Bean_Controller_P.TransferFcn_A *
    V1_Ball_and_Bean_Controller_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += V1_Ball_and_Bean_Controller_B.SFunction1[0];

  /* Derivatives for TransferFcn: '<Root>/Derivative filter' */
  _rtXdot->Derivativefilter_CSTATE = 0.0;
  _rtXdot->Derivativefilter_CSTATE +=
    V1_Ball_and_Bean_Controller_P.Derivativefilter_A *
    V1_Ball_and_Bean_Controller_X.Derivativefilter_CSTATE;
  _rtXdot->Derivativefilter_CSTATE += V1_Ball_and_Bean_Controller_B.Gain;
}

/* Model initialize function */
void V1_Ball_and_Bean_Controller_initialize(void)
{
  /* Start for S-Function (rti_commonblock): '<S1>/S-Function1' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  V1_Ball_and_Bean_Controller_DW.SFunction1_IWORK[0] = 0;

  /* Start for S-Function (rti_commonblock): '<S1>/S-Function2' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  V1_Ball_and_Bean_Controller_DW.SFunction2_IWORK[1] = 0;

  /* Start for S-Function (rti_commonblock): '<S1>/S-Function3' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  V1_Ball_and_Bean_Controller_DW.SFunction3_IWORK[2] = 0;

  /* Start for S-Function (rti_commonblock): '<S1>/S-Function4' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  V1_Ball_and_Bean_Controller_DW.SFunction4_IWORK[3] = 0;

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  V1_Ball_and_Bean_Controller_X.Integrator_CSTATE =
    V1_Ball_and_Bean_Controller_P.Integrator_IC;

  /* InitializeConditions for TransferFcn: '<Root>/Transfer Fcn' */
  V1_Ball_and_Bean_Controller_X.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<Root>/Derivative filter' */
  V1_Ball_and_Bean_Controller_X.Derivativefilter_CSTATE = 0.0;
}

/* Model terminate function */
void V1_Ball_and_Bean_Controller_terminate(void)
{
  /* Terminate for S-Function (rti_commonblock): '<S1>/S-Function1' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH1_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh1 = 2;

  /* Terminate for S-Function (rti_commonblock): '<S1>/S-Function2' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH2_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh2 = 2;

  /* Terminate for S-Function (rti_commonblock): '<S1>/S-Function3' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH3_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh3 = 2;

  /* Terminate for S-Function (rti_commonblock): '<S1>/S-Function4' */

  /* dSPACE I/O Board DS1104 #1 Unit:PWM Group:PWM */
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH4_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh4 = 2;
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  V1_Ball_and_Bean_Controller_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  V1_Ball_and_Bean_Controller_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  V1_Ball_and_Bean_Controller_initialize();
}

void MdlTerminate(void)
{
  V1_Ball_and_Bean_Controller_terminate();
}

/* Registration function */
RT_MODEL_V1_Ball_and_Bean_Con_T *V1_Ball_and_Bean_Controller(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)V1_Ball_and_Bean_Controller_M, 0,
                sizeof(RT_MODEL_V1_Ball_and_Bean_Con_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
                          &V1_Ball_and_Bean_Controller_M->Timing.simTimeStep);
    rtsiSetTPtr(&V1_Ball_and_Bean_Controller_M->solverInfo, &rtmGetTPtr
                (V1_Ball_and_Bean_Controller_M));
    rtsiSetStepSizePtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
                       &V1_Ball_and_Bean_Controller_M->Timing.stepSize0);
    rtsiSetdXPtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
                 &V1_Ball_and_Bean_Controller_M->ModelData.derivs);
    rtsiSetContStatesPtr(&V1_Ball_and_Bean_Controller_M->solverInfo, (real_T **)
                         &V1_Ball_and_Bean_Controller_M->ModelData.contStates);
    rtsiSetNumContStatesPtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
      &V1_Ball_and_Bean_Controller_M->Sizes.numContStates);
    rtsiSetErrorStatusPtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
                          (&rtmGetErrorStatus(V1_Ball_and_Bean_Controller_M)));
    rtsiSetRTModelPtr(&V1_Ball_and_Bean_Controller_M->solverInfo,
                      V1_Ball_and_Bean_Controller_M);
  }

  rtsiSetSimTimeStep(&V1_Ball_and_Bean_Controller_M->solverInfo, MAJOR_TIME_STEP);
  V1_Ball_and_Bean_Controller_M->ModelData.intgData.y =
    V1_Ball_and_Bean_Controller_M->ModelData.odeY;
  V1_Ball_and_Bean_Controller_M->ModelData.intgData.f[0] =
    V1_Ball_and_Bean_Controller_M->ModelData.odeF[0];
  V1_Ball_and_Bean_Controller_M->ModelData.intgData.f[1] =
    V1_Ball_and_Bean_Controller_M->ModelData.odeF[1];
  V1_Ball_and_Bean_Controller_M->ModelData.intgData.f[2] =
    V1_Ball_and_Bean_Controller_M->ModelData.odeF[2];
  V1_Ball_and_Bean_Controller_M->ModelData.intgData.f[3] =
    V1_Ball_and_Bean_Controller_M->ModelData.odeF[3];
  V1_Ball_and_Bean_Controller_M->ModelData.contStates = ((real_T *)
    &V1_Ball_and_Bean_Controller_X);
  rtsiSetSolverData(&V1_Ball_and_Bean_Controller_M->solverInfo, (void *)
                    &V1_Ball_and_Bean_Controller_M->ModelData.intgData);
  rtsiSetSolverName(&V1_Ball_and_Bean_Controller_M->solverInfo,"ode4");

  /* Initialize timing info */
  {
    int_T *mdlTsMap =
      V1_Ball_and_Bean_Controller_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    V1_Ball_and_Bean_Controller_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    V1_Ball_and_Bean_Controller_M->Timing.sampleTimes =
      (&V1_Ball_and_Bean_Controller_M->Timing.sampleTimesArray[0]);
    V1_Ball_and_Bean_Controller_M->Timing.offsetTimes =
      (&V1_Ball_and_Bean_Controller_M->Timing.offsetTimesArray[0]);

    /* task periods */
    V1_Ball_and_Bean_Controller_M->Timing.sampleTimes[0] = (0.0);
    V1_Ball_and_Bean_Controller_M->Timing.sampleTimes[1] = (0.0001);

    /* task offsets */
    V1_Ball_and_Bean_Controller_M->Timing.offsetTimes[0] = (0.0);
    V1_Ball_and_Bean_Controller_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(V1_Ball_and_Bean_Controller_M,
             &V1_Ball_and_Bean_Controller_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = V1_Ball_and_Bean_Controller_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    V1_Ball_and_Bean_Controller_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(V1_Ball_and_Bean_Controller_M, -1);
  V1_Ball_and_Bean_Controller_M->Timing.stepSize0 = 0.0001;
  V1_Ball_and_Bean_Controller_M->Timing.stepSize1 = 0.0001;
  V1_Ball_and_Bean_Controller_M->solverInfoPtr =
    (&V1_Ball_and_Bean_Controller_M->solverInfo);
  V1_Ball_and_Bean_Controller_M->Timing.stepSize = (0.0001);
  rtsiSetFixedStepSize(&V1_Ball_and_Bean_Controller_M->solverInfo, 0.0001);
  rtsiSetSolverMode(&V1_Ball_and_Bean_Controller_M->solverInfo,
                    SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  V1_Ball_and_Bean_Controller_M->ModelData.blockIO = ((void *)
    &V1_Ball_and_Bean_Controller_B);

  {
    V1_Ball_and_Bean_Controller_B.Constant = 0.0;
    V1_Ball_and_Bean_Controller_B.Integrator = 0.0;
    V1_Ball_and_Bean_Controller_B.Ki = 0.0;
    V1_Ball_and_Bean_Controller_B.TransferFcn = 0.0;
    V1_Ball_and_Bean_Controller_B.Sum = 0.0;
    V1_Ball_and_Bean_Controller_B.Gain = 0.0;
    V1_Ball_and_Bean_Controller_B.Derivativefilter = 0.0;
    V1_Ball_and_Bean_Controller_B.Kd = 0.0;
    V1_Ball_and_Bean_Controller_B.Kp = 0.0;
    V1_Ball_and_Bean_Controller_B.Sum1 = 0.0;
    V1_Ball_and_Bean_Controller_B.Saturation = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction1[0] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction1[1] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction2[0] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction2[1] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction3[0] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction3[1] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction4[0] = 0.0;
    V1_Ball_and_Bean_Controller_B.SFunction4[1] = 0.0;
  }

  /* parameters */
  V1_Ball_and_Bean_Controller_M->ModelData.defaultParam = ((real_T *)
    &V1_Ball_and_Bean_Controller_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &V1_Ball_and_Bean_Controller_X;
    V1_Ball_and_Bean_Controller_M->ModelData.contStates = (x);
    (void) memset((void *)&V1_Ball_and_Bean_Controller_X, 0,
                  sizeof(X_V1_Ball_and_Bean_Controller_T));
  }

  /* states (dwork) */
  V1_Ball_and_Bean_Controller_M->ModelData.dwork = ((void *)
    &V1_Ball_and_Bean_Controller_DW);
  (void) memset((void *)&V1_Ball_and_Bean_Controller_DW, 0,
                sizeof(DW_V1_Ball_and_Bean_Controlle_T));

  {
    /* user code (registration function declaration) */
    /*Call the macro that initializes the global TRC pointers
       inside the model initialization/registration function. */
    RTI_INIT_TRC_POINTERS();
  }

  /* Initialize Sizes */
  V1_Ball_and_Bean_Controller_M->Sizes.numContStates = (3);/* Number of continuous states */
  V1_Ball_and_Bean_Controller_M->Sizes.numY = (0);/* Number of model outputs */
  V1_Ball_and_Bean_Controller_M->Sizes.numU = (0);/* Number of model inputs */
  V1_Ball_and_Bean_Controller_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  V1_Ball_and_Bean_Controller_M->Sizes.numSampTimes = (2);/* Number of sample times */
  V1_Ball_and_Bean_Controller_M->Sizes.numBlocks = (26);/* Number of blocks */
  V1_Ball_and_Bean_Controller_M->Sizes.numBlockIO = (15);/* Number of block outputs */
  V1_Ball_and_Bean_Controller_M->Sizes.numBlockPrms = (20);/* Sum of parameter "widths" */
  return V1_Ball_and_Bean_Controller_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
