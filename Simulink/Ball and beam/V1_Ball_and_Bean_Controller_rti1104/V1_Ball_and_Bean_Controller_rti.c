/*********************** dSPACE target specific file *************************

   Include file V1_Ball_and_Bean_Controller_rti.c:

   Definition of functions and variables for the system I/O and for
   the hardware and software interrupts used.

   RTI1104 7.3 (02-Nov-2014)
   Tue Sep 13 12:08:08 2022

   (c) Copyright 2006, dSPACE GmbH. All rights reserved.

 *****************************************************************************/

#if !(defined(__RTI_SIMENGINE__) || defined(RTIMP_FRAME))
# error This file may be included only by the RTI(-MP) simulation engine.
#endif

/* Include the model header file. */
#include "V1_Ball_and_Bean_Controller.h"
#include "V1_Ball_and_Bean_Controller_private.h"

/* Defines for block output and parameter structure existence */
#define RTI_rtB_STRUCTURE_EXISTS       1
#define RTI_rtP_STRUCTURE_EXISTS       1
#define RTB_STRUCTURE_NAME             V1_Ball_and_Bean_Controller_B
#define RTP_STRUCTURE_NAME             V1_Ball_and_Bean_Controller_P

/* dSPACE generated includes for header files */
#include <brtenv.h>
#include <rtkernel.h>
#include <rti_assert.h>
#include <rtidefineddatatypes.h>
#include <def1104.h>
#include <slvdsp1104.h>
#include <rti_slv1104.h>

/****** Definitions: task functions for timer tasks *********************/

/* Timer Task 1. (Base rate). */
static void rti_TIMERA(rtk_p_task_control_block task)
{
  /* Task entry code BEGIN */
  /* -- None. -- */
  /* Task entry code END */

  /* Task code. */
  baseRateService(task);

  /* Task exit code BEGIN */
  /* -- None. -- */
  /* Task exit code END */
}

/* ===== Declarations of RTI blocks ======================================== */

/* flag = UNDEF */
int slaveDSPPwmStopFlagCh1 = 2;

/* flag = UNDEF */
int slaveDSPPwmStopFlagCh2 = 2;

/* flag = UNDEF */
int slaveDSPPwmStopFlagCh3 = 2;

/* flag = UNDEF */
int slaveDSPPwmStopFlagCh4 = 2;

/* ===== Definition of interface functions for simulation engine =========== */
#ifdef MULTITASKING
# define dsIsSampleHit(RTM,sti)        rtmGetSampleHitPtr(RTM)[sti]
#else
# define dsIsSampleHit(RTM,sti)        rtmIsSampleHit(RTM,sti,0)
#endif

#if defined(_INLINE)
# define __INLINE                      static inline
#else
# define __INLINE                      static
#endif

static void rti_mdl_initialize_host_services(void)
{
  ts_timestamp_type ts = { 0, 0 };

  host_service(1, &ts);
}

static void rti_mdl_initialize_io_boards(void)
{
  /* Registering of RTI products and modules at VCM */
  {
    vcm_module_register(VCM_MID_RTI1104, (void *) 0,
                        VCM_TXT_RTI1104, 7, 3, 0,
                        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);

    {
      vcm_module_descriptor_type* msg_mod_ptr;
      msg_mod_ptr = vcm_module_register(VCM_MID_MATLAB, (void *) 0,
        VCM_TXT_MATLAB, 8, 3, 0,
        VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_SIMULINK, msg_mod_ptr,
                          VCM_TXT_SIMULINK, 8, 3, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
      vcm_module_register(VCM_MID_RTW, msg_mod_ptr,
                          VCM_TXT_RTW, 8, 6, 0,
                          VCM_VERSION_RELEASE, 0, 0, 0, VCM_CTRL_NO_ST);
    }
  }

  /* dSPACE I/O Board DS1104SLAVE #1 */
  /* Initialize master - slave DSP communication */
  ds1104_slave_dsp_communication_init();

  /* Initialize automatic index generation */
  rti_slv1104_init_fcn_index();

  /* Initialize slave DSP error level */
  rti_slv1104_error_level = 0;

  /* dSPACE I/O Board DS1104SLAVE #1 Unit:PWM */
  /* Initialize Slave DSP PWM Channel 1 */
  ds1104_slave_dsp_pwm_init(0, 1.0/ 50, 0.5, SLVDSP1104_PWM_MODE_ASYM,
    SLVDSP1104_PWM_POL_HIGH, SLVDSP1104_PWM_CH1_MSK);

  /* Start Slave DSP PWM Channel 1 */
  ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH1_MSK);

  /* Register Slave DSP PWM Channel 1 */
  ds1104_slave_dsp_pwm_duty_write_register(0, &rti_slv1104_fcn_index[6], 1);

  /* Initialize Slave DSP PWM Channel 2 */
  ds1104_slave_dsp_pwm_init(0, 1.0/ 50, 0.5, SLVDSP1104_PWM_MODE_ASYM,
    SLVDSP1104_PWM_POL_HIGH, SLVDSP1104_PWM_CH2_MSK);

  /* Start Slave DSP PWM Channel 2 */
  ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH2_MSK);

  /* Register Slave DSP PWM Channel 2 */
  ds1104_slave_dsp_pwm_duty_write_register(0, &rti_slv1104_fcn_index[7], 2);

  /* Initialize Slave DSP PWM Channel 3 */
  ds1104_slave_dsp_pwm_init(0, 1.0/ 50, 0.5, SLVDSP1104_PWM_MODE_ASYM,
    SLVDSP1104_PWM_POL_HIGH, SLVDSP1104_PWM_CH3_MSK);

  /* Start Slave DSP PWM Channel 3 */
  ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH3_MSK);

  /* Register Slave DSP PWM Channel 3 */
  ds1104_slave_dsp_pwm_duty_write_register(0, &rti_slv1104_fcn_index[8], 3);

  /* Initialize Slave DSP PWM Channel 4 */
  ds1104_slave_dsp_pwm_init(0, 1.0/ 50, 0.5, SLVDSP1104_PWM_MODE_ASYM,
    SLVDSP1104_PWM_POL_HIGH, SLVDSP1104_PWM_CH4_MSK);

  /* Start Slave DSP PWM Channel 4 */
  ds1104_slave_dsp_pwm_start(0, SLVDSP1104_PWM_CH4_MSK);

  /* Register Slave DSP PWM Channel 4 */
  ds1104_slave_dsp_pwm_duty_write_register(0, &rti_slv1104_fcn_index[9], 4);

  /* dSPACE I/O Board DS1104SLAVE #1 Unit:PWM2D */
  ds1104_slave_dsp_pwm2d_read_register(2, &rti_slv1104_fcn_index[19], 1,
    SLVDSP1104_INT_DISABLE);
  ds1104_slave_dsp_pwm2d_init(2);
}

/* Function rti_mdl_slave_load() is empty */
#define rti_mdl_slave_load()

/* Function rti_mdl_rtk_initialize() is empty */
#define rti_mdl_rtk_initialize()

static void rti_mdl_initialize_io_units(void)
{
  /* dSPACE I/O Board DS1104SLAVE #1 Unit:PWM */
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH1_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh1 = 2;
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH2_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh2 = 2;
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH3_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh3 = 2;
  ds1104_slave_dsp_pwm_output_set(0, SLVDSP1104_PWM_CH4_MSK,
    SLVDSP1104_PWM_TTL_LOW);

  /* flag = UNDEF */
  slaveDSPPwmStopFlagCh4 = 2;
}

/* Function rti_mdl_acknowledge_interrupts() is empty */
#define rti_mdl_acknowledge_interrupts()

/* Function rti_mdl_timetables_register() is empty */
#define rti_mdl_timetables_register()

/* Function rti_mdl_timesync_simstate() is empty */
#define rti_mdl_timesync_simstate()

/* Function rti_mdl_timesync_baserate() is empty */
#define rti_mdl_timesync_baserate()

static void rti_mdl_background(void)
{
  /* dSPACE I/O Board DS1104SLAVE #1 */
  /* Check master - slave dsp communication */
  rti_slv1104_taskqueue_error_all_check();
}

__INLINE void rti_mdl_sample_input(void)
{
  /* Calls for base sample time: [0.0001, 0] */
  /* dSPACE I/O Board DS1104 #1 Unit:PWM2D */
  ds1104_slave_dsp_pwm2d_read_request( 2, rti_slv1104_fcn_index[19]);
}

static void rti_mdl_daq_service()
{
  /* dSPACE Host Service */
  host_service(1, rtk_current_task_absolute_time_ptr_get());
}

#undef __INLINE

/****** [EOF] ****************************************************************/
