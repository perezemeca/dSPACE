  /*********************** dSPACE target specific file *************************

   Header file V1_Ball_and_Bean_Controller_trc_ptr.h:

   Declaration of function that initializes the global TRC pointers

   RTI1104 7.3 (02-Nov-2014)
   Tue Sep 13 12:08:08 2022

   (c) Copyright 2008, dSPACE GmbH. All rights reserved.

  *****************************************************************************/
  #ifndef RTI_HEADER_V1_Ball_and_Bean_Controller_trc_ptr_h_
  #define RTI_HEADER_V1_Ball_and_Bean_Controller_trc_ptr_h_
  /* Include the model header file. */
  #include "V1_Ball_and_Bean_Controller.h"
  #include "V1_Ball_and_Bean_Controller_private.h"

  #ifdef EXTERN_C
  #undef EXTERN_C
  #endif

  #ifdef __cplusplus
  #define EXTERN_C                       extern "C"
  #else
  #define EXTERN_C                       extern
  #endif

  /*
   *  Declare the global TRC pointers
   */
              EXTERN_C volatile  real_T *p_0_V1_Ball_and_Bean_Controller_real_T_0;
              EXTERN_C volatile  real_T *p_1_V1_Ball_and_Bean_Controller_real_T_0;
              EXTERN_C volatile  int_T *p_2_V1_Ball_and_Bean_Controller_int_T_0;
              EXTERN_C volatile  real_T *p_3_V1_Ball_and_Bean_Controller_real_T_0;

   #define RTI_INIT_TRC_POINTERS() \
              p_0_V1_Ball_and_Bean_Controller_real_T_0 = &V1_Ball_and_Bean_Controller_B.Constant;\
              p_1_V1_Ball_and_Bean_Controller_real_T_0 = &V1_Ball_and_Bean_Controller_P.Constant_Value;\
              p_2_V1_Ball_and_Bean_Controller_int_T_0 = &V1_Ball_and_Bean_Controller_DW.SFunction1_IWORK[0];\
              p_3_V1_Ball_and_Bean_Controller_real_T_0 = &V1_Ball_and_Bean_Controller_X.Integrator_CSTATE;\

   #endif                       /* RTI_HEADER_V1_Ball_and_Bean_Controller_trc_ptr_h_ */
