/*
 * File: Sensor.c
 *
 * Code generated for Simulink model 'Sensor'.
 *
 * Model version                  : 1.50
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Mon Mar 28 18:04:56 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Sensor_MPU.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void Sensor_step(void)
{
  int32_T cff;
  int32_T j;
  real32_T rtb_r;
  real32_T IIR_IMUgyro_r_tmp;
  real32_T inverseIMU_gain_idx_0;
  real32_T inverseIMU_gain_idx_1;
  real32_T inverseIMU_gain_idx_2;

  /* Gain: '<S2>/inverseIMU_gain' incorporates:
   *  Inport: '<Root>/sensor_data_in'
   */
  inverseIMU_gain_idx_0 = 0.994075298F * rtU.sensor_data_in.ddx;
  inverseIMU_gain_idx_1 = 0.996184587F * rtU.sensor_data_in.ddy;
  inverseIMU_gain_idx_2 = 1.00549F * rtU.sensor_data_in.ddz;

  /* Outport: '<Root>/p' incorporates:
   *  Gain: '<S2>/inverseIMU_gain'
   *  Inport: '<Root>/sensor_data_in'
   */
  rtY.p = 1.00139189F * rtU.sensor_data_in.p;

  /* Outport: '<Root>/q' incorporates:
   *  Gain: '<S2>/inverseIMU_gain'
   *  Inport: '<Root>/sensor_data_in'
   */
  rtY.q = 0.993601203F * rtU.sensor_data_in.q;

  /* DiscreteFir: '<S2>/FIR_IMUaccel' */
  IIR_IMUgyro_r_tmp = inverseIMU_gain_idx_0 * 0.0264077242F;
  cff = 1;
  for (j = rtDW.FIR_IMUaccel_circBuf; j < 5; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  for (j = 0; j < rtDW.FIR_IMUaccel_circBuf; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  /* Outport: '<Root>/ddx' incorporates:
   *  DiscreteFir: '<S2>/FIR_IMUaccel'
   */
  rtY.ddx = IIR_IMUgyro_r_tmp;

  /* DiscreteFir: '<S2>/FIR_IMUaccel' */
  IIR_IMUgyro_r_tmp = inverseIMU_gain_idx_1 * 0.0264077242F;
  cff = 1;
  for (j = rtDW.FIR_IMUaccel_circBuf; j < 5; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[5 + j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  for (j = 0; j < rtDW.FIR_IMUaccel_circBuf; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[5 + j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  /* Outport: '<Root>/ddy' incorporates:
   *  DiscreteFir: '<S2>/FIR_IMUaccel'
   */
  rtY.ddy = IIR_IMUgyro_r_tmp;

  /* DiscreteFir: '<S2>/FIR_IMUaccel' */
  IIR_IMUgyro_r_tmp = inverseIMU_gain_idx_2 * 0.0264077242F;
  cff = 1;
  for (j = rtDW.FIR_IMUaccel_circBuf; j < 5; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[10 + j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  for (j = 0; j < rtDW.FIR_IMUaccel_circBuf; j++) {
    IIR_IMUgyro_r_tmp += rtDW.FIR_IMUaccel_states[10 + j] *
      rtConstP.FIR_IMUaccel_Coefficients[cff];
    cff++;
  }

  /* Outport: '<Root>/ddz' incorporates:
   *  DiscreteFir: '<S2>/FIR_IMUaccel'
   */
  rtY.ddz = IIR_IMUgyro_r_tmp;

  /* DiscreteFilter: '<S2>/IIR_IMUgyro_r' incorporates:
   *  Gain: '<S2>/inverseIMU_gain'
   *  Inport: '<Root>/sensor_data_in'
   */
  IIR_IMUgyro_r_tmp = 1.00003F * rtU.sensor_data_in.r;
  cff = 1;
  for (j = 0; j < 5; j++) {
    IIR_IMUgyro_r_tmp -= rtConstP.IIR_IMUgyro_r_DenCoef[cff] *
      rtDW.IIR_IMUgyro_r_states[j];
    cff++;
  }

  rtb_r = 0.282124132F * IIR_IMUgyro_r_tmp;
  cff = 1;
  for (j = 0; j < 5; j++) {
    rtb_r += rtConstP.IIR_IMUgyro_r_NumCoef[cff] * rtDW.IIR_IMUgyro_r_states[j];
    cff++;
  }

  /* End of DiscreteFilter: '<S2>/IIR_IMUgyro_r' */

  /* Outport: '<Root>/r' */
  rtY.r = rtb_r;

  /* Update for DiscreteFir: '<S2>/FIR_IMUaccel' */
  /* Update circular buffer index */
  rtDW.FIR_IMUaccel_circBuf--;
  if (rtDW.FIR_IMUaccel_circBuf < 0) {
    rtDW.FIR_IMUaccel_circBuf = 4;
  }

  /* Update circular buffer */
  rtDW.FIR_IMUaccel_states[rtDW.FIR_IMUaccel_circBuf] = inverseIMU_gain_idx_0;
  rtDW.FIR_IMUaccel_states[rtDW.FIR_IMUaccel_circBuf + 5] =
    inverseIMU_gain_idx_1;
  rtDW.FIR_IMUaccel_states[rtDW.FIR_IMUaccel_circBuf + 10] =
    inverseIMU_gain_idx_2;

  /* End of Update for DiscreteFir: '<S2>/FIR_IMUaccel' */

  /* Update for DiscreteFilter: '<S2>/IIR_IMUgyro_r' */
  rtDW.IIR_IMUgyro_r_states[4] = rtDW.IIR_IMUgyro_r_states[3];
  rtDW.IIR_IMUgyro_r_states[3] = rtDW.IIR_IMUgyro_r_states[2];
  rtDW.IIR_IMUgyro_r_states[2] = rtDW.IIR_IMUgyro_r_states[1];
  rtDW.IIR_IMUgyro_r_states[1] = rtDW.IIR_IMUgyro_r_states[0];
  rtDW.IIR_IMUgyro_r_states[0] = IIR_IMUgyro_r_tmp;
}

/* Model initialize function */
void Sensor_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
