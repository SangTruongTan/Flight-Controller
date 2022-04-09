/*
 * File: Sensor_data.c
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

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Computed Parameter: FIR_IMUaccel_Coefficients
   * Referenced by: '<S2>/FIR_IMUaccel'
   */
  { 0.0264077242F, 0.140531361F, 0.33306092F, 0.33306092F, 0.140531361F,
    0.0264077242F },

  /* Computed Parameter: IIR_IMUgyro_r_NumCoef
   * Referenced by: '<S2>/IIR_IMUgyro_r'
   */
  { 0.282124132F, 1.27253926F, 2.42084408F, 2.42084408F, 1.27253926F,
    0.282124132F },

  /* Computed Parameter: IIR_IMUgyro_r_DenCoef
   * Referenced by: '<S2>/IIR_IMUgyro_r'
   */
  { 1.0F, 2.22871494F, 2.52446198F, 1.57725322F, 0.54102242F, 0.0795623958F }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
