/*
 * File: Sensor.h
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

#ifndef RTW_HEADER_Sensor_h_
#define RTW_HEADER_Sensor_h_
#include "rtwtypes.h"
#ifndef Sensor_COMMON_INCLUDES_
# define Sensor_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Sensor_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef DEFINED_TYPEDEF_FOR_sensordata_t_
#define DEFINED_TYPEDEF_FOR_sensordata_t_

typedef struct {
  real32_T ddx;
  real32_T ddy;
  real32_T ddz;
  real32_T p;
  real32_T q;
  real32_T r;
  real32_T altitude_sonar;
  real32_T prs;
  real32_T vbat_V;
  uint32_T vbat_percentage;
} sensordata_t;

#endif

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T FIR_IMUaccel_states[15];    /* '<S2>/FIR_IMUaccel' */
  real32_T IIR_IMUgyro_r_states[5];    /* '<S2>/IIR_IMUgyro_r' */
  int32_T FIR_IMUaccel_circBuf;        /* '<S2>/FIR_IMUaccel' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: FIR_IMUaccel_Coefficients
   * Referenced by: '<S2>/FIR_IMUaccel'
   */
  real32_T FIR_IMUaccel_Coefficients[6];

  /* Computed Parameter: IIR_IMUgyro_r_NumCoef
   * Referenced by: '<S2>/IIR_IMUgyro_r'
   */
  real32_T IIR_IMUgyro_r_NumCoef[6];

  /* Computed Parameter: IIR_IMUgyro_r_DenCoef
   * Referenced by: '<S2>/IIR_IMUgyro_r'
   */
  real32_T IIR_IMUgyro_r_DenCoef[6];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  sensordata_t sensor_data_in;         /* '<Root>/sensor_data_in' */
  real32_T sensorCalibration_datin;    /* '<Root>/sensorCalibration_datin' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T ddx;                        /* '<Root>/ddx' */
  real32_T ddy;                        /* '<Root>/ddy' */
  real32_T ddz;                        /* '<Root>/ddz' */
  real32_T p;                          /* '<Root>/p' */
  real32_T q;                          /* '<Root>/q' */
  real32_T r;                          /* '<Root>/r' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Sensor_initialize(void);
extern void Sensor_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('stateEstimator/Sensor Preprocessing')    - opens subsystem stateEstimator/Sensor Preprocessing
 * hilite_system('stateEstimator/Sensor Preprocessing/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'stateEstimator'
 * '<S1>'   : 'stateEstimator/Sensor Preprocessing'
 * '<S2>'   : 'stateEstimator/Sensor Preprocessing/SensorPreprocessing'
 */
#endif                                 /* RTW_HEADER_Sensor_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
