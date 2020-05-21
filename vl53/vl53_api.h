/*------------------------------------------------------------------------
| vl53_api.h  --  libvl53 api header
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
|
| Application modules #include this one header.
*/

#ifndef _VL53_API_H_
#define _VL53_API_H_

#include "vl53_dev.h"
#include "vl53_def.h"
#include "vl53_core.h"

/*---  init  ---*/

void vl_deviceReset( VL_Dev_t *, uint8_t );
void vl_deviceInit( VL_Dev_t * );

/*---  device setup  ---*/

void vl_setMeasure( VL_Dev_t *, const VL_MeasureParams_t * );
void vl_getMeasure( VL_Dev_t *, VL_MeasureParams_t * );

void vl_setTiming( VL_Dev_t *, VL_TimingParams_t * );
void vl_getTiming( VL_Dev_t *, VL_TimingParams_t * );

/*---  calibration  ---*/

void vl_setCalibration( VL_Dev_t *, VL_CalParams_t * );
void vl_getCalibration( VL_Dev_t *, VL_CalParams_t * );

void vl_calibrateRef( VL_Dev_t * );
void vl_calibrateXtalk( VL_Dev_t *, uint16_t );
void vl_calibrateOffset( VL_Dev_t *, SFP2408_t );

void vl_xtalkMeasurement( VL_Dev_t *, uint32_t, FP1616_t *, uint8_t * );

/*---  measurement  ---*/

void vl_singleMeasurementWait( VL_Dev_t *, VL_RangeData_t * );
void vl_startMeasurement( VL_Dev_t *, uint8_t );
void vl_stopRun( VL_Dev_t * );
void vl_getRunStatus( VL_Dev_t *, uint8_t * );

void vl_getRangingDataReady( VL_Dev_t *, uint8_t * );
void vl_getRangingResults( VL_Dev_t * Dev, VL_RangeData_t * );
void vl_getLimitActual( VL_Dev_t *, uint8_t, VL_RangeData_t *, FP1616_t * );

/*---  power, GPIO and interrupt  ---*/

void vl_setPowerMode( VL_Dev_t *, VL_PowerModes_t );
void vl_getPowerMode( VL_Dev_t *, VL_PowerModes_t * );

void vl_setGpioConfig( VL_Dev_t *, uint8_t, VL_GpioMode_t, VL_GpioInt_t, uint8_t );

void vl_clearInterrupts( VL_Dev_t *, uint8_t );
void vl_getInterruptStatus( VL_Dev_t *, uint8_t * );
void vl_enableInterrupts( VL_Dev_t *, uint8_t );

void vl_setInterruptThresholds( VL_Dev_t *, uint16_t, uint16_t );
void vl_getInterruptThresholds( VL_Dev_t *, uint16_t *, uint16_t * );
void vl_checkAndLoadInterruptSettings( VL_Dev_t *, uint8_t );

/*---  SPAD SPAD SPAD  ---*/

void vl_manageRefSpads( VL_Dev_t *, VL_Spads_t * );
void vl_getRefSpads( VL_Dev_t *, VL_Spads_t * );

void vl_setSpadAmbientDamper( VL_Dev_t *, uint16_t, uint16_t );

/* unused tailings - not VL53L0x */
void vl_waitDeviceBooted( VL_Dev_t * );
void vl_setGroupParamHold( VL_Dev_t *, uint8_t );
void vl_getUpperLimitMm( VL_Dev_t *, uint16_t * );
void vl_singleMeasureHistogram( VL_Dev_t *, VL_HistMeasData_t * );
void vl_getHistogramResults( VL_Dev_t *, VL_HistMeasData_t * );

#endif

