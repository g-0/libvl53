/*------------------------------------------------------------------------
| vl53_dev.h  --  libvl53 VL53L0x device definitions
|
| New material Copyright (c) 2020 Gordon Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#ifndef _VL53_DEV_H_
#define _VL53_DEV_H_

#define PllPeriod_ps		1655			/* (604.23MHz) */

/*---  Device-specific Codes from the device Status Register  ---*/
typedef uint8_t VL_DevErr_t;

#define VL_DevErr_None					0
#define VL_DevErr_VcselContinuityTest	1
#define VL_DevErr_VcselWatchdogTest		2
#define VL_DevErr_NoVhvValueFound		3
#define VL_DevErr_MsrcNoTarget			4
#define VL_DevErr_SnrTest				5
#define VL_DevErr_RangePhaseTest		6
#define VL_DevErr_SigmaThresholdTest	7
#define VL_DevErr_Tcc					8
#define VL_DevErr_PhaseConsistency		9
#define VL_DevErr_MinClip				10
#define VL_DevErr_RangeOk				11
#define VL_DevErr_AlgoUnderflow			12
#define VL_DevErr_AlgoOverflow			13
#define VL_DevErr_Rit					14

/*---  Device register map  ---*/

#define VlReg_I2cSlaveAddr			0x8a

#define VlReg_RangeCmd				0x00
/* cannot stop an in-progress measurement */
#define VlReg_RangeStop				0x00
#define VlReg_RangeSingle			0x01	/* d0 self-resets */
#define VlReg_RangeContinuous		0x02
#define VlReg_RangeTimed			0x04
#define VlReg_RangeHistogram		0x08

#define VlReg_SeqConfig				0x01
#define VlReg_TimedRangingPeriod	0x04
#define VlReg_RangeConfig			0x09

#define VlReg_GpioIntType			0x0a
#define VlReg_GpioIntNone			0x00	/* no interrupt */
#define VlReg_GpioIntLevelLow		0x01	/* level < thresh_low */
#define VlReg_GpioIntLevelHigh		0x02	/* level > thresh_high */
#define VlReg_GpioIntOutOfWindow	0x03	/* beyond either limit */
#define VlReg_GpioIntNewRange		0x04	/* ranging data ready */

#define VlReg_ThresholdHigh			0x0c
#define VlReg_ThresholdLow			0x0e

#define VlReg_GpioHvMux				0x84

#define VlReg_IntAck				0x0b

/*---  Result registers  ---*/
#define VlReg_ResultIntStatus		0x13
#define VlReg_ResRangeStatus		0x14

/* next group is in mapped page 1 */
#define VlReg_ResAmbWindowEventsRtn			0xbc	/* not used */
#define VlReg_ResRangingTotEventsRtn		0xc0	/* not used */
#define VlReg_ResAmbWindowEventsRef			0xd0	/* not used */
#define VlReg_ResRangingTotEventsRef		0xd4	/* not used */
#define VlReg_ResPeakSigRateRef				0xb6	/* 0x1b6 */

/*---  Algo register  ---*/
#define VlReg_AlgoRangeOffsetMm				0x28	/* word */

/*---  Limit Check configuration registers  ---*/
#define VlReg_MsrcConfig					0x60

#define VlReg_PreRangeMinSnr				0x27
#define VlReg_PreRangeValidPhaseLow			0x56
#define VlReg_PreRangeValidPhaseHigh		0x57
#define VlReg_PreRangeMinCountRateRtn		0x64	/* FP0507 */

#define VlReg_FinalRangeMinSnr				0x67
#define VlReg_FinalRangeValidPhaseLow		0x47
#define VlReg_FinalRangeValidPhaseHigh		0x48
#define VlReg_FinalRangeMinCountRateRtn		0x44	/* word */

#define VlReg_PreRangeSigmaThreshold		0x61	/* word */

/*---  Pre Range registers  ---*/
#define VlReg_PreRangeVcselPeriod			0x50
#define VlReg_PreRangeTimeoutMac			0x51	/* word */

#define VlReg_HistogramBin					0x81
#define VlReg_HistogramInitialPhaseSelect	0x33
#define VlReg_HistogramReadoutCtrl			0x55

#define VlReg_FinalRangeVcselPeriod			0x70
#define VlReg_FinalRangeTimeoutMac			0x71	/* word */
#define VlReg_XtalkCompPeakRateMcps			0x20	/* word */

#define VlReg_MsrcTimeoutMac				0x46

#define VlReg_G1PowerMode					0x80
#define VlReg_G2SoftReset					0xbf
#define VlReg_IdentModel					0xc0
#define VlReg_IdentRevision					0xc2

#define VlReg_OscCalibrate					0xf8	/* ro */

#define VlReg_GlobalVcselWidth				0x32

#define VL_SpadMapSiz			6		/* map size, bytes */
#define VL_NumSpads				44		/* map size, bits */

/*---  the configured reference SPAD enable map  ---*/
#define VlReg_GlobalRefSpadEnableBase		0xb0
#define VlReg_GlobalRefSpadFirstSpad		0xb6

#define VlReg_DynRefSpadNumRequested		0x4e	/* 0x14e */
#define VlReg_DynRefSpadStartOffset			0x4f	/* 0x14f */

#define VlReg_VhvPadI2cExtsupHv				0x89

#define VlReg_AlgoPhaseCalLimit				0x30	/* 0x130 */
#define VlReg_AlgoPhaseCalTimeout			0x30

/* 0x104 stop completed status */
/* 0x140 */
/* 0x142 */
/* 0x191 cycling modes/enables */
/* 0x*ff page select, 00 for normal operation */

#endif

