/*------------------------------------------------------------------------
| vl53_api.c  --  libvl53 API function targets
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#include "vl53_plat.h"
#include "vl53_api.h"

/* Init Functions */

/*------------------------------------------------------------------------
| vl_deviceReset  --  place the device in a post power-up state
|
| Toggles the soft reset bit, with delays.
| Formerly tested the id reg, which passed on 1st test (w+r took
| 4ms with LB700 SBC).  Simplified to a 5ms wait after each write.
| Reset *assertion* restores the powerup default i2c SLA.
*/

void
vl_deviceReset( VL_Dev_t *Dev,
	uint8_t		newSla			/* desired SLA, 0=use default */
) {
	if( ! Dev->i2cDevAddr ) Dev->i2cDevAddr = 0x52;

	/*---  assert the reset using current i2c address  ---*/
	vl_wrByte( Dev, VlReg_G2SoftReset, 0x00 );
	vl_pollDelay( Dev );

	/*---  deassert reset using default i2c address  ---*/
	Dev->i2cDevAddr = 0x52;
	vl_wrByte( Dev, VlReg_G2SoftReset, 0x01 );
	vl_pollDelay( Dev );

	if( newSla ) {
		vl_wrByte( Dev, VlReg_I2cSlaveAddr, newSla >> 1 );
		Dev->i2cDevAddr = newSla;
	}
}

/*------------------------------------------------------------------------
| vl_deviceInit  --  device initialization
|
| Call this at startup and after a device reset.  Then use the Get/Set
| functions or load a custom register blast.
*/

void
vl_deviceInit( VL_Dev_t *Dev
) {
	VL_Spads_t	nvRef;
	uint8_t		j;
	uint16_t	temp0412;
	extern uint8_t		defaultTuning[];

	/*---  some s/w calibration defaults  ---*/
	VL_Cal( linearRangeScale ) = 1000;				/* 1.000 */
	VL_Cal( dmaxCalRangeMm ) = 400;
	VL_Cal( dmaxCalSigRateMcps ) = 1.42 * 65536;	/* FP1616 No Cover Glass */

	/*---  set default s/w limit values  ---*/
	VL_Meas( limVal_rit ) = 1.5 * 0.023 * 65536;	/* 1.5 * xtalk/SPAD */
	VL_Meas( limVal_sigmaFinalRange ) = 18 << 16;	/* 18.000 */
	VL_Meas( limVal_signalRefClip ) = 35 << 16;		/* 35.000 */

#ifdef I2C_2V8
	/* i2c runs at 1V8 by default */
	Status = vl_updateByte( Dev, VlReg_VhvPadI2cExtsupHv, 0xfe, 0x01 );
#endif
	/*---  Set I2C standard mode  ---*/
	vl_wrByte( Dev, 0x88, 0x00 );

	/*---  blast out the magic tuning settings  ---*/
	vl_load_settings( Dev, defaultTuning );

	/*---  config GPIO pin to 'new sample ready' interrupt  ---*/
	vl_setGpioConfig( Dev, 0, VlGpioFunc_Int, VlReg_GpioIntNewRange, 0 );

	/*---  save cycle-enabling modes/flags  ---*/
	vl_run_reg_rw( Dev, & Dev->runEnables, 0 );

	/*---  get NVM ref SPAD count & type  ---*/
	vl_get_nvram_info( Dev, & nvRef );	/* ref SPAD data, 40cm cal */
	for( j = 0; j < VL_SpadMapSiz; j++ )
		VL_Device( nvmRefSpadMap )[ j ] = nvRef.refSpadMap[ j ];

	/* default to a ref map based on NVM count & type */
	if( nvRef.nSpads > 32 || ( ! nvRef.tSpads && nvRef.nSpads > 12 ) ) {
		/*---  rebuild the map by signal rate convergence  ---*/
		nvRef.nSpads = 0;
	}
	vl_manage_ref_spads( Dev, & nvRef );

	/*---  record the Osc frequency, but never used  ---*/
	/* was set as a 'default' constant: 618660 (9.44MHz) */
	vl_wrByte( Dev, 0xff, 0x01 );
	vl_rdWord( Dev, 0x84, & temp0412 );		/* 619312 read from device, YMMV */
	vl_wrByte( Dev, 0xff, 0x00 );
	VL_Device( oscFreqMHz ) = temp0412 << 4;	/* FP1616 = FP0412 << 4 */

	/*---  cache timing & measurement parameters from the device  ---*/
	vl_getMeasure( Dev, NULL );
	vl_getTiming( Dev, NULL );
}

/* device and measurement setup */

/*------------------------------------------------------------------------
| vl_setMeasure  --  push wholesale measurement parameters out to the device
| vl_getMeasure  --  read measurement parameters from device
|
| If the provided pointer is NULL, the device is refreshed from the
| handler's cached values.
|
| pParams			Pointer to measurement parameters
*/

void vl_setMeasure( VL_Dev_t *Dev,
	const VL_MeasureParams_t	*pParams
) {
	VL_MeasureParams_t	*pDrv;			/* driver's image of parameters */
	uint16_t	osc_calibrate_val;
	uint32_t	encodedRepeatPeriod;	/* oscillator periods */

	/*---  copy user's structure to internal, then update device  ---*/
	pDrv = & Dev->MeasureParams;
	if( pParams ) *pDrv = *pParams;		/* copy from caller */

	/*---  set up measure repeat period  ---*/
	vl_rdWord( Dev, VlReg_OscCalibrate, &osc_calibrate_val );
	encodedRepeatPeriod = pDrv->repeatPeriodMs;
	if( osc_calibrate_val ) encodedRepeatPeriod *= osc_calibrate_val;
	vl_wrDWord( Dev, VlReg_TimedRangingPeriod, encodedRepeatPeriod );

	/*---  set up fractional range results  ---*/
	vl_wrByte( Dev, VlReg_RangeConfig, pDrv->rangeFractEnable ? 0x01 : 0 );

	/*---  limit checks, only device-processed limits need refresh  ---*/
	/* SigrateFinalRange, no h/w enable, limit value 0=disabled */
	vl_wrWord( Dev, VlReg_FinalRangeMinCountRateRtn,
		( pDrv->limitEnables & VlLim_SigRateFinalRange ) ?
			FP1616TOFP0907( pDrv->limVal_sigRateFinalRange ) : 0 );

	/* SigRateMsrc enable */
	vl_updateByte( Dev, VlReg_MsrcConfig, 0xfd,
		( pDrv->limitEnables & VlLim_SigRateMsrc ) ? 0 : 0x02 );

	/* SigratePreRange enable */
	vl_updateByte( Dev, VlReg_MsrcConfig, 0xef,
		( pDrv->limitEnables & VlLim_SigRatePreRange ) ? 0 : 0x10 );

	/* SigRateMsrc & SigRatePreRange value */
	vl_wrWord( Dev, VlReg_PreRangeMinCountRateRtn,
		FP1616TOFP0907( pDrv->limVal_sigRatePreRange ) );
}

/* cache values from device, then copy internal struct to the caller's */

void vl_getMeasure( VL_Dev_t *Dev,
	VL_MeasureParams_t	*pParams
) {
	VL_MeasureParams_t	*pDrv;		/* driver's image of parameters */
	uint16_t	osc_calibrate_val;
	uint32_t	encodedRepeatPeriod;
	uint16_t	tmp0907;
	uint8_t		regByte;

	pDrv = & Dev->MeasureParams;

	/*---  get measurement repeat interval  ---*/
	vl_rdWord( Dev, VlReg_OscCalibrate, &osc_calibrate_val );
	vl_rdDWord( Dev, VlReg_TimedRangingPeriod, &encodedRepeatPeriod );
	if( osc_calibrate_val ) encodedRepeatPeriod /= osc_calibrate_val;
	pDrv->repeatPeriodMs = encodedRepeatPeriod;

	/*---  get range fractional result enable  ---*/
	vl_rdByte( Dev, VlReg_RangeConfig, & regByte );
	pDrv->rangeFractEnable = regByte & 0x01;

	/*---  refresh h/w limit enables and values from the device  ---*/
	pDrv->limitEnables &= ~( VlLim_SigRateMsrc |
		VlLim_SigRatePreRange | VlLim_SigRateFinalRange );

	/* SigRateFinalRange, no h/w enable, limit value 0=disabled */
	vl_rdWord( Dev, VlReg_FinalRangeMinCountRateRtn, &tmp0907 );
	if( tmp0907 ) {		/* don't wipe the enabled limit */
		pDrv->limVal_sigRateFinalRange = FP0907TOFP1616( tmp0907 );
		pDrv->limitEnables |= VlLim_SigRateFinalRange;
	}

	/* SigRateMsrc & SigRatePreRange enables */
	vl_rdByte( Dev, VlReg_MsrcConfig, & regByte );
	if( ~regByte & 0x02 ) pDrv->limitEnables |= VlLim_SigRateMsrc;
	if( ~regByte & 0x10 ) pDrv->limitEnables |= VlLim_SigRatePreRange;

	/* SigRateMsrc & SigRatePreRange */
	vl_rdWord( Dev, VlReg_PreRangeMinCountRateRtn, &tmp0907 );
	pDrv->limVal_sigRatePreRange = FP0907TOFP1616( tmp0907 );

	if( pParams ) *pParams = *pDrv;		/* copy to caller */
}

/*------------------------------------------------------------------------
| vl_setTiming  --  Set timing parameters
| vl_getTiming  --  Get timing parameters
|
| Set: All parameters are set from the supplied structure.
| Get: All parameters are read from the device into the supplied structure.
|	Total measurement time is calculated.
*/

void vl_setTiming( VL_Dev_t *Dev,
	VL_TimingParams_t	*tp
) {
	uint8_t		usrEnab;
	uint8_t		seqFlags = 0;

	/*---  setup order is important  ---*/
	if( tp ) Dev->TimingParams = *tp;		/* copy from caller */

	/*---  set up sequence step enables  ---*/
	/*---  build the new configuration register image  ---*/
	usrEnab = VL_Timing( seqEnables );
	if( usrEnab & SeqStepTcc ) seqFlags |= 0x10;
	if( usrEnab & SeqStepDss ) seqFlags |= 0x28;	/* correct? */
	if( usrEnab & SeqStepMsrc ) seqFlags |= 0x04;
	if( usrEnab & SeqStepPreRange ) seqFlags |= 0x40;
	if( usrEnab & SeqStepFinalRange ) seqFlags |= 0x80;

	vl_wrByte(Dev, VlReg_SeqConfig, seqFlags );

	vl_set_vcsel_pulse_periods( Dev );
	vl_set_sequence_step_timeouts( Dev );

	/* Phase calibration is needed after changing a vcsel period */
	vl_calibrate_phase( Dev );
}

void vl_getTiming( VL_Dev_t *Dev,
	VL_TimingParams_t	*tp
) {
	uint8_t		devFlags;		/* from device */
	uint8_t		usrEnab = 0;	/* API flags */

	/*---  get sequence step enables  ---*/
	vl_rdByte( Dev, VlReg_SeqConfig, & devFlags );
	/*---  pick up only the documented steps (not cal)  ---*/
	if( devFlags & 0x10 ) usrEnab |= SeqStepTcc;
	if( devFlags & 0x08 ) usrEnab |= SeqStepDss;
	if( devFlags & 0x04 ) usrEnab |= SeqStepMsrc;
	if( devFlags & 0x40 ) usrEnab |= SeqStepPreRange;
	if( devFlags & 0x80 ) usrEnab |= SeqStepFinalRange;
	VL_Timing( seqEnables ) = usrEnab;

	vl_get_vcsel_pulse_periods( Dev );
	vl_get_sequence_step_timeouts( Dev );

	if( tp ) *tp = Dev->TimingParams;		/* copy to caller */
}

/*---  calibration  ---*/

/*------------------------------------------------------------------------
| vl_setCalibration  --  set calibration factors
| vl_getCalibration  --  get calibration factors
|
| Provides/accepts a structure containing all device calibration info.
|
| Calibration information:
| linearRangeScale		S/W Range scaling factor x1000 (1000 = unity)
| xTalkComp				compensation rate
| offset				range offset
| ref cal				vhv & phase
| dmax					sigRate and range
|
| Linear range scaling selects either device or s/w xtalk compensation.
| If 1.000, the device comp is used.  Otherwise, the device comp is set to 0,
| ignored on read, and S/W performs the comp.
*/

#define MaxOffsetUm		511000
#define MinOffsetUm		-512000

void vl_setCalibration( VL_Dev_t *Dev,
	VL_CalParams_t	*pCalInfo
) {
	VL_CalParams_t	*pDrv;			/* driver's image of parameters */

	pDrv = & Dev->CalibrateParams;

	/*---  qualify the linear range gain  ---*/
	if( pDrv->linearRangeScale > 1000 ) {
		Dev->err = VLerr_InvalidParams;
		return;
	}

	/*---  copy user's structure to internal, then update device  ---*/
	if( pCalInfo ) *pDrv = *pCalInfo;		/* copy from caller */

	/*---  write offset cal to device  ---*/
	if( pDrv->offsetUm > MaxOffsetUm || pDrv->offsetUm < MinOffsetUm ) {
		Dev->err = VLerr_InvalidParams;
		return;
	}
	/* convert um to 10.2mm by division of 250 */
	vl_wrWord( Dev, VlReg_AlgoRangeOffsetMm, ( pDrv->offsetUm / 250 ) & 0xfff );

	/*---  Reference calibration, vhv & phase  ---*/
	vl_ref_calibration_io( Dev, 0, & pDrv->vhvSettings, & pDrv->phaseCal );

	/*---  S/W Dmax cal point, if either is 0 use nvm parameters  ---*/
	if( ! pDrv->dmaxCalRangeMm || ! pDrv->dmaxCalSigRateMcps ) {
		pDrv->dmaxCalRangeMm = 400;		/* should this be from nvm? */
		pDrv->dmaxCalSigRateMcps = VL_Device( nvmSigRateAt40cm );
	}

	/*---  write xtalk compensation to device  ---*/
	vl_wrWord( Dev, VlReg_XtalkCompPeakRateMcps,
		pDrv->linearRangeScale == 1000 ?
			FP1616TOFP0313( pDrv->xTalkCompRateMcps ) : 0 );
}

/* update internal structure, then copy to caller */

void vl_getCalibration( VL_Dev_t *Dev,
	VL_CalParams_t	*pCalInfo
) {
	VL_CalParams_t	*pDrv;			/* driver's image of parameters */
	uint16_t		offsetReg;	/* offset is signed 10.2 format */
	int16_t			signedOffset;
	uint16_t		temp16;

	pDrv = & Dev->CalibrateParams;

	/*---  linear range scaling s/w control only, no action needed  ---*/
	/*---  read offset cal from device  ---*/
	vl_rdWord( Dev, VlReg_AlgoRangeOffsetMm, & offsetReg );

	/* offset register discards d15-12.  Writing 1s are read as 0s */
	signedOffset = offsetReg &= 0x0fff;
	if( signedOffset & 0x800 ) signedOffset |= 0xf000;	/* sign-extend */

	pDrv->offsetUm = signedOffset * 250;

	/*---  reference calibration, vhv & phase  ---*/
	vl_ref_calibration_io( Dev, 1, & pDrv->vhvSettings, & pDrv->phaseCal );

	/*---  Dmax cal s/w control only, no action needed  ---*/

	/*---  get xtalk compensation  ---*/
	if( pDrv->linearRangeScale == 1000 ) {
		vl_rdWord( Dev, VlReg_XtalkCompPeakRateMcps, & temp16 );
		pDrv->xTalkCompRateMcps = FP0313TOFP1616( temp16 );
	}
	if( pCalInfo ) *pCalInfo = *pDrv;		/* copy to caller */
}

/*------------------------------------------------------------------------
| Perform XTalk Measurement
|
| Measures the current crosstalk from glass in front of the sensor.
| Performs a histogram measurement and uses the results to measure the
| crosstalk.  There must be no target in front of the sensor.
|
| This function is not supported when the final range
| vcsel clock period is < 10 Pclks.
*/

void vl_xtalkMeasurement( VL_Dev_t *Dev,
	uint32_t	timeoutMs,			/* histogram measurement duration */
	FP1616_t	*pXtalkPerSpad,		/* crosstalk result, Mcps/SPAD */
	uint8_t		*pAmbientTooHigh	/* result invalid, Ambient too high */
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented on VL53L0X */
}

/*------------------------------------------------------------------------
| vl_calibrateRef  --  perform reference calibration
|
| This function will run two undocumented ranging cycles, which
| calibrate VHV and phase, both temperature-dependent.
| Call this before offset or xtalk cal, and occasionally while ranging.
|
| Any interrupts generated by the calibration cycles are cleared.
*/

void vl_calibrateRef( VL_Dev_t *Dev
) {
	vl_calibrate_ref( Dev );
}

/*------------------------------------------------------------------------
| vl_calibrateXtalk  --  perform xtalk calibration
|
| This function will make 50 ranging measurements, then set a new
| value for the XTalk compensation.
*/

void vl_calibrateXtalk( VL_Dev_t *Dev,
	uint16_t	xTalkCalDistance		/* setpoint distance, mm */
) {
	vl_calibrate_xtalk( Dev, xTalkCalDistance );
}

/*------------------------------------------------------------------------
| vl_calibrateOffset  --  perform offset calibration
|
| This function will make 50 ranging measurements, then set a new
| value of offset calibration.
*/

void vl_calibrateOffset( VL_Dev_t *Dev,
	SFP2408_t	calDistMm		/* setpoint distance, mm (usually 10cm) */
) {
	vl_calibrate_offset( Dev, calDistMm );
}

/* Measurement Functions */

/*------------------------------------------------------------------------
| vl_singleMeasurementWait  --  single ranging measurement, wait for results
|
| Returns the results and clears any interrupt.
*/

void vl_singleMeasurementWait( VL_Dev_t *Dev,
	VL_RangeData_t	*pData		/* caller's data structure to populate */
) {
	/*---  start a single ranging measurement  ---*/
	vl_startMeasurement( Dev, VlReg_RangeSingle );
	vl_wait_for_ranging( Dev );
	vl_getRangingResults( Dev, pData );
}

/*------------------------------------------------------------------------
| vl_startMeasurement  --  start ranging measurement, non-blocking
|
| VLerr_UnknownMode		Measurement mode provided is not one of:
|		VlReg_RangeSingle
|		VlReg_RangeContinuous
|		VlReg_RangeTimed
*/
void vl_startMeasurement( VL_Dev_t *Dev,
	uint8_t		measMode			/* measurement mode */
) {
	/*---  must be Standby for GetRun..() to detect stoppage  ---*/
	vl_wrByte( Dev, 0x80, 0x00 );

	switch( measMode ) {
	case VlReg_RangeSingle:
		vl_wrByte( Dev, VlReg_RangeCmd, measMode );
#ifdef AllFunction
{
		uint8_t		j;
		uint8_t		Byte;

		/*---  wait for go bit to clear  ---*/
		for( j = 0;; j++ ) {
			vl_rdByte( Dev, VlReg_RangeCmd, & Byte );
			if( Dev->err || ( ~Byte & 0x01 ) ) break;
			/* never gets here, just sayin' (wo bit?) */
			if( j > 5 ) {
				Dev->err = VLerr_Timeout;
				break;
			}
		}
}
#endif
		break;
	case VlReg_RangeContinuous:
	case VlReg_RangeTimed:
		/*---  apply interrupt settings for range comparisons  ---*/
		vl_checkAndLoadInterruptSettings( Dev, 1 );
		vl_wrByte( Dev, VlReg_RangeCmd, measMode );
		break;
	default:
		Dev->err = VLerr_UnknownMode;
	}
}

/*------------------------------------------------------------------------
| vl_stopRun  --  stop continuous and timed measurements
| vl_getRunStatus  --  sense active run status
|
| After vl_stopRun, *must* call vl_getRunStatus until device inactive.
*/

void vl_stopRun( VL_Dev_t *Dev
) {
	/*---  clear ranging mode  ---*/
	vl_wrByte( Dev, VlReg_RangeCmd, VlReg_RangeStop );
	vl_run_reg_rw( Dev, NULL, 0x00 );
	vl_checkAndLoadInterruptSettings( Dev, 0 );
}

void vl_getRunStatus( VL_Dev_t *Dev,
	uint8_t		*pStat		/* running = !0 */
) {
	uint8_t		rStat;

	vl_wrByte( Dev, 0xff, 0x01 );		/* device must be in Standby */
	vl_rdByte( Dev, 0x04, & rStat );	/*  when measurement is started */
	vl_wrByte( Dev, 0xff, 0x00 );		/*  for this to work */

	/*---  restore cycling enables  ---*/
	if( ! rStat ) vl_run_reg_rw( Dev, NULL, Dev->runEnables );

	*pStat = rStat;
}

/*------------------------------------------------------------------------
| Return Measurement Data Ready
|
| This function indicates if measurement data is ready.
| If the interrupt is set up, it is tested and cleared.
*/

void vl_getRangingDataReady( VL_Dev_t *Dev,
	uint8_t		*pReady			/* set !0 if ready */
) {
	uint8_t		rangeStatReg;
	uint8_t		intStatus;

	if( Dev->pin0Gpio == VlReg_GpioIntNewRange ) {	/* test int config'd */
		vl_getInterruptStatus( Dev, & intStatus );
		*pReady = ( intStatus == VlReg_GpioIntNewRange );	/* test active */
		if( *pReady ) vl_clearInterrupts( Dev, 0 );
	} else {
		vl_rdByte( Dev, VlReg_ResRangeStatus, & rangeStatReg );
		*pReady = rangeStatReg & 0x01;
	}
}

/*------------------------------------------------------------------------
| VL_GetRangingData  --  retrieve ranging measurement data
|
| Get data from last successful ranging measurement.
| cut1.1: if RangeConfig (0x09) is 1 range is FP1102
| else not fractional.  A cached enable is tested.
|
| Range scaling and xtalk comp:
| scaling = 1, send comp factor to device & let it do the comp.
| scaling != 1, send 0 (no comp) to device, and let s/w do the comp.
*/

void vl_getRangingResults( VL_Dev_t *Dev,
	VL_RangeData_t	*pData		/* destination data structure */
) {
	uint8_t		devStatReg;
	FP1616_t	sigRate;			/* peak signal return rate */
	uint16_t	rangeMm;			/* FP1102 mostly */
	uint16_t	devWord;
	FP1616_t	xTalkTotalCompMcps;

	vl_rdWord( Dev, 0x1e, & rangeMm );
	if( ! VL_Meas( rangeFractEnable ) ) rangeMm <<= 2;

	/*---  read peak signal return rate  ---*/
	vl_rdWord( Dev, 0x1a, & devWord );
	pData->sigRateRtnMcps = sigRate = FP0907TOFP1616( devWord );

	vl_rdWord( Dev, 0x1c, & devWord );
	pData->sigRateAmbRtnMcps = FP0907TOFP1616( devWord );

	vl_rdWord( Dev, 0x16, & pData->effSpadRtnCnt );

	/*---  read Signal Reference Rate from device  ---*/
	vl_wrByte( Dev, 0xff, 0x01 );
	vl_rdWord( Dev, VlReg_ResPeakSigRateRef, & devWord );
	vl_wrByte( Dev, 0xff, 0x00 );
	pData->sigRatePeakRefMcps = FP0907TOFP1616( devWord );

	vl_rdByte( Dev, VlReg_ResRangeStatus, & devStatReg );

	if( Dev->err ) return;

	if( VL_Cal( linearRangeScale ) != 1000 ) {
		rangeMm = ( rangeMm * VL_Cal( linearRangeScale ) + 500 ) / 1000;

		/*---  s/w Xtalk compensation  ---*/
		xTalkTotalCompMcps =	/* FP1616 = ( FP1616 * FP0808 ) >> 8 */
			( VL_Cal( xTalkCompRateMcps ) * pData->effSpadRtnCnt ) >> 8;

		if( sigRate < xTalkTotalCompMcps ) {
			/*---  comp too high  ---*/
			rangeMm = 8888 << 2;			/* a failure signature */
		} else {
			rangeMm = ( rangeMm * sigRate ) / ( sigRate - xTalkTotalCompMcps );
		}
	}
	/* fraction may be !0 even if device mode not fractional */
	pData->rangeMm = rangeMm >> 2;
	pData->rangeFractMm = ( rangeMm & 0x03 ) << 6;

	/*---  validate range data  ---*/
	vl_get_api_range_status( Dev, devStatReg, pData );
}

/*------------------------------------------------------------------------
| vl_getLimitActual  --  get the value a limit is checked against
|
| Perform a ranging before requesting actual limit data.
|
| no device access
*/

void vl_getLimitActual( VL_Dev_t *Dev,
	uint8_t			limId,			/* limit check ID (flag) */
	VL_RangeData_t	*rDat,			/* from last ranging */
	FP1616_t		*pLimit			/* where to store actual value */
) {
	if( ! Dev->err ) return;
	switch( limId ) {
	case VlLim_SigmaFinalRange:
		*pLimit = rDat->sigmaEstimate;	/* calc'd only when limit enabled */
		break;
	case VlLim_SignalRefClip:
		*pLimit = rDat->sigRatePeakRefMcps;
		break;
	case VlLim_Rit:
	case VlLim_SigRateMsrc:
	case VlLim_SigRatePreRange:
	case VlLim_SigRateFinalRange:
		*pLimit = rDat->sigRateRtnMcps;
		break;
	default:
		Dev->err = VLerr_InvalidParams;
	}
}

/*---  Power, GPIO and interrupt functions  ---*/

/*------------------------------------------------------------------------
| vl_setPowerMode  --  set device to Standby or Idle
| vl_getPowerMode  --  get the device power mode
|
| Changing mode does not appear to affect any device setup.
| This function should not be used when device is ranging.
*/

void
vl_getPowerMode( VL_Dev_t *Dev,
	VL_PowerModes_t *pMode
) {
	uint8_t Byte;

	/* only level1 of Power mode exists */
	vl_rdByte( Dev, VlReg_G1PowerMode, & Byte );

	*pMode = Byte == 1 ? VlPwr_Idle1 : VlPwr_Standby1;
}

void vl_setPowerMode( VL_Dev_t *Dev,
	VL_PowerModes_t mode		/* VlPwr_Standby1 or VlPwr_Idle1 */
) {
	if( Dev->err ) return;

	/* Only level1 of Power mode exists */
	switch( mode ) {
	case VlPwr_Standby1 :
		vl_wrByte( Dev, VlReg_G1PowerMode, 0x00 );
		break;
	case VlPwr_Idle1 :
		vl_wrByte( Dev, VlReg_G1PowerMode, 0x01 );	/*0x01:gfs */
		break;
	default:
		Dev->err = VLerr_UnknownMode;
	}
}

/*------------------------------------------------------------------------
| Set GPIO pin configuration
|
| Pin                   GPIO pin ID (only 0 valid)
| DeviceMode            GPIO pin mode (drive, osc or interrupt)
| Function				Pin function (interrupt condition)
| Polarity              Drive state, !0 for high, !0 for -ve int transition
|
| With !0 int polarity, GPIO is high during measurement
| There's a lot not known about how to configure the device.
|
| VLerr_UnknownGpioPin			Only Pin=0 is accepted.
| VLerr_UnknownGpioFunc		Not one of VL_GPIO_INT_*
*/

static uint8_t	oscBlast[] = {
	0xff,0x01, 0x00,0x00, 0xff,0x00, 0x80,0x01, 0x85,0x02,
	0xff,0x04, 0xcd,0x00, 0xcc,0x11, 0xff,0x07, 0xbe,0x00, 0xff,0x06, 0xcc,0x09,
	/* original: leaves alternate mapping on? (hangs SMB controller) */
//	0xff,0x00, 0xff,0x01, 0x00,0x00, MetaStop
	0xff,0x01, 0x00,0x01, 0xff,0x00, MetaStop	/* this works better */
};

void vl_setGpioConfig( VL_Dev_t *Dev, uint8_t Pin,
	VL_GpioMode_t mode, VL_GpioInt_t intSource,
	uint8_t polarity		/* for drive or int */
) {
	if( Dev->err ) return;
	if( Pin != 0 ) {
		Dev->err = VLerr_UnknownGpioPin;
		return;
	}
	Dev->pin0Gpio = VlReg_GpioIntNone;		/* for non-int modes */

	if( mode == VlGpioFunc_Drive ) {
		vl_wrByte( Dev, VlReg_GpioHvMux, polarity ? 0x01 : 0x10 );	/* ??? */
	} else if( mode == VlGpioFunc_Int ) {
		switch( intSource ) {
		case VlReg_GpioIntNone:
		case VlReg_GpioIntLevelLow:
		case VlReg_GpioIntLevelHigh:
		case VlReg_GpioIntOutOfWindow:
		case VlReg_GpioIntNewRange:
			Dev->pin0Gpio = intSource;
			vl_wrByte( Dev, VlReg_GpioIntType, intSource );

			/* 'drive' must be set high regardless of int polarity */
			/*gfs: changed to also set d0 */
			vl_updateByte( Dev, VlReg_GpioHvMux, 0xef, polarity ? 0x11 : 1 );

			vl_clearInterrupts( Dev, 0 );
			break;
		default:
			Dev->err = VLerr_UnknownGpioFunc;
		}
	} else if( mode == VlGpioFunc_Osc ) {
		/*---  emits approx 9.4MHz  ---*/
		vl_load_settings( Dev, oscBlast );
	} else {
		Dev->err = VLerr_UnknownGpioFunc;
	}
}

/*------------------------------------------------------------------------
| vl_clearInterrupts  --  clear specified interrupt condition(s)
| vl_getInterruptStatus  --  return currently raised device interrupt(s)
| VL_EnableInterupts  --  configure ranging interrupt (not implemented)
|
| User is able to activate/deactivate interrupts through vl_setGpioConfig()
| Int status d2-d0 seems to be same int source # as config reg.
| If the GPIO pin is used to signal an external CPU, this must be called.
|
*/

void vl_clearInterrupts( VL_Dev_t *Dev,
	uint8_t intMask			/* unused */
) {
	uint8_t		statByte;
	uint8_t		k;

	/* "clear bit 0 range interrupt, bit 1 error interrupt" */
	for( k = 0;; k++ ) {
		vl_wrByte( Dev, VlReg_IntAck, 0x01 );
		vl_wrByte( Dev, VlReg_IntAck, 0x00 );
		vl_rdByte( Dev, VlReg_ResultIntStatus, & statByte );
			if( Dev->err || ! ( statByte & 0x07 ) ) break;
		if( k > 2 ) {
			Dev->err = VLerr_InterruptNotCleared;	/* Can't clear */
			break;
		}
	}
}

void vl_getInterruptStatus( VL_Dev_t *Dev,
	uint8_t *pIntStatus
) {
	uint8_t		statByte;

	/* reading this reg doesn't change it */
	vl_rdByte( Dev, VlReg_ResultIntStatus, & statByte );
	if( Dev->err ) return;
	*pIntStatus = statByte & 0x07;

	if( statByte & 0x18 )
		Dev->err = VLerr_RangeError;		/* ...i wonder what these are */
}

void vl_enableInterrupts( VL_Dev_t *Dev, uint8_t InterruptMask
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented for VL53L0X */
}

/*------------------------------------------------------------------------
| Set low and high Interrupt thresholds
| Get high and low Interrupt thresholds
|
| Set low and high Interrupt thresholds for a given mode (ranging, ALS, ...)
| Units are mm, lux ..., depending on the mode.
| The device registers are x2.
*/

void vl_setInterruptThresholds( VL_Dev_t *Dev,
	uint16_t	thLow,			/* low threshold */
	uint16_t	thHigh			/* high threshold */
) {
	vl_wrWord(Dev, VlReg_ThresholdLow, (thLow >> 1) & 0x0fff );
	vl_wrWord(Dev, VlReg_ThresholdHigh, (thHigh >> 1) & 0x0fff );
}

void vl_getInterruptThresholds( VL_Dev_t *Dev,
	uint16_t	*pthLow,	/* mm, i guess */
	uint16_t	*pthHigh
) {
	uint16_t	reg16;

	vl_rdWord( Dev, VlReg_ThresholdLow, & reg16 );
	*pthLow = ( reg16 & 0x0fff ) << 1;

	vl_rdWord( Dev, VlReg_ThresholdHigh, & reg16 );
	*pthHigh = ( reg16 & 0x0fff ) << 1;
}

/* called when starting or stopping a Continuous or Timed measurement! */

void vl_checkAndLoadInterruptSettings( VL_Dev_t *Dev,
	uint8_t		startStop		/* !0 to activate, 0 to disable */
) {
	uint8_t		intSource;
	uint16_t	thLow;
	uint16_t	thHigh;
	extern uint8_t	InterruptThresholdSettings[];

	intSource = Dev->pin0Gpio;

	if( intSource == VlReg_GpioIntLevelLow ||	/* comparison interrupts */
		intSource == VlReg_GpioIntLevelHigh ||
		intSource == VlReg_GpioIntOutOfWindow
	) {
		vl_getInterruptThresholds( Dev, & thLow, & thHigh );

		if( startStop ) {
			if( thLow > 255 || thHigh > 255 ) {		/* puzzling... */
				/* a bewildering mega register blast */
				vl_load_settings( Dev, InterruptThresholdSettings );
			}
		} else {
			vl_wrByte( Dev, 0xff, 0x04 );
			vl_wrByte( Dev, 0x70, 0x00 );
			vl_wrByte( Dev, 0xff, 0x00 );
		}
	}
}

/* SPAD functions */

/*------------------------------------------------------------------------
| vl_manageRefSpads  --  create a ref SPAD map
| vl_getRefSpads  --  read SPAD map from device
|
| vl_manageReferenceSpads enables the minimum number of reference spads to
| achieve a target reference signal rate.
| Should be performed once during initialization.  DeviceInit calls this.
| Can be called to enable a fixed # of SPADs (specify the count and
| type) or to converge on the target signal rate (specify count as 0).
|
| Caller must provide the 'good' map.  The nvm map read from the device
| in DeviceInit is normally used for this.
|
| GetRefSpads reads the device for the current number of applied
| reference spads and their type.
|
| sp->nSpads		Number of reference SPADs
| sp->tSpads		SPAD type, aperture (1) or non-aperture (0).
*/

void vl_manageRefSpads( VL_Dev_t *Dev,
	VL_Spads_t	*sp
) {
	vl_manage_ref_spads( Dev, sp );
}

void vl_getRefSpads( VL_Dev_t *Dev,
	VL_Spads_t	*sp
) {
	vl_get_ref_spads( Dev, sp );
}

#ifdef AllFunction
/*------------------------------------------------------------------------
| Set SPAD ambient damper: factor and threshold (no explanation found)
|
| Register 0x40 is set to 0x40 (byte) in the tuning blast.
|	So 0x40 (the data) is the high byte, low written as 0.
| Register 0x42 is set to 00 in the tuning blast.
*/

void
vl_setSpadAmbientDamper( VL_Dev_t *Dev,
	uint16_t	threshold,
	uint16_t	factor
) {
	vl_wrByte( Dev, 0xff, 0x01 );
	vl_wrWord( Dev, 0x40, threshold );
	vl_wrByte( Dev, 0x42, (uint8_t) factor );
	vl_wrByte( Dev, 0xFF, 0x00 );
}

/*------------------------------------------------------------------------
| Wait for device booted after chip enable (hardware standby)
|
*/
void vl_waitDeviceBooted( VL_Dev_t *Dev
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented on VL53L0X */
}

/*------------------------------------------------------------------------
| Set Group parameter Hold state
| Set or remove device internal group parameter hold
*/

void vl_setGroupParamHold( VL_Dev_t *Dev,
	uint8_t		groupParamHold		/* hold state, on/off */
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented on VL53L0X */
}

/*------------------------------------------------------------------------
| Get the maximal distance for current setup
| Device must be initialized prior to calling this function.
| Any measured range value more than the value returned is to be considered
| "no target detected" or "no target in detectable range".
| Beware, the maximal distance depends on the setup
*/
void vl_getUpperLimitMm( VL_Dev_t *Dev,
	uint16_t	*pUpperLimitMm		/* maximal range limit (mm) */
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented on VL53L0X */
}

/*------------------------------------------------------------------------
| Performs a single histogram measurement and retrieve the histogram
| measurement data
| Waits for measurements complete and gets data.
|
| This function will clear the interrupt in case of these are enabled.
*/

void vl_singleMeasureHistogram( VL_Dev_t *Dev,
	VL_HistMeasData_t *pHistMeasData	/* destination data structure */
) {
	Dev->err = VLerr_NotAvailable;	/* not implemented on VL53L0X */
}

/*------------------------------------------------------------------------
| Get data from last successful Histogram measurement
|
| Will fill a NumberOfROIZones times the corresponding data structure
| used in the measurement function.
*/

void vl_getHistogramResults( VL_Dev_t *Dev,
	VL_HistMeasData_t *pHistMeasData	/* histogram data structure */
) {
	Dev->err = VLerr_NotAvailable;
}
#endif

