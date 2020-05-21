/*------------------------------------------------------------------------
| vl53_cal.c  --  libvl53 calibration functions
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#include "vl53_plat.h"
#include "vl53_api.h"

#define FirstSpad			180

/*---  internal fcn protos  ---*/
static void i_new_ref_spads( VL_Dev_t *, uint8_t, uint8_t [], uint8_t [],
	uint8_t, uint8_t * );
static void i_get_next_good_spad( uint8_t [], uint8_t * );
static void i_ref_signal_measure( VL_Dev_t *, uint16_t * );
static void i_vhv_calibration( VL_Dev_t * );
static void i_ref_calibration_cycle( VL_Dev_t *, uint8_t, uint8_t );
static uint8_t i_is_aperture( uint8_t );
static void i_write_ref_spad_map( VL_Dev_t *, uint8_t * );
static void i_read_ref_spad_map( VL_Dev_t *, uint8_t * );

/* new offset cal is calculated and applied */
void
vl_calibrate_offset( VL_Dev_t *Dev,
	SFP2408_t	calDistanceMm
) {
	VL_RangeData_t		rangeData;
	uint16_t	sumRanging = 0;
	uint8_t		nMeas = 0;
	SFP2408_t	avgRange;
	int			meas = 0;
	uint8_t		seqMask;			/* enables at entry */

	VL_Cal( offsetUm ) = 0;			/* remove offset comp */
	vl_setCalibration( Dev, NULL );

	/* save the current sequence enables, they will be restored */
	seqMask = VL_Timing( seqEnables );

	/* Disable the TCC sequence step */
	VL_Timing( seqEnables ) &= ~SeqStepTcc;
	vl_setTiming( Dev, NULL );
	/* Disable the RIT limit check */
	VL_Meas( limitEnables ) &= ~VlLim_Rit;
	vl_setMeasure( Dev, NULL );

	/* Perform 50 measurements and compute the averages */
	sumRanging = 0;
	nMeas = 0;
	for( meas = 0; meas < 50; meas++ ) {
		vl_singleMeasurementWait( Dev, & rangeData );
		if( ! rangeData.rangeStat ) {		/* only consider valid ranges */
			sumRanging += rangeData.rangeMm;
			nMeas++;
		}
	}
	if( Dev->err ) return;
	if( ! nMeas ) {
		Dev->err = VLerr_NoValidRange;
		return;
	}

	/* SFP2408 = uint16_t << 8 / uint8_t */
	avgRange = ( (SFP2408_t) sumRanging << 8 ) / nMeas;

	/*---  calculate the signed offset and apply it  ---*/
	VL_Cal( offsetUm ) = ( ( calDistanceMm - avgRange ) * 250 ) / 0x40;
	vl_setCalibration( Dev, NULL );

	/* Restore the sequence enables (tcc) */
	VL_Timing( seqEnables ) = seqMask;
	vl_setTiming( Dev, NULL );
}

/* vl_calibrate_xtalk  --  calculate new compensation at given distance
|
| Calculates the xtalk comp for the current range scaling and makes it
| active.  Called only from API, no internal callers
*/

void
vl_calibrate_xtalk( VL_Dev_t *Dev,
	uint16_t	calRangeMm
) {
	VL_MeasureParams_t		measPar;	/* saved measurement parameters */
	FP1616_t	avgSigRate;
	FP1616_t	avgRange;
	FP1616_t	avgSpads;

	/*---  adjust some measurement parameters  ---*/
	vl_getMeasure( Dev, & measPar );
	VL_Meas( limitEnables ) &= ~VlLim_Rit;	/* disable RIT check */
	VL_Cal( xTalkCompRateMcps ) = 0;	/* disable all xtalk compensation */
	vl_setCalibration( Dev, NULL );
	vl_setMeasure( Dev, NULL );				/* update device */

	/* Perform 50 measurements and compute the averages */
{
	VL_RangeData_t		rangeData;
	uint8_t		nMeas;			/* # of good measurements */
	uint16_t	sumRanging;
	uint16_t	sumSpads;
	FP1616_t	sumSigRate;
	uint8_t		k;

	sumRanging = sumSpads = sumSigRate = 0;
	nMeas = 0;
	for( k = 0; k < 50; k++ ) {
		vl_singleMeasurementWait( Dev, & rangeData );

		/* The range is valid when rangeStat = 0 */
		if( ! rangeData.rangeStat ) {
			sumRanging += rangeData.rangeMm;
			sumSigRate += rangeData.sigRateRtnMcps;
			sumSpads += rangeData.effSpadRtnCnt >> 8;
			nMeas++;
		}
	}
	if( Dev->err ) return;
	if( ! nMeas ) {
		Dev->err = VLerr_NoValidRange;
		return;
	}
	/* FP1616 = FP1616 / uint8_t */
	avgSigRate = sumSigRate / nMeas;
	avgRange = ( (uint32_t)sumRanging << 16 ) / nMeas;
	avgSpads = ( (uint32_t)sumSpads << 16 ) / nMeas;
}
{
	uint32_t	avgSpadsAsInt;
	uint32_t	signalXTalkPerSpad;
	FP1616_t	xtalkCompRateMcps;			/* final result */

	/* Round # Spads to a whole number.
	| Typically the average SPAD count is very close to a whole number,
	| therefore any truncation will not result in a significant error.
	| Also, for a grey target at approx 400mm, around 220 SPADs will
	| be enabled, so truncation causes less than 0.5% error.
	*/
	avgSpadsAsInt = (avgSpads + 0x8000) >> 16;

	if( avgSpadsAsInt == 0 || calRangeMm == 0 ||	/* denominators */
		/* xtalk comp must increase the measurement */
		( avgRange >> 16 ) >= calRangeMm
	) {
		xtalkCompRateMcps = 0;
	} else {
		/* Divide by SPAD count now to maintain a 32bit calculation
		| FP1616 = FP1616 / int */
		signalXTalkPerSpad = avgSigRate / avgSpadsAsInt;

		/* xtalk compensation Signal per SPAD
		| FP0032 = FP1616 *= FP1616 - FP1616/int */
		signalXTalkPerSpad *= (1 << 16) - avgRange / calRangeMm;

		/* FP1616 = FP0032 >> 16 */
		xtalkCompRateMcps = (signalXTalkPerSpad + 0x8000) >> 16;
	}
	VL_Cal( xTalkCompRateMcps ) = xtalkCompRateMcps;
	vl_setCalibration( Dev, NULL );		/* to device, if needed */
}
	/*---  restore the measurement parameters  ---*/
	vl_setMeasure( Dev, & measPar );
}

/*
| This creates a new reference SPAD map in the device.
|
| A map of eligible SPADs is provided.  The new map is generated by specifying
| the # of SPADs to enable (and type), or allowing the function to
| select SPADs by converging on a target reference signal rate.
|
| Either aperture or non-aperture spads are applied, but never both.
|
| To converge, non-aperture spads are selected first, begining with
| MinSpadCnt spads. If the target rate is exceeded when minimum spads
| are enabled, convergence is attempted with aperture spads.
|
| SPADs are enabled one at a time until a signal rate closest to the
| target rate is achieved.
|
| This procedure operates within a SPAD window of interest of a maximum
| VL_NumSpads spads.
|
| The start SPAD # is currently fixed to 180, which lies towards the end
| of the non-aperture quadrant and runs into the adjacent aperture quadrant.
| Future revision could allow the app to provide this as an argument.
|
*/

#define MinSpadCnt			3			/* was 5 ? */
#define TargetRateRef		0x0a00		/* 20 Mcps FP0907 */

void
vl_manage_ref_spads( VL_Dev_t *Dev,
	VL_Spads_t	*spDat		/* 'good' map and operation options */
) {
	uint8_t		workMap[ VL_SpadMapSiz ];	/* for building new map */
	uint32_t	spadType;			/* aperture or non-aperture SPADs */
	uint8_t		spadIndex;			/* index runner */
	uint8_t		lastSpad;			/* may need to remove it again */
	uint16_t	peakSigRateRef;
	uint32_t	sigRateDiff = 0;
	uint32_t	lastSigRateDiff = 0;

	vl_wrByte( Dev, 0xff, 0x01 );
	vl_wrByte( Dev, VlReg_DynRefSpadStartOffset, 0 );
	vl_wrByte( Dev, VlReg_DynRefSpadNumRequested, VL_NumSpads );
	vl_wrByte( Dev, 0xff, 0x00 );

	vl_wrByte( Dev, VlReg_GlobalRefSpadFirstSpad, FirstSpad );
//	vl_wrByte( Dev, VlReg_G1PowerMode, 0 );	/* standby */

	/*---  perform reference calibration  ---*/
	vl_calibrate_ref( Dev );

	/*---  use a simple forced method if SPAD count != 0  ---*/
	if( spDat->nSpads ) {
		i_new_ref_spads( Dev, spDat->tSpads, spDat->refSpadMap,
			workMap, spDat->nSpads, & spadIndex );
		return;
	}

	/*---  enable minimum non-aperture SPADs  ---*/
	spadType = 0;
	i_new_ref_spads( Dev, spadType, spDat->refSpadMap,
		workMap, MinSpadCnt, & spadIndex );

	i_ref_signal_measure( Dev, & peakSigRateRef );
	if( Dev->err ) return;
	if( peakSigRateRef > TargetRateRef ) {
		/*---  signal rate too high, switch to aperture SPADs  ---*/
		spadType = 1;
		i_new_ref_spads( Dev, spadType, spDat->refSpadMap,
			workMap, MinSpadCnt, & spadIndex );

		/*---  get signal rate for first convergence pass  ---*/
		i_ref_signal_measure( Dev, & peakSigRateRef );
	}
	if( ! Dev->err && peakSigRateRef < TargetRateRef ) {
		/* The minimum number of either aperture or non-aperture spads
		| have been set.  Add spads one at a time and perform measurements
		| until the target signal rate is reached.
		*/
		lastSigRateDiff =
			abs( (int16_t) peakSigRateRef - (int16_t) TargetRateRef );

		for( ;; ) {
			i_get_next_good_spad( spDat->refSpadMap, & spadIndex );
			if( spadIndex >= VL_NumSpads ) {
				Dev->err = VLerr_InsuffRefSpads;
				return;
			}

			/* Cannot combine Aperture and Non-Aperture spads, so
			| ensure the current spad is of the correct type.
			*/
			if( i_is_aperture( spadIndex ) != spadType ) {
				break;		/* all available same-type spads are enabled */
			}
			/*---  add the new SPAD and write it out  ---*/
			workMap[ spadIndex / 8 ] |= 0x1 << ( spadIndex % 8 );
			i_write_ref_spad_map( Dev, workMap );

			lastSpad = spadIndex++;
			lastSigRateDiff = sigRateDiff;

			i_ref_signal_measure( Dev, &peakSigRateRef );

			sigRateDiff =
				abs( (int16_t) peakSigRateRef - (int16_t) TargetRateRef );

			if( peakSigRateRef > TargetRateRef ) {
				/* now beyond target rate.  Keep latest spad only if closer
				| to target (above/below) than previous */
				if( sigRateDiff > lastSigRateDiff ) {
					/* Previous map produced a closer measurement */
					workMap[ lastSpad / 8 ] &= ~( 0x1 << ( lastSpad % 8 ) );
					i_write_ref_spad_map( Dev, workMap );
				}
				break;
			}
			if( Dev->err ) return;
		}
	}
}

/* Reads the SPAD map from the device and finds the aperture
| count and type from the map.
| The apertureness is set according to the first enable found.
|
| Called only from one API call.
*/

void
vl_get_ref_spads( VL_Dev_t *Dev,
	VL_Spads_t	*sp
) {
	uint8_t		spadTypeSet = 0;	/* SPAD type has been set */
	uint8_t		k;

	i_read_ref_spad_map( Dev, sp->refSpadMap );	/* from device */

	/*---  count the enabled SPADs in the map  ---*/
	sp->nSpads = 0;
	for( k = 0; k < VL_NumSpads; k++ ) {
		if( sp->refSpadMap[ k / 8 ] & ( 0x1 << ( k % 8 ) ) ) {
			sp->nSpads++;
			if( ! spadTypeSet ) {
				sp->tSpads = i_is_aperture( k );
				spadTypeSet = ! 0;
			}
		}
	}
}

/*
| Creates an output SPAD array from an input array, enabling SPADs
| of only a given type, up to a requested maximum number.
|
| The starting point is the lowest offset (SPAD #) of the given type.
| The offset of the last spad enabled is returned.
|
| The output SPAD map is written to the device, with a readback check.
*/

static void
i_new_ref_spads( VL_Dev_t *Dev,
	uint8_t		spadType,			/* 1 or 0, 1=aper */
	uint8_t		goodSpadArray[],	/* input array */
	uint8_t		spadArray[],		/* output array */
	uint8_t		reqCount,			/* # of new enables to add */
	uint8_t		*offset				/* runner */
) {
	uint8_t		i;

	if( Dev->err ) return;
	for( i = 0; i < VL_SpadMapSiz; i++ ) spadArray[ i ] = 0;

	/*---  select the starting SPAD #  ---*/
	*offset = 0;		/* non-aperture */
	if( spadType ) {
		/*---  skip to the first aperture spad #  ---*/
		while( ! i_is_aperture( *offset ) && *offset < VL_NumSpads ) {
			( *offset )++;
		}
	}
	for( i = 0; i < reqCount; i++ ) {
		i_get_next_good_spad( goodSpadArray, offset );
		if( *offset >= VL_NumSpads ) {
			Dev->err = VLerr_InsuffRefSpads;
			break;
		}

		/*---  confirm the SPAD found is same apertureness  ---*/
		if( i_is_aperture( *offset ) != spadType ) {
			/* can't get the required # of good spads (of correct type) */
			Dev->err = VLerr_InsuffRefSpads;
			break;
		}
		spadArray[ *offset / 8 ] |= 0x1 << ( *offset % 8 );
		(*offset)++;
	}
	i_write_ref_spad_map( Dev, spadArray );
	{
		uint8_t		checkSpadArray[ VL_SpadMapSiz ];

		/*---  read back the SPAD map array and compare it  ---*/
		i_read_ref_spad_map( Dev, checkSpadArray );

		for( i = 0; i < VL_SpadMapSiz; i++ ) {
			if( spadArray[ i ] != checkSpadArray[ i ] ) {
				if( ! Dev->err ) Dev->err = VLerr_BadSpadReadback;
			}
		}
	}
}

/*------------------------reference calibration---------------------*/

/* called from API fcn and cal.c (before spad management) */
void
vl_calibrate_ref( VL_Dev_t *Dev
) {
	i_vhv_calibration( Dev );
	vl_calibrate_phase( Dev );
}

/* Called internally and from core.c */

void
vl_calibrate_phase( VL_Dev_t *Dev
) {
	i_ref_calibration_cycle( Dev, 0x0, 0x02 );	/* single */
}

static void
i_vhv_calibration( VL_Dev_t *Dev
) {
	i_ref_calibration_cycle( Dev, 0x40, 0x01 );	/* undoc */
}

static void
i_ref_calibration_cycle( VL_Dev_t *Dev,
	uint8_t		startReg,		/* extra bits for single measurement */
	uint8_t		seqMask		/* the sequence steps for the cycle */
) {
	uint8_t		seqSave;		/* sequence step reg at entry */

	vl_rdByte(Dev, VlReg_SeqConfig, & seqSave );
	vl_wrByte( Dev, VlReg_SeqConfig, seqMask );

/* not waiting for Go bit to clear?? */
	vl_wrByte( Dev, VlReg_RangeCmd, startReg | VlReg_RangeSingle );

	vl_wait_for_ranging( Dev );
	vl_wrByte( Dev, VlReg_RangeCmd, VlReg_RangeStop );	/* why? */

	/*---  restore sequence step mask  ---*/
	vl_wrByte( Dev, VlReg_SeqConfig, seqSave );
}

/* Called only by API */

uint8_t	calMap[] = { 0xff,0x01, 0x00,0x00, 0xff,0x00, MetaStop };
uint8_t	calUnMap[] = { 0xff,0x01, 0x00,0x01, 0xff,0x00, MetaStop };

void
vl_ref_calibration_io( VL_Dev_t *Dev,
	uint8_t		read,				/* r/w, !0 = read */
	uint8_t		*pVhvSettings,		/* NULL pointer means not interested */
	uint8_t		*pPhaseCal			/* " */
) {
	vl_load_settings( Dev, calMap );
	if( pVhvSettings ) {
		if( read ) vl_rdByte( Dev, 0xcb, pVhvSettings );
		else vl_wrByte( Dev, 0xcb, *pVhvSettings );
	}
	if( pPhaseCal ) {
		if( read ) {
			vl_rdByte( Dev, 0xee, pPhaseCal );
			*pPhaseCal &= 0xef;		/* these masks don't make sense... */
		} else {
			vl_updateByte( Dev, 0xee, 0x80, *pPhaseCal );
		}
	}
	vl_load_settings( Dev, calUnMap );
}

/*-------------------spad helpers----------------*/

/* performs one reference signal rate measurement with sequence config = 0xc0
| Return pk signal rate.  This is called from spad management
*/

static void
i_ref_signal_measure( VL_Dev_t *Dev,
	uint16_t	*refSigRate
) {
	VL_RangeData_t rangeData;
	uint8_t		seqSave;		/* sequence step reg at entry */

	vl_rdByte(Dev, VlReg_SeqConfig, & seqSave );
	vl_wrByte(Dev, VlReg_SeqConfig, 0xc0);

	vl_singleMeasurementWait( Dev, & rangeData );

	*refSigRate = FP1616TOFP0907( rangeData.sigRatePeakRefMcps );
	vl_wrByte( Dev, VlReg_SeqConfig, seqSave );
}

/* Reports if a given spad index is an aperture SPAD by sensing
| the quadrant.  Only quadrant 2 is non-aperture. */

/* 64 SPADs per quadrant.  A quadrant comprises aperture or non-aperture SPADs.
| FirstSpad is a SPAD # offset that directs the device to a segment of
| 48 contiguous SPADs for reference measurements.
| FirstSpad is sent to the device at the start of a map rebuild.
| With FirstSpad=180, SPADs 0-11 are non-aperture, 12-43 are aperture.
| Tests in DeviceInit are still hard-coded to align with this 180 definition.
*/

#define RefSpad_0	0		/* non-aperture */
#define RefSpad_5	5		/* these numbers signify attenuation */
#define RefSpad_10	10

static uint32_t	refArrayQuadrants[ 4 ] = {
	RefSpad_10, RefSpad_5, RefSpad_0, RefSpad_5 };

static uint8_t
i_is_aperture(
	uint8_t		spadIndex		/* offset into device SPAD map */
) {
	uint8_t		quadrant;

	quadrant = ( FirstSpad + spadIndex ) >> 6;
	return( refArrayQuadrants[ quadrant ] == RefSpad_0 ? 0 : 1 );
}

/* Starting with the provided spad index, scan the array to find
| a bit set in the map.  Index = VL_NumSpads indicates end of map.
*/

static void
i_get_next_good_spad(
	uint8_t		spadArray[],
	uint8_t		*start			/* starting index, runner */
) {
	for( ; *start < VL_NumSpads; (*start)++ ) {
		if( spadArray[ *start / 8 ] & ( 0x1 << ( *start % 8 ) ) ) break;
	}
}

static void
i_write_ref_spad_map( VL_Dev_t *Dev,
	uint8_t		*refSpadArray
) {
	vl_wrBlock( Dev, VlReg_GlobalRefSpadEnableBase,
		refSpadArray, VL_SpadMapSiz );
}

static void
i_read_ref_spad_map( VL_Dev_t *Dev,
	uint8_t		*refSpadArray
) {
	vl_rdBlock( Dev, VlReg_GlobalRefSpadEnableBase,
		refSpadArray, VL_SpadMapSiz );
}

