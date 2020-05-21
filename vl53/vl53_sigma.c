/*------------------------------------------------------------------------
| vl53_sigma.h  --  libvl53 sigma and dmax estimation
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

/*------------------------------------------------------------------------
| vl_sigma_estimate  --  estimate the range deviation
| i_calc_dmax  --  calculate max reliable range for conditions
|
| Called to evaluate a sigma limit check when retrieving ranging data.
| Some of the constants used and operations are still not understood.
| Validation of the algorithm is left as an exercise for the reader...
| (Sig = Signal, Eff = Effective, Est = Estimated, Amb=Ambient)
|
| There are two versions of the functions: a cleaned up integer version and
| a new floating point version.
*/

#include <math.h>
#include "vl53_plat.h"
#include "vl53_api.h"

#define sigmalog( f, ... )		//printf( f, ##__VA_ARGS__ )

#ifdef SigmaEst
#ifdef SigmaFloat /*{*/
/*---  floating point sigma & dmax calculation  ---*/
static uint16_t i_calc_dmax( VL_Dev_t *, float, float, float, float, uint32_t );

#define EffWidthPulse_cNs	800				/* 100ths of ns */
#define EffWidthAmb_cNs		600				/* for sigma est, 100ths of ns */
#define EffWidthAmbDmax_cNs	700				/* for dmax est */
#define WidthVcsel_ps		4700			/* pico sec (2.84 PLL clks) */
#define DfltFinalRangeTimeUs	25000.0		/* 25 ms integration time */

#define XtalkMaxMcps		0.05			/* 50 kcps */
//#define AmbToSigRatioMax	(0xf0000000/EffWidthAmbDmax_cNs)	/* ~100 FP1616 */
#define AmbToSigRatioMax	100.0
#define SigmaEstRtnMax		100.0			/* mm */
#define SigmaEstMax			655.53			/* mm */

#define Vol					2.997			/* C, x 1e8 m/s */
#define Tof_psPerMm			6.6				/* 6.6 psec/mm round trip */

void
vl_sigma_estimate( VL_Dev_t *Dev,
	VL_RangeData_t *pRangeData
) {
	float		sigRateRtn_mcps;		/* return signal */
	float		sigRateXtalkComp_mcps;	/* total comp, all SPADS */
	float		sigRateTotal_mcps;		/* return signal + total xtalk comp */

	uint32_t	peakVcselDuration_us;
	float		vcselTotalEventsRtn;
	uint32_t	finalRangeInteg_us;		/* integration time */

	float		pwMult;
	float		ambToSigRatio;			/* ambient/total signal ratio */
	float		sigmaEst;

	/*---  total xTalk CompRate = # spads * comp parameter  ---*/
	sigRateXtalkComp_mcps =		/* FP0824 = FP0808 * FP1616 */
		( pRangeData->effSpadRtnCnt * VL_Cal( xTalkCompRateMcps ) ) /
			(float)( 1 << 24 );

	sigRateRtn_mcps = pRangeData->sigRateRtnMcps / 65536.0;
	sigRateTotal_mcps = sigRateRtn_mcps + sigRateXtalkComp_mcps;
//	if( sigRateXtalkComp_mcps > XtalkMaxMcps ) {
//		sigRateXtalkComp_mcps = XtalkMaxMcps;		/* per SPAD ? */
//	}
	ambToSigRatio = ( pRangeData->sigRateAmbRtnMcps / 65536.0 ) /
		sigRateTotal_mcps;

	/* clip to prevent overflow and ensure safe max result */
	if( ambToSigRatio > AmbToSigRatioMax ) ambToSigRatio = AmbToSigRatioMax;
sigmalog( "sigRateTotal=%f, sigRateRtn=%f\n",
	sigRateTotal_mcps, sigRateRtn_mcps );
sigmalog( "ambToSigRatio: %f\n", ambToSigRatio );
{
	uint32_t	preRangeUs, finalRangeUs;	/* step timeouts */
	uint32_t	preRangeMclks;				/* macro PLL clks */
	uint32_t	finalRangeMclks;
	uint32_t	vcselWidth;

	/*---  calculate pre-range and final range macro periods  ---*/
	preRangeUs = VL_Timing( preRangeTimeoutUs );
	finalRangeUs = VL_Timing( finalRangeTimeoutUs );
	finalRangeInteg_us = finalRangeUs + preRangeUs;

	preRangeMclks = vl_calc_timeout_mclks( preRangeUs,
		VL_Timing( preRangePclks ) );
	finalRangeMclks = vl_calc_timeout_mclks( finalRangeUs,
		VL_Timing( finalRangePclks ) );

	vcselWidth = ( VL_Timing( finalRangePclks ) == 8 ) ? 2 : 3;

	peakVcselDuration_us = vcselWidth * 2048 *
		( preRangeMclks + finalRangeMclks );
	peakVcselDuration_us = vl_div( peakVcselDuration_us, 1000 );
	peakVcselDuration_us *= PllPeriod_ps;
	peakVcselDuration_us = vl_div( peakVcselDuration_us, 1000 );
}
	vcselTotalEventsRtn = sigRateTotal_mcps * peakVcselDuration_us;

	if( sigRateTotal_mcps == 0 || vcselTotalEventsRtn == 0 ) {
		pRangeData->sigmaEstimate = SigmaEstMax * 65536.0;
		pRangeData->RangeMaxMm = 0;
		return;
	}
{
	float		tof_ps = pRangeData->rangeMm * Tof_psPerMm;
	float		xTalkCorrection;

	/*---  vcselRate -/+ xtalkCompRate  ---*/
	xTalkCorrection = ( sigRateRtn_mcps - sigRateXtalkComp_mcps ) /
		( sigRateRtn_mcps + sigRateXtalkComp_mcps );

	pwMult = 1.0 + ( tof_ps / WidthVcsel_ps ) * ( 1.0 - xTalkCorrection );
	pwMult *= pwMult;
}
{
	float		sqr1, sqr2;
	float		sqrtResult_cNs;
	float		sigmaEstRtn;			/* mm */
	float		sigmaEstRef;			/* mm */

	sqr1 = powf( pwMult * EffWidthPulse_cNs, 2 );
	sqr2 = powf( ambToSigRatio * EffWidthAmb_cNs, 2 );
	sqrtResult_cNs = sqrtf( sqr1 + sqr2 );

	sigmaEstRtn = ( sqrtResult_cNs * Vol ) /		/* Vol is mm/cNs */
		( 2.0 * sqrtf( vcselTotalEventsRtn * 12.0 ) );

	/* clip to prevent overflow and ensure safe max result */
	if( sigmaEstRtn > SigmaEstRtnMax ) sigmaEstRtn = SigmaEstRtnMax;

	/* sigmaEstRef = 1mm * 25ms/final range integration time (inc pre-range) */
	sigmaEstRef = sqrtf( DfltFinalRangeTimeUs / finalRangeInteg_us );

	sigmaEst = sqrtf( powf( sigmaEstRtn, 2 ) + powf( sigmaEstRef, 2 ) );
}
	if( sigmaEst > SigmaEstMax ) sigmaEst = SigmaEstMax;
	pRangeData->sigmaEstimate = ( sigmaEst * 65536.0 );

	pRangeData->RangeMaxMm = i_calc_dmax( Dev,
		sigRateRtn_mcps,
		sigRateTotal_mcps,
		pwMult, ambToSigRatio,
		peakVcselDuration_us
	);
sigmalog( "Sigma Est: %f, DmaxMm: %d\n",
	pRangeData->sigmaEstimate / 65536.0, pRangeData->RangeMaxMm );
}

#define SigmaLimit			18.0		/* mm */
#define SigmaEstRef			1.0			/* mm */
#define SigRateLimit		0.25

static uint16_t
i_calc_dmax( VL_Dev_t *Dev,
	float		sigRateRtn_mcps,
	float		sigRateTotal_mcps,
	float		pwMult,
	float		ambToSigRatio,			/* limit checked */
	uint32_t	peakVcselDuration_us
) {
	float		sigAt0mm;
	float		p1, p2, p3, p4;
	float		minSigNeeded;
	uint16_t	dmaxDark, dmaxAmb;

	sigAt0mm = ( VL_Cal( dmaxCalSigRateMcps ) / 65536.0 ) *
		powf( VL_Cal( dmaxCalRangeMm ), 2 );

	p1 = powf( pwMult * EffWidthPulse_cNs, 2 );

	/* dmax ambient width is different from sigma */
	p2 = powf( ambToSigRatio * EffWidthAmbDmax_cNs, 2 );

	if( sigRateRtn_mcps == 0.0 ) {
		p3 = 0.0;
	} else {
		p3 = powf( ( sigRateTotal_mcps / sigRateRtn_mcps ) * Vol, 2 );
	}
	p4 = 4 * 12 * ( powf( SigmaLimit, 2 ) - powf( SigmaEstRef, 2 ) );

	minSigNeeded = ( p1 + p2 ) / peakVcselDuration_us;
	minSigNeeded *= p3 / p4;

	dmaxDark = sqrtf( sigAt0mm / SigRateLimit );
	dmaxAmb = minSigNeeded ? sqrtf( sigAt0mm / minSigNeeded ) : 0;

sigmalog( "sigAt0mm=%f\n", sigAt0mm );
sigmalog( "pwMult: %f\n", pwMult );
sigmalog( "p1=%f\n", p1 );
sigmalog( "p2=%f\n", p2 );
sigmalog( "p3=%f\n", p3 );
sigmalog( "p4=%f\n", p4 );
sigmalog( "minSigNeeded=%f\n", minSigNeeded );
sigmalog( "dmax: Dark=%u, Amb=%u\n", dmaxDark, dmaxAmb );
	return( dmaxDark < dmaxAmb ? dmaxDark : dmaxAmb );
}

#else /*}{*/
/*---  integer sigma & dmax calculation  ---*/
static uint16_t i_calc_dmax( VL_Dev_t *, FP1616_t, FP1616_t, FP1616_t,
	FP1616_t, uint32_t );
static uint32_t i_sqrt( uint32_t );
static uint32_t i_sq( uint32_t );
static uint32_t i_shr16( uint32_t );

#define EffWidthPulse_cNs	800				/* 100ths of ns */
#define EffWidthAmb_cNs		600				/* for sigma est, 100ths of ns */
#define EffWidthAmbDmax_cNs	700				/* for dmax est */
#define WidthVcsel_ps		4700			/* pico sec (2.84 PLL clks) */
#define DfltFinalRangeTimeUs	0x61a80000	/* 25 ms FP1616 integration time */

#define XtalkMaxMcps		3277			/* 50 kcps FP1616 */
#define AmbToSigRatioMax	(0xf0000000/EffWidthAmbDmax_cNs)	/* ~100 FP1616 */
#define SigmaEstRtnMax		0x1999			/* 0.100 m, FP1616 */
#define SigmaEstMax			0x028F87AE		/* 655.53 mm, FP1616 */

#define One					0x10000			/* 1.0 FP1616 */
#define Vol					2997			/* C, x 1e5 m/s */
#define PllPeriod_ps		1655			/* (604.23MHz) FP3200 */
#define Tof_psPerMm			0x0006999A		/* 6.6 psec/mm round trip, FP1616 */

void
vl_sigma_estimate( VL_Dev_t *Dev,
	VL_RangeData_t *pRangeData
) {
	FP1616_t	sigRateRtn_mcps;		/* return signal */
	FP1616_t	sigRateXtalkComp_mcps;	/* total comp, all SPADS */
	FP1616_t	sigRateTotal_mcps;		/* return signal + total xtalk comp */
	FP1616_t	ambToSigRatio;			/* ambient/total signal ratio */

	uint32_t	peakVcselDuration_us;
	uint32_t	vcselTotalEventsRtn;
	uint32_t	finalRangeInteg_us;		/* integration time */

	FP1616_t	pwMult;
	FP1616_t	sigmaEst;

	/*---  total xTalkCompRate = # spads * comp parameter  ---*/
	sigRateXtalkComp_mcps =			/* FP0824 = FP0808 * FP1616 */
		pRangeData->effSpadRtnCnt * VL_Cal( xTalkCompRateMcps );
	/* FP1616 = FP0824 >> 8 */
	sigRateXtalkComp_mcps = ( sigRateXtalkComp_mcps + 0x80 ) >> 8;

	/*---  calc total signal, and ambient/total ratio  ---*/
	sigRateRtn_mcps = pRangeData->sigRateRtnMcps;
	sigRateTotal_mcps = sigRateRtn_mcps + sigRateXtalkComp_mcps;
//	if( sigRateXtalkComp_mcps > XtalkMaxMcps ) {	/* per SPAD ? */
//		sigRateXtalkComp_mcps = XtalkMaxMcps;
//	}
	/* FP1616 = FP1616 / (FP1616 >> 16) */
	ambToSigRatio = ( pRangeData->sigRateAmbRtnMcps * 1000 ) /
		i_shr16( sigRateTotal_mcps * 1000 );	/* denominator test? */

	/* clip to prevent overflow and ensure safe max result */
	if( ambToSigRatio > AmbToSigRatioMax ) ambToSigRatio = AmbToSigRatioMax;
sigmalog( "sigRateTotal=%f, sigRateRtn=%f\n",
	sigRateTotal_mcps / 65536.0, sigRateRtn_mcps / 65536.0 );
sigmalog( "ambToSigRatio: %f\n", ambToSigRatio / 65536.0 );
{
	uint32_t	preRangeUs, finalRangeUs;	/* step timeouts */
	uint32_t	preRangeMclks;				/* macro PLL clks */
	uint32_t	finalRangeMclks;
	uint32_t	vcselWidth;

	/*---  calculate pre-range and final range macro periods  ---*/
	preRangeUs = VL_Timing( preRangeTimeoutUs );
	finalRangeUs = VL_Timing( finalRangeTimeoutUs );
	finalRangeInteg_us = finalRangeUs + preRangeUs;

	preRangeMclks = vl_calc_timeout_mclks( preRangeUs,
		VL_Timing( preRangePclks ) );
	finalRangeMclks = vl_calc_timeout_mclks( finalRangeUs,
		VL_Timing( finalRangePclks ) );

	vcselWidth = ( VL_Timing( finalRangePclks ) == 8 ) ? 2 : 3;

	peakVcselDuration_us = vcselWidth * 2048 *
		( preRangeMclks + finalRangeMclks );
	peakVcselDuration_us = vl_div( peakVcselDuration_us, 1000 );
	peakVcselDuration_us *= PllPeriod_ps;
	peakVcselDuration_us = vl_div( peakVcselDuration_us, 1000 );
}
	/* FP2408 = (FP1616 >> 8) * FP3200 */
	vcselTotalEventsRtn = ( ( sigRateTotal_mcps + 0x80 ) >> 8 )
		* peakVcselDuration_us;

	/* FP3200 = FP2408 >> 8 */
	vcselTotalEventsRtn = ( vcselTotalEventsRtn + 0x80 ) >> 8;

	if( ! sigRateTotal_mcps || ! vcselTotalEventsRtn ) {
		pRangeData->sigmaEstimate = SigmaEstMax;
		pRangeData->RangeMaxMm = 0;
		return;
	}
{
	FP1616_t	tof_ps = pRangeData->rangeMm * Tof_psPerMm;	/* FP1616 */
	FP1616_t	diff1_mcps;
	FP1616_t	diff2_mcps;
	FP1616_t	xTalkCorrection;

	/*---  vcselRate -/+ xtalkCompRate  ---*/
	diff1_mcps = sigRateRtn_mcps - sigRateXtalkComp_mcps;	/* all FP1616 */
	diff2_mcps = sigRateRtn_mcps + sigRateXtalkComp_mcps;

	diff1_mcps <<= 8;							/* FP0824 = FP1616 << 8 */
	xTalkCorrection	= diff1_mcps / diff2_mcps;	/* FP2408 = FP0824 / FP1616 */
	xTalkCorrection <<= 8;						/* FP1616 = FP2408 << 8 */

	/* FP0032 = (FP1616 / FP3200) * (FP1616 - FP1616) */
	pwMult = ( tof_ps / WidthVcsel_ps ) * ( One - xTalkCorrection );
	pwMult = One + i_shr16( pwMult ); /* FP1616 = FP1616 + (FP0032 >> 16) */
	/*
	| pwMult is now >1.  Squaring the value will exceed 32 bits.
	| Perform a single right shift before multiplication (limit is then 4.0).
	*/
	pwMult = i_sq( pwMult >> 1 );			/* FP0230 = (FP1616 >> 1)^2 */
	pwMult = ( pwMult + 0x2000 ) >> 14;		/* FP1616 = FP0230 >> 14 */
}
{
	FP1616_t	sqr1;
	FP1616_t	sqr2;
	FP1616_t	sqrtResult_cNs;
	FP1616_t	sqrtResult;
	FP1616_t	sigmaEstRtn;		/* m */
	FP1616_t	sigmaEstRef;		/* m */

	sqr1 = pwMult * EffWidthPulse_cNs;		/* FP1616 = FP1616 * FP3200 */
	sqr1 = i_sq( i_shr16( sqr1 ) );			/* FP3200 = (FP1616 >> 16)^2 */

	sqr2 = ambToSigRatio * EffWidthAmb_cNs;	/* FP1616 = FP1616 * FP3200 */
	sqr2 = i_sq( i_shr16( sqr2 ) );			/* FP3200 = (FP1616 >> 16)^2 */

	sqrtResult_cNs = i_sqrt( sqr1 + sqr2 );	/* FP3200 = sqrt(FP3200+FP3200) */
	sqrtResult_cNs <<= 16;					/* FP1616 = FP3200 << 16 */

	sigmaEstRtn = vl_div( sqrtResult_cNs, 100 ) /
		( 2 * i_sqrt( vcselTotalEventsRtn * 12 ) );		/* overflow poss */
	sigmaEstRtn	= vl_div( sigmaEstRtn * Vol, 10000 );	/* now m/ns */

	/* clip to prevent overflow and ensure safe max result */
	if( sigmaEstRtn > SigmaEstRtnMax ) sigmaEstRtn = SigmaEstRtnMax;

	/* sigmaEstRef = 1mm * 25ms/final range integration time (inc pre-range) */
	/* FP2408 = sqrt( FP1616 / FP3200 ) */
	sigmaEstRef = i_sqrt( vl_div( DfltFinalRangeTimeUs, finalRangeInteg_us ) );
	sigmaEstRef = vl_div( sigmaEstRef << 8, 1000 );	/* FP1616 = FP2408 << 8 */

	sqr1 = i_sq( sigmaEstRtn );			/* FP0032 = FP1616^2 */
	sqr2 = i_sq( sigmaEstRef );			/* FP0032 = FP1616^2 */

	sqrtResult = i_sqrt( sqr1 + sqr2 );	/* FP1616 = sqrt( FP0032 + FP0032 ) */
	sigmaEst = sqrtResult * 1000;
}
	if( sigmaEst > SigmaEstMax ) sigmaEst = SigmaEstMax;
	pRangeData->sigmaEstimate = sigmaEst;

	pRangeData->RangeMaxMm = i_calc_dmax( Dev,
		sigRateRtn_mcps,
		sigRateTotal_mcps,
		pwMult, ambToSigRatio,
		peakVcselDuration_us
	);
sigmalog( "Sigma Est: %f, DmaxMm: %d\n",
	pRangeData->sigmaEstimate / 65536.0, pRangeData->RangeMaxMm );
}

#define SigmaLimit		0x049c		/* 0.018 um, FP1616 use limit value? */
#define SigmaEstRef		0x0042		/* 0.001 um, FP1616 */
#define SigRateLimit	0x0040		/* 0.25 mcps, FP2408 */

static uint16_t
i_calc_dmax( VL_Dev_t *Dev,
	FP1616_t	sigRateRtn_mcps,		/* 'corrected' signal rate */
	FP1616_t	sigRateTotal_mcps,
	FP1616_t	pwMult,
	FP1616_t	ambToSigRatio,			/* limit checked */
	uint32_t	peakVcselDuration_us
) {
	uint32_t	sigAt0mm;		/* FP2408 */
	uint32_t	p1, p2;
	FP1616_t	p3;				/* FP2804 */
	FP1616_t	p4;				/* FP1814 */
	FP1616_t	minSigNeeded;	/* multiple formats */

	/* FP1616 = FP1616 * uint32 */
	sigAt0mm = VL_Cal( dmaxCalSigRateMcps ) * VL_Cal( dmaxCalRangeMm );
	sigAt0mm = ( sigAt0mm + 0x80 ) >> 8;	/* FP2408 = FP1616 >> 8 */
	sigAt0mm *= VL_Cal( dmaxCalRangeMm );

	/* FP3200 = (FP1616 >> 16)^2 */
	p1 = i_sq( i_shr16( pwMult * EffWidthPulse_cNs ) );

	/* dmax uses a different ambient width from sigma */
	/* FP3200 = ( ( FP1616 * FP3200 ) >> 16)^2 */
	p2 = i_sq( i_shr16( ambToSigRatio * EffWidthAmbDmax_cNs ) );

	if( ! sigRateRtn_mcps ) {
		p3 = 0;
	} else {
		/* pre-shift to increase resolution prior to the division */
		/* FP2210 = ( FP0626 = FP1616 << 10 ) / FP1616 */
		p3 = vl_div( sigRateTotal_mcps << 10, sigRateRtn_mcps );

		/* Apply a scaled version of C. Corrected later */
		/* FP2804 = ( FP1220 = FP2210 * FP2210 ) >> 16 */
		p3 = i_shr16( i_sq( 3 * p3 ) );
	}

	/* FP0032 = FP3200 * ( (FP1616)^2 - (FP1616)^2 ) */
	p4 = 4 * 12 * ( i_sq( SigmaLimit ) - i_sq( SigmaEstRef ) );
	p4 = ( p4 + 0x20000 ) >> 18;				/* FP1814 = FP0032 >> 18 */

	minSigNeeded = vl_div( p1 + p2, peakVcselDuration_us );	/* all FP3200 */
	minSigNeeded <<= 14;						/* FP1814 = FP3200 << 14 */

	minSigNeeded = vl_div( minSigNeeded, p4 );	/* FP3200 = FP1814 / FP1814 */
	minSigNeeded *= p3;							/* FP2804 = FP3200 * FP2804 */

	/* 10e-6 on the numerator and 10e-18 on the denominator.  Divide by
	| 1000000 to get 10e6 (mcps).
	*/
	minSigNeeded = vl_div( minSigNeeded, 1000 );		/* FP2804 */
	minSigNeeded = vl_div( minSigNeeded << 4, 1000 );	/* FP2408 */
{
	uint16_t	dmaxDark;
	uint16_t	dmaxAmb;

	/* uint32 = FP2408 / FP2408 */
	dmaxDark = i_sqrt( vl_div( sigAt0mm, SigRateLimit ) );

	/* uint32 = FP2408 / FP2408 */
	dmaxAmb = minSigNeeded ? i_sqrt( vl_div( sigAt0mm, minSigNeeded ) ) : 0;

sigmalog( "sigAt0mm=%f\n", sigAt0mm / 256.0 );
sigmalog( "pwMult: %f\n", pwMult / 65536.0 );
sigmalog( "p1=%u\n", p1 );
sigmalog( "p2=%u\n", p2 );
sigmalog( "p3=%f\n", p3 / 16.0 );
sigmalog( "p4=%f\n", p4 / (float)( 1 << 14 ) );
sigmalog( "minSigNeeded=%f\n", minSigNeeded / 256.0 );
sigmalog( "dmax: Dark=%u, Amb=%u\n", dmaxDark, dmaxAmb );
	return( dmaxDark < dmaxAmb ? dmaxDark : dmaxAmb );
}
}

/* calc integer square root
| From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
*/
static uint32_t
i_sqrt(
	uint32_t	num
) {
	uint32_t	res = 0;
	uint32_t	bit = 1 << 30;
	/* The second-to-top bit is set:
	| 1 << 14 for 16-bits, 1 << 30 for 32 bits */

	 /* "bit" starts at the highest power of four <= the argument. */
	while( bit > num ) bit >>= 2;

	while( bit ) {
		if (num >= res + bit) {
			num -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

static uint32_t
i_sq(
	uint32_t	a
) {
	return( a * a );
}

static uint32_t
i_shr16(
	uint32_t	a
) {
	return( ( a + 0x8000 ) >> 16 );
}
#endif /*}*/
#endif
