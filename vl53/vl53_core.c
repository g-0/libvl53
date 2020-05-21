/*------------------------------------------------------------------------
| vl53_core.c  --  libvl53 functions
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#include "vl53_plat.h"
#include "vl53_api.h"

/*---  local function protos  ---*/
static uint16_t i_encode_timeout( uint32_t );
static uint32_t i_decode_timeout( uint16_t );
static void i_device_read_nvm( VL_Dev_t *, uint8_t, uint8_t * );

/*------------------------------------------------------------------------
| vl_get_nvram_info  --  read & cache NVM device parameters
|
| Mapped NVM registers are only available thru one 32-bit reg.
| An 8-bit address register @ 0x94, with 32-bit result register @ 0x90-93,
| provides 1024 bytes of NVM space.  Result can be read as 8/16/32 bits
*/

static uint8_t	nvMapper[] = {
	0x80,0x01, 0xff,0x01, 0x00,0x00,
	0xff,0x06, MetaUpdate,0x83,0xff,4, 0xff,0x07, 0x81,0x01,
	MetaDelay, 0x80,0x01, MetaStop
};

static uint8_t	nvUnmapper[] = {
	0x81,0x00, 0xff,0x06, MetaUpdate,0x83,~4,0 , 0xff,0x01, 0x00,0x01,
	0xff,0x00, 0x80,0x00, MetaStop
};

void
vl_get_nvram_info( VL_Dev_t *Dev,
	VL_Spads_t	*sp			/* eligible reference SPADs from nvm */
) {
	uint32_t	distMeasFP1104at40cm = 0;
	uint32_t	sigRateMeasFP0907at40cm = 0;
	uint8_t		nvBuf[ 4 ];

	vl_load_settings( Dev, nvMapper );

	/*---  SPAD data  ---*/
	i_device_read_nvm( Dev, 0x6b, nvBuf );
	sp->nSpads = nvBuf[ 2 ] & 0x7f;
	sp->tSpads = ( nvBuf[ 2 ] >> 7 ) & 0x01;

	i_device_read_nvm( Dev, 0x24, sp->refSpadMap );
	i_device_read_nvm( Dev, 0x25, NULL );
	vl_rdBlock( Dev, 0x90, sp->refSpadMap + 4, 2 );

	/*---  40cm cal point  ---*/
	i_device_read_nvm( Dev, 0x73, nvBuf );
	sigRateMeasFP0907at40cm = nvBuf[ 3 ] << 8;

	i_device_read_nvm( Dev, 0x74, nvBuf );
	sigRateMeasFP0907at40cm |= nvBuf[ 0 ];

	i_device_read_nvm( Dev, 0x75, nvBuf );
	distMeasFP1104at40cm = nvBuf[ 3 ] << 8;		/* suppress d15?  FP1104 */

	i_device_read_nvm( Dev, 0x76, nvBuf );
	distMeasFP1104at40cm |= nvBuf[ 0 ];

	/*---  undo mapping at entry  ---*/
	vl_load_settings( Dev, nvUnmapper );

	/*---  convert signal rate and store  ---*/
	VL_Device( nvmSigRateAt40cm ) =
		FP0907TOFP1616( sigRateMeasFP0907at40cm );

	/*---  store cal distance  ---*/
	VL_Device( nvmRangeAt40cmUm ) = ( distMeasFP1104at40cm * 1000 ) >> 4;
}

/* reads device version information into the caller's structure */

void
vl_get_revision_info( VL_Dev_t *Dev,
	VL_RevInfo_t	*rInf
) {
	uint8_t		nvBits[ 4 ];	/* packed ID chars */
	uint8_t		k, nv;			/* string & nvram index */
	uint8_t		carryBits = 0;	/* trailing bits from previous byte */
	uint8_t		lsbPos = 1;		/* bit # of lsb of next character */
	uint8_t		nvAddr = 0x77;	/* running NVRAM address */

	vl_load_settings( Dev, nvMapper );

	i_device_read_nvm( Dev, 0x02, NULL );
	vl_rdByte( Dev, 0x90, & rInf->moduleId );

	/*---  unpack the device ID string  ---*/
	k = nv = 0;
	while( k < VL_ProdIdLen ) {
		/* get next 4 bytes if we're empty */
		if( ! ( nv & 3 ) ) {
			i_device_read_nvm( Dev, nvAddr++, nvBits );
		}
		/*---  combine previous and next bytes for one char  ---*/
		rInf->productId[ k++ ] =
			( ( nvBits[ nv ] >> lsbPos ) | carryBits ) & 0x7f;
		carryBits = nvBits[ nv ] << ( 7 - lsbPos );
		if( lsbPos++ == 7 ) {
			lsbPos = 1;
			rInf->productId[ k++ ] = carryBits & 0x7f;	/* bonus char ! */
			carryBits = 0;
		}
		nv = ( nv + 1 ) & 0x3;
	}
	rInf->productId[ VL_ProdIdLen ] = '\0';

	i_device_read_nvm( Dev, 0x7b, NULL );
	vl_rdByte( Dev, 0x90, & rInf->revision );
	vl_rdDWord( Dev, 0x90, & rInf->uidUpper );

	i_device_read_nvm( Dev, 0x7c, NULL );
	vl_rdDWord( Dev, 0x90, & rInf->uidLower );

	/*---  undo what was done at entry  ---*/
	vl_load_settings( Dev, nvUnmapper );

	/*---  fixup: force Revision to 0 if module ID is zero  ---*/
	if( ! rInf->moduleId ) {
		rInf->revision = 0;
		strcpy( rInf->productId, "(no ID)" );
	}

	/*---  pick up unmapped version info as well  ---*/
	vl_rdByte( Dev, VlReg_IdentModel, & rInf->modelId );
	vl_rdByte( Dev, VlReg_IdentRevision, & rInf->revisionId );
}

/* Verify completion and evaluate the limit checks and
| device ranging status.  One caller from vl_getRangingResults()
*/

void
vl_get_api_range_status( VL_Dev_t *Dev,
	uint8_t			devRangeStatus,		/* from device */
	VL_RangeData_t	*pRangeData			/* user's structure */
) {
	uint8_t		sigmaLimitFail = 0;
	uint8_t		signalRefClipFail = 0;
	uint8_t		ritFail = 0;
	uint8_t		limitEnables;		/* all of them */
	uint8_t		devStat;			/* device status */
	uint8_t		rangeStat;			/* api-generated result code */

	/*---  perform s/w limit checking regardless of device status  ---*/
	limitEnables = VL_Meas( limitEnables );

#ifdef SigmaEst
	/*---  evaluate Sigma limit  ---*/
	if( limitEnables & VlLim_SigmaFinalRange ) {
		FP1616_t	limitVal = VL_Meas( limVal_sigmaFinalRange );

		/*---  compute the Sigma & Dmax and check sigma limit  ---*/
		vl_sigma_estimate( Dev, pRangeData );

		if( pRangeData->sigmaEstimate > limitVal ) sigmaLimitFail = 1;
	}
#endif

	/*---  evaluate Signal Ref limit  ---*/
	if( limitEnables & VlLim_SignalRefClip ) {
		FP1616_t	limitVal = VL_Meas( limVal_signalRefClip );

		if( pRangeData->sigRatePeakRefMcps > limitVal ) signalRefClipFail = 1;
	}

	/*---  evaluate Return Ignore Threshold limit  ---*/
 	if( limitEnables & VlLim_Rit ) {
		FP1616_t	limitVal = VL_Meas( limVal_rit );
		uint16_t	effSpadRtnCnt;		/* FP0808 */
		FP1616_t	sigRatePerSpad = 0;

		/* Compute the signal rate per spad */
		effSpadRtnCnt = pRangeData->effSpadRtnCnt;
		if( effSpadRtnCnt ) {			/* test denominator */
			sigRatePerSpad =		/* FP1616 = (FP1616 << 8) / FP0808 */
				( pRangeData->sigRateRtnMcps << 8 ) / effSpadRtnCnt;
		}
		if( sigRatePerSpad < limitVal ) ritFail = 1;
	}

	/*
	| Convert device status and s/w limits to 'range status' codes.
	| VL53L0x has good ranging when devRangeStatus = 11.
	| Also, the SigmaEstimator is not provided by the VL53L0x status,
	| so was calculated and incorporated in rangeStat.
	*/
	devStat = ( devRangeStatus & 0x78 ) >> 3;

	if( devStat == 0 || devStat == 5 || devStat == 7 || devStat == 12 ||
		devStat == 13 || devStat == 14 || devStat == 15
	) {
		rangeStat = 255;			/* no update */
	} else if( devStat == 1 || devStat == 2 || devStat == 3 ) {
		rangeStat = 5;				/* HW fail */
	} else if( devStat == 6 || devStat == 9 ) {
		rangeStat = 4;				/* Phase fail */
	} else if( devStat == 8 || devStat == 10 || signalRefClipFail ) {
		rangeStat = 3;				/* Min range */
	} else if( devStat == 4 || ritFail ) {
		rangeStat = 2;				/* Signal Fail */
	} else if( sigmaLimitFail ) {
		rangeStat = 1;				/* Sigma Fail */
	} else {
		rangeStat = 0;				/* Range Valid */
	}
	pRangeData->rangeStat = rangeStat;
}

/* Determines the register-format timeout values and writes them to the
| appropriate registers, buffering those needed for internal use.
| Affects total ranging time budget.
*/

void
vl_set_sequence_step_timeouts( VL_Dev_t *Dev
) {
	VL_TimingParams_t	*pDrv;
	uint16_t	msrcTimeoutMclks;
	uint16_t	preRangeTimeoutMClks;
	uint16_t	finalRangeTimeoutMClks;
	uint8_t		seqStepMask;		/* enabled steps */

	pDrv = & Dev->TimingParams;
	seqStepMask = VL_Timing( seqEnables );

	/*---  set up MSRC timeout (mantissa only)  ---*/
	msrcTimeoutMclks = vl_calc_timeout_mclks(
		pDrv->msrcTimeoutUs, pDrv->preRangePclks );

	if( msrcTimeoutMclks > 255 ) msrcTimeoutMclks = 256;	/* 0? */
	vl_wrByte( Dev, VlReg_MsrcTimeoutMac,
		(uint8_t) i_encode_timeout( msrcTimeoutMclks ) );

	/*---  set up Pre-Range timeout  ---*/
	preRangeTimeoutMClks = vl_calc_timeout_mclks(
		pDrv->preRangeTimeoutUs, pDrv->preRangePclks );

	vl_wrWord( Dev, VlReg_PreRangeTimeoutMac,
		i_encode_timeout( preRangeTimeoutMClks ) );

	/*---  set up Final-Range timeout  ---*/
	/* The final range timeout includes the pre-range timeout, if active.
	| To do this both final and pre-range timeouts are expressed in
	| macro periods (MClks), as they have different vcsel periods.
	*/
	finalRangeTimeoutMClks = vl_calc_timeout_mclks(
		pDrv->finalRangeTimeoutUs, pDrv->finalRangePclks );

	if( seqStepMask & SeqStepPreRange ) {
		finalRangeTimeoutMClks += preRangeTimeoutMClks;
	}
	vl_wrWord( Dev, VlReg_FinalRangeTimeoutMac,
		i_encode_timeout( finalRangeTimeoutMClks ) );
}

/* A 'get timing' helper.  It calculates the aggregate timing budget.
| It only updates the driver struct.
*/

#define StartOverheadUs			1910
#define EndOverheadUs			960

#define MsrcOverheadUs			660
#define TccOverheadUs			590
#define DssOverheadUs			690
#define PreRangeOverheadUs		660
#define FinalRangeOverheadUs	550

void
vl_get_sequence_step_timeouts( VL_Dev_t *Dev
) {
	VL_TimingParams_t	*pDrv;
	uint8_t		seqStepMask;			/* enabled steps */
	uint8_t		encodedTimeoutByte;		/* mantissa only */
	uint16_t	encodedTimeoutWord;		/* exponent:mantissa */
	uint16_t	msrcTimeoutMclks;
	uint16_t	preRangeTimeoutMclks;
	uint16_t	finalRangeTimeoutMclks;
	uint32_t	timeoutSumUs;			/* aggregate measurement time */

	pDrv = & Dev->TimingParams;
	seqStepMask = VL_Timing( seqEnables );

	/*---  get timeout used for Tcc, Dss, Msrc steps  ---*/
	vl_rdByte( Dev, VlReg_MsrcTimeoutMac, & encodedTimeoutByte );
	msrcTimeoutMclks = i_decode_timeout( encodedTimeoutByte );

	pDrv->msrcTimeoutUs = vl_calc_timeout_us(
		msrcTimeoutMclks, pDrv-> preRangePclks );

	/*---  get pre-range timeout  ---*/
	vl_rdWord( Dev, VlReg_PreRangeTimeoutMac, & encodedTimeoutWord );
	preRangeTimeoutMclks = i_decode_timeout( encodedTimeoutWord );

	pDrv->preRangeTimeoutUs = vl_calc_timeout_us(
		preRangeTimeoutMclks, pDrv->preRangePclks );

	/*---  get final-range timeout  ---*/
	vl_rdWord( Dev, VlReg_FinalRangeTimeoutMac, & encodedTimeoutWord );
	finalRangeTimeoutMclks = i_decode_timeout( encodedTimeoutWord );

	if( seqStepMask & SeqStepPreRange ) {
		finalRangeTimeoutMclks -= preRangeTimeoutMclks;
	}
	pDrv->finalRangeTimeoutUs = vl_calc_timeout_us(
		finalRangeTimeoutMclks, pDrv->finalRangePclks );

	/*---  now calculate the total measurement time  ---*/
	timeoutSumUs = StartOverheadUs + EndOverheadUs;		/* fixed overhead */

	if( seqStepMask & ( SeqStepTcc || SeqStepMsrc || SeqStepDss ) ) {
		if( seqStepMask & SeqStepTcc ) {
			timeoutSumUs += pDrv->msrcTimeoutUs + TccOverheadUs;
		}
		if( seqStepMask & SeqStepDss ) {
			timeoutSumUs += 2 * ( pDrv->msrcTimeoutUs + DssOverheadUs );
		} else if( seqStepMask & SeqStepMsrc ) {
			timeoutSumUs += pDrv->msrcTimeoutUs + MsrcOverheadUs;
		}
	}
	if( seqStepMask & SeqStepPreRange ) {
		timeoutSumUs += pDrv->preRangeTimeoutUs + PreRangeOverheadUs;
	}
	if( seqStepMask & SeqStepFinalRange ) {
		timeoutSumUs += pDrv->finalRangeTimeoutUs + FinalRangeOverheadUs;
	}
	pDrv->totalTimeUs = timeoutSumUs;
}

/*
| This is all about VCSEL pulse configuration.
|
| Step timeouts are affected by vcsel periods.
| The MSRC timeout depends on the pre-range vcsel period.
*/

#define PreVcselPclksMin		12
#define PreVcselPclksMax		18
#define FinalVcselPclksMin		8
#define FinalVcselPclksMax		14

/*
| PreRangeValidPhaseHigh		0x57
| PreRangeValidPhaseLow			0x56
*/
static uint8_t	PreRangeBlast12[] = { 0x57,0x18, 0x56,0x08, MetaStop };
static uint8_t	PreRangeBlast14[] = { 0x57,0x30, 0x56,0x08, MetaStop };
static uint8_t	PreRangeBlast16[] = { 0x57,0x40, 0x56,0x08, MetaStop };
static uint8_t	PreRangeBlast18[] = { 0x57,0x50, 0x56,0x08, MetaStop };

/*
| FinalRangeValidPhaseHigh		0x48
| FinalRangeValidPhaseLow		0x47
| GlobalVcselWidth				0x32
| AlgoPhaseCalTimeout			0x30
| AlgoPhaseCalLimit				0x130
*/

static uint8_t	FinalRangeBlast08[] = {
	0x48,0x10, 0x47,0x08, 0x32,0x02, 0x30,0x0c, 0xff,1, 0x30,0x30, 0xff,0, MetaStop };
static uint8_t	FinalRangeBlast10[] = {
	0x48,0x28, 0x47,0x08, 0x32,0x03, 0x30,0x09, 0xff,1, 0x30,0x20, 0xff,0, MetaStop };
static uint8_t	FinalRangeBlast12[] = {
	0x48,0x38, 0x47,0x08, 0x32,0x03, 0x30,0x08, 0xff,1, 0x30,0x20, 0xff,0, MetaStop };
static uint8_t	FinalRangeBlast14[] = {
	0x48,0x48, 0x47,0x08, 0x32,0x03, 0x30,0x07, 0xff,1, 0x30,0x20, 0xff,0, MetaStop };

void
vl_set_vcsel_pulse_periods( VL_Dev_t *Dev
) {
	VL_TimingParams_t	*pDrv;
	VL_VcselPer_t		nPclks;
	uint8_t				*blastPtr;

	if( Dev->err ) return;
	pDrv = & Dev->TimingParams;

	/*---  qualify the clock periods  ---*/
	if( ( pDrv->preRangePclks & 1 ) ||
		( pDrv->finalRangePclks & 1 )
	) {
		Dev->err = VLerr_InvalidParams;		/* must be an even number */
	} else if( pDrv->preRangePclks < PreVcselPclksMin ||
		pDrv->preRangePclks > PreVcselPclksMax
	) {
		Dev->err = VLerr_InvalidParams;
	} else if( pDrv->finalRangePclks < FinalVcselPclksMin ||
		pDrv->finalRangePclks > FinalVcselPclksMax
	) {
		Dev->err = VLerr_InvalidParams;
	}

	/*---  process pre-range, phase check limits  ---*/
	nPclks = pDrv->preRangePclks;
	if( nPclks == 12 ) blastPtr = PreRangeBlast12;
	else if( nPclks == 14 ) blastPtr = PreRangeBlast14;
	else if( nPclks == 16 ) blastPtr = PreRangeBlast16;
	else blastPtr = PreRangeBlast18;

	vl_load_settings( Dev, blastPtr );
	vl_wrByte( Dev, VlReg_PreRangeVcselPeriod, ( nPclks >> 1 ) - 1 );

	/*---  process final-range  ---*/
	nPclks = pDrv->finalRangePclks;
	if( nPclks == 8 ) blastPtr = FinalRangeBlast08;
	else if( nPclks == 10 ) blastPtr = FinalRangeBlast10;
	else if( nPclks == 12 ) blastPtr = FinalRangeBlast12;
	else blastPtr = FinalRangeBlast14;

	vl_load_settings( Dev, blastPtr );
	vl_wrByte( Dev, VlReg_FinalRangeVcselPeriod, ( nPclks >> 1 ) - 1 );
}

void
vl_get_vcsel_pulse_periods( VL_Dev_t *Dev
) {
	VL_TimingParams_t	*pDrv;
	uint8_t			pClkReg;

	pDrv = & Dev->TimingParams;

	/*---  read pre-range and final range registers  ---*/
	vl_rdByte( Dev, VlReg_PreRangeVcselPeriod, & pClkReg );
	pDrv->preRangePclks = ( pClkReg + 1 ) << 1;

	vl_rdByte( Dev, VlReg_FinalRangeVcselPeriod, & pClkReg );
	pDrv->finalRangePclks = ( pClkReg + 1 ) << 1;
}

/* this sets up the NVM index (0x94) and issues a strobe to cause
| an NVM transfer to a 32-bit register (0x90).
| Assumes the needed mapping/setup is in effect.
| Waits for some device ACK (which always happens 1st iteration).
| Copies reg 0x90 to caller's block (optional).  Caller may also access 0x90.
*/

static void
i_device_read_nvm( VL_Dev_t *Dev,
	uint8_t		nvInd,			/* index into NVM dwords */
	uint8_t		*nvBlk			/* store reg90 here */
) {
	uint8_t		strobe;
	uint32_t	j;

	vl_wrByte( Dev, 0x94, nvInd );
	vl_wrByte( Dev, 0x83, 0x00 );

	for( j = 0; j < 5; j++ ) {
		vl_rdByte( Dev, 0x83, & strobe );
		if( Dev->err || strobe ) {
			vl_wrByte( Dev, 0x83, 0x01 );
			if( nvBlk ) vl_rdBlock( Dev, 0x90, nvBlk, 4 );
			return;
		}
	}
	Dev->err = VLerr_Timeout;
}

/*------------------
| encode/decode timeout macro periods to/from device register.
|
| timeout is in macro periods in (lsb * 2^msb) + 1 format
*/

static uint16_t
i_encode_timeout(
	uint32_t	nClks
) {
	uint8_t		exponent = 0;

	if( nClks ) {
		--nClks;
		while( nClks > 0xff ) {
			nClks >>= 1;
			exponent++;
		}
	}
	return( ( exponent << 8 ) | nClks );
}

static uint32_t
i_decode_timeout(
	uint16_t	timeout		/* exponent:mantissa */
) {
	return( ( (uint32_t)( timeout & 0xff ) << (timeout >> 8) ) + 1 );
}

/* Works with basic address-byte pairs */

void
vl_load_settings( VL_Dev_t *Dev,
	uint8_t		*bufPtr		/* running ptr into settings */
) {
	uint8_t		addr;
	uint8_t		andVal, orVal;

	while( *bufPtr != MetaStop && ! Dev->err ) {
		addr = *bufPtr++;
		switch( addr ) {
		case MetaStop:
			return;
		case MetaDelay:
			/* possibly get time (ms) from next byte */
			vl_pollDelay( Dev );
			break;
		case MetaUpdate:
			addr = *bufPtr++;
			andVal = *bufPtr++;
			orVal = *bufPtr++;
			vl_updateByte( Dev, addr, andVal, orVal );
			break;
		default:
			vl_wrByte( Dev, addr, *bufPtr++ );
			break;
		}
	}
}

/*----------------
| Tuning settings and interrupt threshold settings
| "update 02/11/2015_v36"
*/

uint8_t
defaultTuning[] = {

	0xff,0x01, 0x00,0x00, 0xff,0x00,

	0x09,0x00,				/* range, non-fraction */
	0x10,0x00, 0x11,0x00,	/* ? */
	0x24,0x01, 0x25,0xff,	/* ? */
	0x75,0x00,				/* ? */

	0xff,0x01, 0x4e,0x2c, 0x48,0x00, 0x30,0x20, 0xff,0x00,

	0x30,0x09,	/* algo phase cal timeout */
	0x54,0x00,	/* ? */
	0x31,0x04,	/* ? */
	0x32,0x03,	/* global vcsel width */
	0x40,0x83,	/* ? */
	0x46,0x25,	/* MSRC step timeout */
	0x60,0x12,	/* sigrate MSRC & pre-range limit enables =disabled */
	0x27,0x00,	/* pre-range min SNR */
	0x50,0x06, 0x51,0x00, 0x52,0x96,	/* pre-range vcsel, step timeout(2) */
	0x56,0x08, 0x57,0x30,	/* pre-range valid phase low, high */
	0x61,0x00, 0x62,0x00,	/* pre-range sigma threshold(2) */
	0x64,0x00, 0x65,0x00,	/* pre-range sigrate low limit(2) */
	0x66,0xa0,	/* ? */

	0xff,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xff, 0x4a,0x00, 0xff,0x00,

	0x7a,0x0a, 0x7b,0x00, 0x78,0x21,	/* ? */

	0xff,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xff, 0x45,0x26, 0x46,0x05,
	0x40,0x40, 0x0E,0x06, 0x20,0x1a, 0x43,0x40, 0xff,0x00,

	0x34,0x03, 0x35,0x44,	/* ? */

	0xff,0x01, 0x31,0x04, 0x4b,0x09, 0x4c,0x05, 0x4d,0x04, 0xff,0x00,

	0x44,0x00, 0x45,0x20,	/* final sigrate low limit(2) =0.25 */
	0x47,0x08, 0x48,0x28,	/* final valid phase low, high */
	0x67,0x00,				/* final min SNR */
	0x70,0x04, 0x71,0x01, 0x72,0xfe,	/* final vcsel, step timeout(2) */
	0x76,0x00, 0x77,0x00,	/* ? */
	0x28,0x00, 0x29,0xc8,	/* offset=5cm */

	0xff,0x01, 0x0d,0x01, 0xff,0x00,	/* ? */

	0x80,0x01,	/* power to IDLE */
	0x01,0xe8,	/* sequence enables = DSS, PRE, FINAL */

	0xff,0x01, 0x8e,0x01, 0x00,0x01, 0xff,0x00,		/* ? */
	MetaStop
};

/* looks to be in a different map space */
/* some suspicious sequences, but impossible to be sure without docs */
uint8_t
InterruptThresholdSettings[] = {
	0xff,0x00, 0x80,0x01, 0xff,0x01, 0x00,0x00, 0xff,0x01, 0x4f,0x02,

	0xff,0x0e,
	0x00,0x03, 0x01,0x84, 0x02,0x0A, 0x03,0x03, 0x04,0x08, 0x05,0xC8,
	0x06,0x03, 0x07,0x8D, 0x08,0x08, 0x09,0xC6, 0x0A,0x01, 0x0B,0x02,
	0x0C,0x00, 0x0D,0xD5, 0x0E,0x18, 0x0F,0x12,
	0x10,0x01, 0x11,0x82, 0x12,0x00, 0x13,0xD5, 0x14,0x18, 0x15,0x13,
	0x16,0x03, 0x17,0x86, 0x18,0x0A, 0x19,0x09, 0x1A,0x08, 0x1B,0xC2,
	0x1C,0x03, 0x1D,0x8F, 0x1E,0x0A, 0x1F,0x06,
	0x20,0x01, 0x21,0x02, 0x22,0x00, 0x23,0xD5, 0x24,0x18, 0x25,0x22,
	0x26,0x01, 0x27,0x82, 0x28,0x00, 0x29,0xD5, 0x2A,0x18, 0x2B,0x0B,
	0x2C,0x28, 0x2D,0x78, 0x2E,0x28, 0x2F,0x91,
	0x30,0x00, 0x31,0x0B, 0x32,0x00, 0x33,0x0B, 0x34,0x00, 0x35,0xA1,
	0x36,0x00, 0x37,0xA0, 0x38,0x00, 0x39,0x04, 0x3A,0x28, 0x3B,0x30,
	0x3C,0x0C, 0x3D,0x04, 0x3E,0x0F, 0x3F,0x79,
	0x40,0x28, 0x41,0x1E, 0x42,0x2F, 0x43,0x87, 0x44,0x00, 0x45,0x0B,
	0x46,0x00, 0x47,0x0B, 0x48,0x00, 0x49,0xA7, 0x4A,0x00, 0x4B,0xA6,
	0x4C,0x00, 0x4D,0x04, 0x4E,0x01, 0x4F,0x00,
	0x50,0x00, 0x51,0x80, 0x52,0x09, 0x53,0x08, 0x54,0x01, 0x55,0x00,
	0x56,0x0F, 0x57,0x79, 0x58,0x09, 0x59,0x05, 0x5A,0x00, 0x5B,0x60,
	0x5C,0x05, 0x5D,0xD1, 0x5E,0x0C, 0x5F,0x3C,
	0x60,0x00, 0x61,0xD0, 0x62,0x0B, 0x63,0x03, 0x64,0x28, 0x65,0x10,
	0x66,0x2A, 0x67,0x39, 0x68,0x0B, 0x69,0x02, 0x6A,0x28, 0x6B,0x10,
	0x6C,0x2A, 0x6D,0x61, 0x6E,0x0C, 0x6F,0x00,
	0x70,0x0F, 0x71,0x79, 0x72,0x00, 0x73,0x0B, 0x74,0x00, 0x75,0x0B,
	0x76,0x00, 0x77,0xA1, 0x78,0x00, 0x79,0xA0, 0x7A,0x00, 0x7B,0x04,

	0xFF,0x04, 0x79,0x1D, 0x7B,0x27, 0x96,0x0E, 0x97,0xFE, 0x98,0x03,
	0x99,0xEF, 0x9A,0x02, 0x9B,0x44, 0x73,0x07, 0x70,0x01,

	0xff,0x01, 0x00,0x01, 0xff,0x00,
	MetaStop
};

/*------------------------------------------------------------------------
| vl_run_reg_rw  --  read/write the mysterious 0x191 register
|
| Lets face it.  Without any docs, *every* register of the VL53L0X has some
| level of mystery.  But 0x191 is special.  It has an unassignable value
| to enable cycling, and is used to stop the repeated ranging modes.
|
*/

void
vl_run_reg_rw( VL_Dev_t *Dev,
	uint8_t		*rd,		/* destination for read, or NULL */
	uint8_t		wr			/* write this if rd is NULL */
) {
	vl_wrByte( Dev, 0xff, 0x01 );
	vl_wrByte( Dev, 0x00, 0x00 );
	if( rd )
		vl_rdByte( Dev, 0x91, rd );
	else
		vl_wrByte( Dev, 0x91, wr );
	vl_wrByte( Dev, 0x00, 0x01 );
	vl_wrByte( Dev, 0xff, 0x00 );
}

/* Waits for measurement ready */
#define VL_MaxWaitTimeMs		1000		/* WAG */

void
vl_wait_for_ranging( VL_Dev_t *Dev
) {
	uint8_t		ready = 0;
	uint16_t	k;

	for( k = 0; k < VL_MaxWaitTimeMs/VL_PollDelayMs; k++ ) {
		vl_getRangingDataReady( Dev, & ready );
		if( Dev->err || ready ) return;
		vl_pollDelay( Dev );
	}
	Dev->err = VLerr_Timeout;
}

/*---------------------
| vl_calc_macro_period_ps  --  calculate macro period from # of PLL clocks
| vl_calc_timeout_mclks  --  convert us to # of mclks
| vl_calc_timeout_us  --  convert # of mclks to us
|
| A macro period is a fixed multiple of PLL cycles * vcsel period.
| The vcsel period is not the same for all steps.
| Step timeouts are expressed in macro periods.
*/

#define MacroPeriodVclks	2304		/* magic */

uint32_t
vl_calc_macro_period_ps(
	VL_VcselPer_t	vcsel_period_pclks
) {
	return( vcsel_period_pclks * MacroPeriodVclks * PllPeriod_ps );
}

uint32_t
vl_calc_timeout_mclks(
	uint32_t	timeout_period_us,
	VL_VcselPer_t	vcsel_period_pclks
) {
	uint32_t	macro_period_ns;

	macro_period_ns = vl_div(
		vl_calc_macro_period_ps( vcsel_period_pclks ), 1000 );
    return( vl_div( timeout_period_us * 1000, macro_period_ns ) );
}

uint32_t
vl_calc_timeout_us(
	uint32_t	timeout_period_mclks,
	VL_VcselPer_t	vcsel_period_pclks
) {
	uint32_t	macro_period_ns;

	macro_period_ns = vl_div(
		vl_calc_macro_period_ps( vcsel_period_pclks ), 1000 );
	return( vl_div( timeout_period_mclks * macro_period_ns, 1000 ) );
}

void
vl_rev_bytes(
	uint8_t		*data,
	uint8_t		size
) {
	uint8_t		tempData;
	uint8_t		mirrorIndex;
	uint8_t		index;

	for( index = 0; index < size/2; index++ ) {
		mirrorIndex = size - 1 - index;
		tempData = data[ index ];
		data[ index ] = data[ mirrorIndex ];
		data[ mirrorIndex ] = tempData;
	}
}

uint32_t
vl_div(
	uint32_t	a,
	uint32_t	b
) {
	return( ( a + b/2 ) / b );
}

