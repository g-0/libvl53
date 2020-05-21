/*------------------------------------------------------------------------
| vl53_strings.c  --  libvl53 identifier names
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#include "vl53_plat.h"
#include "vl53_api.h"

#ifdef AllStrings

char *
vl_revisionString(
	uint8_t		revision		/* from NVM */
) {
	if( revision == 0 )						return( "VL53L0X TS0" );
	if( revision <= 34 && revision != 32 )	return( "VL53L0X TS1" );
	if( revision < 39 )						return( "VL53L0X TS2" );
	return( "VL53L0X ES1 or later" );
}

char *
vl_apiErrorString(
	VL_Err_t	apiError
) {
	switch( apiError ) {
	case VLerr_None:				return( "No Error" );
	case VLerr_NoValidRange:		return( "Calibration Ranging Fail" );
	case VLerr_InvalidParams:		return( "Invalid Parameters" );
	case VLerr_InterruptNotCleared:	return( "Interrupt not Cleared" );
	case VLerr_BadSpadNumber:		return( "Invalid SPAD Number" );
	case VLerr_InsuffRefSpads:		return( "Insufficient Ref SPADs" );
	case VLerr_RangeError:			return( "Range Fail" );
	case VLerr_Timeout:				return( "Timeout" );
	case VLerr_UnknownMode:			return( "Unknown Mode" );
	case VLerr_UnknownGpioPin:		return( "Invalid GPIO Pin" );
	case VLerr_UnknownGpioFunc:		return( "Invalid GPIO Function" );
	case VLerr_I2cBusy:				return( "I2C Controller Busy" );
	case VLerr_I2cFailedXact:		return( "I2C Transaction Fail" );
	case VLerr_I2cCollision:		return( "I2C Collision" );
	case VLerr_I2cDeviceError:		return( "I2C Device Fail" );
	case VLerr_BadSpadReadback:		return( "Reference Spad Init Fail" );
	case VLerr_NotAvailable:		return( "Not implemented" );
	}
	return( "?undefined error" );
}

char *
VL_DeviceStatusString(
	VL_DevErr_t		devStatus
) {
	switch( devStatus ) {
	case VL_DevErr_None:				return( "No Result" );
	case VL_DevErr_VcselContinuityTest:	return( "VCSEL Continuity Fail" );
	case VL_DevErr_VcselWatchdogTest:	return( "VCSEL Watchdog Fail" );
	case VL_DevErr_NoVhvValueFound:		return( "No VHV Value found" );
	case VL_DevErr_MsrcNoTarget:		return( "MSRC No Target" );
	case VL_DevErr_SnrTest:				return( "S/N Ratio Fail" );
	case VL_DevErr_RangePhaseTest:		return( "Range Phase Fail" );
	case VL_DevErr_SigmaThresholdTest:	return( "Sigma Threshold Fail" );
	case VL_DevErr_Tcc:					return( "TCC Fail" );
	case VL_DevErr_PhaseConsistency:	return( "Phase Consistency Fail" );
	case VL_DevErr_MinClip:				return( "Min Clip Fail" );
	case VL_DevErr_RangeOk:				return( "Range Good" );
	case VL_DevErr_AlgoUnderflow:		return( "Range Algo Underflow" );
	case VL_DevErr_AlgoOverflow:		return( "Range Algo Overlow" );
	case VL_DevErr_Rit:					return( "Range Ignore Threshold Fail" );
	}
	return( "?bad device error code" );
}

char *
vl_rangingErrorString(
	uint8_t		rangeStatus
) {
	switch( rangeStatus ) {
	case 0: return( "Range Valid" );
	case 1: return( "Sigma Fail" );
	case 2: return( "Signal Fail" );
	case 3: return( "Min Range Fail" );
	case 4: return( "Phase Fail" );
	case 5: return( "Hardware Fail" );
	}
	return( "No Update" );
}

char *
vl_sequenceStepString(
	VL_SeqMask_t	seqStepBit
) {
	if( seqStepBit & SeqStepTcc )			return( "(Tcc)" );
	if( seqStepBit & SeqStepDss )			return( "(Dss)" );
	if( seqStepBit & SeqStepMsrc )			return( "(Msrc)" );
	if( seqStepBit & SeqStepPreRange )		return( "(PreRange)" );
	if( seqStepBit & SeqStepFinalRange )	return( "(FinalRange)" );
	return( "(?bad step)" );
}

char *
vl_limitCheckString(
	uint8_t		limitId
) {
	switch( limitId ) {
	case VlLim_SigmaFinalRange:		return( "  SigmaFinalRange   " );
	case VlLim_SignalRefClip:		return( "   SignalRefClip    " );
	case VlLim_Rit:					return( "RangeIgnoreThreshold" );
	case VlLim_SigRateMsrc:			return( "   SignalRateMsrc   " );
	case VlLim_SigRatePreRange:		return( " SignalRatePreRange " );
	case VlLim_SigRateFinalRange:	return( "SignalRateFinalRange" );
	}
	return( "?bad limit ID" );
}
#else

char *vl_revisionString( uint8_t revision ) { return( "" ); }
char *vl_apiErrorString( VL_Err_t apiError ) { return( "" ); }
char *vl_deviceErrorString( VL_DevErr_t devErrorCode ) { return( "" ); }
char *vl_rangingErrorString( uint8_t rangeStatus ) { return( "" ); }
char *vl_sequenceStepString( VL_SeqMask_t seqStepBit ) { return( "" ); }
char *vl_limitCheckString( uint8_t limitId ) { return( "" ); }

#endif

