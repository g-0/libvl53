/*------------------------------------------------------------------------
| vl_dial.c  --  command-line dialogs for entry of device parameters.
|
| Copyright (c) 2020 Gordon Seiferling.
| Refer to license-app.txt.
|
| Test application to operate the VL53L0X distance sensor.
|
| Each function gets the current settings, allows selective entry
| of each parameter and then offers them to the device handler for update.
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "../vl53/vl53_api.h"
#include "vl_app.h"

/*---  local function protos  ---*/
static char *i_getEntry();

void
vd_measSet(
	VL_Dev_t	*dd
) {
	VL_MeasureParams_t	mpar;
	char		*usrEnt;			/* entered string */
	uint8_t		s;
	float		limVal;

	vl_getMeasure( dd, & mpar );
	if( dd->err ) return;

	puts( "Key new value, or <Enter> to keep unchanged" );
	printf( "Auto Repeat Interval, ms (%u): ", mpar.repeatPeriodMs );
	if( usrEnt = i_getEntry() ) mpar.repeatPeriodMs = atoi( usrEnt );

	printf( "Range Fraction: (%s): ",
		mpar.rangeFractEnable ? "Enabled" : "Disabled" );
	if( usrEnt = i_getEntry() )
		mpar.rangeFractEnable = *usrEnt == 'y' ? !0 : 0;

	/*---  show the limit checks  ---*/
	puts( "--- Limits ---" );
	for( s = Bit_0; s; s <<= 1 ) {
		if( s & ~VlLim_All ) continue;
		printf( "%s: (%s) Val: %f: ",
			vl_limitCheckString( s ),
			s & mpar.limitEnables ? " Enabled" : "Disabled",
			*vc_getLimitPtr( & mpar, s ) / 65536.0
		);
		if( usrEnt = i_getEntry() ) {
			if( *usrEnt == 'y' ) {
				mpar.limitEnables |= s;
				printf( "new limit: " );
				if( usrEnt = i_getEntry() ) {
					sscanf( usrEnt, "%f", & limVal );
					*vc_getLimitPtr( & mpar, s ) = limVal * 65536.0 + 0.5;
				}
			} else {
				mpar.limitEnables &= ~s;
			}
		}
	}
	vl_setMeasure( dd, & mpar );
}

void
vd_timingSet(
	VL_Dev_t	*dd
) {
	VL_TimingParams_t	tpar;
	char		*usrEnt;			/* entered string */
	VL_SeqMask_t		seqEn;		/* enabled sequence steps */
	VL_SeqMask_t		j;			/* bit runner */

	vl_getTiming( dd, & tpar );
	if( dd->err ) return;

	puts( "Key new value, or <Enter> to keep unchanged" );
	printf( "Pre-range Vcsel pclks (%d) {12|14|16|18}: ",
		tpar.preRangePclks );
	if( usrEnt = i_getEntry() ) tpar.preRangePclks = atoi( usrEnt );

	printf( "Final range Vcsel pclks (%d) {8|10|12|14}: ",
		tpar.finalRangePclks );
	if( usrEnt = i_getEntry() ) tpar.finalRangePclks = atoi( usrEnt );

	printf( "Tcc/Dss/Msrc Step Timeout, us (%d): ", tpar.msrcTimeoutUs );
	if( usrEnt = i_getEntry() ) tpar.msrcTimeoutUs = atoi( usrEnt );
	printf( "Pre-range Step Timeout, us (%d): ", tpar.preRangeTimeoutUs );
	if( usrEnt = i_getEntry() ) tpar.preRangeTimeoutUs = atoi( usrEnt );
	printf( "Final range Step Timeout, us (%d): ", tpar.finalRangeTimeoutUs );
	if( usrEnt = i_getEntry() ) tpar.finalRangeTimeoutUs = atoi( usrEnt );

	seqEn = tpar.seqEnables;
	puts( "Sequence Steps..." );
	for( j = Bit_0; j; j <<= 1 ) {
		if( j & ~SeqStepAll ) continue;
		printf( "%s (%s): ", vl_sequenceStepString( j ),
			seqEn & j ? "Enabled" : "Disabled" );
		if( usrEnt = i_getEntry() ) {
			if( *usrEnt == 'y' ) {
				seqEn |= j;
			} else {
				seqEn &= ~j;
			}
		}
	}
	tpar.seqEnables = seqEn;
	vl_setTiming( dd, & tpar );
}

void
vd_calSet(
	VL_Dev_t	*dd
) {
	VL_CalParams_t	cpar;
	char		*usrEnt;			/* entered string */
	float		usrVal;

	vl_getCalibration( dd, & cpar );
	if( dd->err ) return;

	printf( "Offset, um (%d): ", cpar.offsetUm );
	if( usrEnt = i_getEntry() ) cpar.offsetUm = atoi( usrEnt );

	printf( "Linear Range Scaling, x1000 (%d): ", cpar.linearRangeScale );
	if( usrEnt = i_getEntry() ) cpar.linearRangeScale = atoi( usrEnt );

	printf( "VHV setting (%d): ", cpar.vhvSettings );
	if( usrEnt = i_getEntry() ) cpar.vhvSettings = atoi( usrEnt );

	printf( "Phase cal (%d): ", cpar.phaseCal );
	if( usrEnt = i_getEntry() ) cpar.phaseCal = atoi( usrEnt );

	printf( "Xtalk Comp Rate, Mcps/SPAD (%f): ",
		cpar.xTalkCompRateMcps / 65536.0 );
	if( usrEnt = i_getEntry() ) {
		sscanf( usrEnt, "%f", & usrVal );
		cpar.xTalkCompRateMcps = usrVal * 65536.0 + 0.5;
	}

	printf( "DMax Rate, Mcps (%f): ", cpar.dmaxCalSigRateMcps / 65536.0 );
	if( usrEnt = i_getEntry() ) {
		sscanf( usrEnt, "%f", & usrVal );
		cpar.dmaxCalSigRateMcps = usrVal * 65536.0 + 0.5;
	}

	printf( "DMax Range, mm (%d): ", cpar.dmaxCalRangeMm );
	if( usrEnt = i_getEntry() ) cpar.dmaxCalRangeMm = atoi( usrEnt );

	vl_setCalibration( dd, & cpar );
}

/* returns a static string with an entered value, or NULL */

static char *
i_getEntry(
) {
	static char		buf[ 80 ];

	if( fgets( buf, sizeof( buf ), stdin ) ) {
		if( strtok( buf, " \r\n\t" ) ) return( buf );
	}
	return( NULL );
}

