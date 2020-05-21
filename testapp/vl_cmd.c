/*------------------------------------------------------------------------
| vl_cmd.c  --  VL53L0x test utility command handler
|
| Copyright (c) 2020 Gord Seiferling.
| Refer to license-app.txt.
|
| Test application to operate the VL53L0X distance sensor.
| Allows test of all device functionality.
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <hw/inout.h>
#include <sys/dispatch.h>

#include "../vl53/vl53_api.h"
#include "vl_app.h"

uint16_t	debugGrant;			/* flags, not a count */

/*---  local function protos  ---*/

void dumpSpace( VL_Dev_t *, uint32_t, uint32_t, uint8_t );
static void i_registerInteractive( VL_Dev_t * );

static void i_show_api_status( VL_Err_t );
static void i_init( VL_Dev_t * );
static void i_rangeOne( VL_Dev_t * );
static void i_rangeMany( VL_Dev_t *, uint8_t, char * );
static void i_showTiming( VL_Dev_t * );
static void i_showCal( VL_Dev_t * );
static void i_showMeasure( VL_Dev_t * );
static void i_showRevision( VL_Dev_t * );
static void i_calOffset( VL_Dev_t *, char * );
static void i_calXtalk( VL_Dev_t *, char * );
static void i_calReference( VL_Dev_t * );
static void i_calSpads( VL_Dev_t * );
static void i_showSpadMap( char *, uint8_t [] );
static void i_setMode( VL_Dev_t *, int );
static void i_showHelp();

const char AppOpts[] = ":d:h";

char * const cmdToks[] = {
	"cal", "offset", "meas", "set", "range", "reg", "dump", "q",	/* 0-7 */
	"rev", "init", "timing", "ref", "power", "reset", "cont", "timed",	/* 8-15 */
	"xtalk", "spads", "help", "gpio", "osc", "int", "drive", "debug",	/* 16-23 */
	"mode", "acc", "far", "fast",
	NULL
};

#define Cmd_cal		0
#define Cmd_offset	1
#define Cmd_meas	2
#define Cmd_set		3
#define Cmd_range	4
#define Cmd_reg		5
#define Cmd_dump	6
#define Cmd_quit	7

#define Cmd_rev		8
#define Cmd_init	9
#define Cmd_timing	10
#define Cmd_ref		11
#define Cmd_power	12
#define Cmd_reset	13
#define Cmd_cont	14
#define Cmd_timed	15

#define Cmd_xtalk	16
#define Cmd_spads	17
#define Cmd_help	18
#define Cmd_gpio	19
#define Cmd_osc		20
#define Cmd_int		21
#define Cmd_drive	22
#define Cmd_debug	23

#define Cmd_mode	24
#define Cmd_acc		25
#define Cmd_far		26
#define Cmd_fast	27

static VL_Dev_t		dd;			/* our device */

int
main(
	int		argc,
	char	*argv[]
) {
	int		opt;		/* cmd line option character */
	int			iveHadEnuf;
	char		cmdBuf[ 80 ];

	/*---  enable i/o access  ---*/
	ThreadCtl( _NTO_TCTL_IO, 0 );

	/*---  parse the cmd-line options  ---*/
	while( ( opt = getopt( argc, argv, AppOpts ) ) != -1 ) {
		switch( opt ) {
		case 'd':
			debugGrant = atoh( optarg );
			break;
		case 'h':
			i_showHelp();
			break;
		}
	}

	/*---  begin interactive device operation  ---*/
	puts( "begin VL53L0x interactive" );
	for( iveHadEnuf = 0; ! iveHadEnuf; ) {
		char	*cmdRun;	/* command runner */
		char	*vp;		/* value ptr */

		printf( "- " );
		if( fgets( cmdBuf, sizeof( cmdBuf ), stdin ) == NULL ) break;
		cmdRun = strtok( cmdBuf, " \r\n\t" );
		if( cmdRun ) {
			switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
			case Cmd_help: i_showHelp(); break;
			case Cmd_quit: iveHadEnuf = !0; break;
			case Cmd_power: {
				VL_PowerModes_t		mode;

				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					vl_getPowerMode( & dd, & mode );
					printf( "Power status is %s\n", mode == VlPwr_Idle1 ?
						"IDLE" : "STANDBY" );
					break;
				}
				vl_setPowerMode( & dd, atoi( cmdRun ) ?
					VlPwr_Idle1 : VlPwr_Standby1 );
				break;
			}
			case Cmd_reset: vl_deviceReset( & dd, 0 ); break;
			case Cmd_rev: i_showRevision( & dd ); break;
			case Cmd_init: i_init( & dd ); break;
			case Cmd_gpio:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					uint8_t		intStatus;

					printf( "GPIO Int configured: %d\n", dd.pin0Gpio );
					vl_getInterruptStatus( & dd, & intStatus );
					printf( "Int Status=%02x\n", intStatus );
					break;
				}
				switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
				case Cmd_int:
					cmdRun = strtok( NULL, " \r\n\t" );
					if( cmdRun ) vl_setGpioConfig( & dd, 0,
						VlGpioFunc_Int, atoi( cmdRun ), 0 );
					break;
				case Cmd_drive:
					cmdRun = strtok( NULL, " \r\n\t" );
					if( cmdRun ) {
						vl_setGpioConfig( & dd, 0,
							VlGpioFunc_Drive, 0, atoi( cmdRun ) );
						printf( "gpio drive set to %d\n", atoi( cmdRun ) );
					}
					break;
				case Cmd_osc:
					vl_setGpioConfig( & dd, 0, VlGpioFunc_Osc, 0, 0 );
					break;
				default:
					puts( "?don't know that GPIO fcn" );
					break;
				}
				break;
			case Cmd_meas:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					i_showMeasure( & dd );
					break;
				}
				switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
				case Cmd_set: vd_measSet( & dd ); break;
				default:
					puts( "?don't know what that is" );
					break;
				}
				break;
			case Cmd_mode:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					puts( "?specify a ranging mode" );
					break;
				}
				i_setMode( &dd, getsubopt( & cmdRun, cmdToks, & vp ) );
				break;
			case Cmd_timing:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					i_showTiming( & dd );
					break;
				}
				switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
				case Cmd_set: vd_timingSet( & dd ); break;
				default:
					puts( "?don't know what that is" );
					break;
				}
				break;
			case Cmd_cal:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					i_showCal( & dd );
					break;
				}
				switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
				case Cmd_spads: i_calSpads( & dd ); break;
				case Cmd_offset: i_calOffset( & dd, vp ); break;
				case Cmd_xtalk: i_calXtalk( & dd, vp ); break;
				case Cmd_ref: i_calReference( & dd ); break;
				case Cmd_set: vd_calSet( & dd ); break;
				default:
					puts( "?don't know how to calibrate that" );
					break;
				}
				break;
			case Cmd_range:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					i_rangeOne( & dd );
					break;
				}
				switch( getsubopt( & cmdRun, cmdToks, & vp ) ) {
				case Cmd_cont:
					i_rangeMany( & dd, VlReg_RangeContinuous, vp );
					break;
				case Cmd_timed:
					i_rangeMany( & dd, VlReg_RangeTimed, vp );
					break;
				default:
					puts( "?don't know how to range that" );
					break;
				}
				break;
			case Cmd_reg:
				i_registerInteractive( & dd );
				break;
			case Cmd_debug:
				cmdRun = strtok( NULL, " \r\n\t" );
				if( ! cmdRun ) {
					printf( "debug selector is 0x%04x\n", debugGrant );
					break;
				}
				debugGrant = atoh( cmdRun );
				break;
			default:
				puts( "?don't know how to do that" );
			}
			if( dd.err ) i_show_api_status( dd.err );
		}
	}
	return( 0 );
}

static void
i_init(
	VL_Dev_t	*dd
) {
	memset( (uint8_t *) dd, 0, sizeof( VL_Dev_t ) );
	dd->i2cDevAddr = 0x52;

	puts( "Initializing..." );
	/*---  device initialization  ---*/
	vl_deviceInit( dd );
	if( dd->err ) {
		puts( "vl_deviceInit failure" );
		return;
	}
}

static void
i_setMode(
	VL_Dev_t	*dd,
	int		cmdMode		/* command token mode */
) {
	VL_TimingParams_t	tiPar;
	VL_MeasureParams_t	mePar;

	/*---  get the current settings and preset the defaults  ---*/
	vl_getTiming( dd, & tiPar );
	if( dd->err ) return;
	vl_getMeasure( dd, & mePar );
	if( dd->err ) return;

	mePar.limVal_sigRateFinalRange = 0.25 * 65536;
	mePar.limVal_sigmaFinalRange = 18.0 * 65536;
	mePar.limitEnables |= VlLim_SigmaFinalRange | VlLim_SigRateFinalRange;

	tiPar.preRangePclks = 14;
	tiPar.finalRangePclks = 10;
	tiPar.finalRangeTimeoutUs = 25000;

	switch( cmdMode ) {
	case Cmd_acc:		/* accurate */
		tiPar.finalRangeTimeoutUs = 200000;
		break;
	case Cmd_far:		/* far */
		mePar.limVal_sigRateFinalRange = 0.10 * 65536;
		mePar.limVal_sigmaFinalRange = 60.0 * 65536;
		tiPar.preRangePclks = 18;
		tiPar.finalRangePclks = 14;
		tiPar.finalRangeTimeoutUs = 33000;
		/* disable DSS? */
		break;
	case Cmd_fast:		/* fast */
		mePar.limVal_sigmaFinalRange = 32.0 * 65536;
		tiPar.finalRangeTimeoutUs = 20000;
		break;
	default:
		puts( "setting default mode" );
	}

	vl_setMeasure( dd, & mePar );
	if( dd->err ) return;
	vl_setTiming( dd, & tiPar );
}

static void
i_rangeOne(
	VL_Dev_t	*dd
) {
	VL_RangeData_t	rangeData;
	uint8_t		Status;

	vl_singleMeasurementWait( dd, & rangeData );
	if( dd->err ) {
		puts( "vl_singleMeasurementWait failure" );
		return;
	}
	Status = rangeData.rangeStat;
	printf( "Ranging Status: %d : %s\n", Status,
		vl_rangingErrorString( Status ) );

	printf( "Range: %d mm\n", rangeData.rangeMm );
}

/* Continuous/Timed ranging.  ST demo code disabled
| SigRateFinal and SigmaFinal limits.
*/
static void
i_rangeMany(
	VL_Dev_t	*dd,
	uint8_t		cycleType,		/* for command register */
	char		*countPtr		/* optional pointer to # cycles */
) {
	VL_RangeData_t	rangeData;
	uint8_t		ready;
	int			cycCount;
	int		j, k;

	vl_startMeasurement( dd, cycleType );
	if( dd->err ) {
		puts( "vl_startMeasurement failure" );
		return;
	}
	cycCount = countPtr ? atoi( countPtr ) : 5;
	for( j = 0; j < cycCount; j++ ) {
		/*---  wait for a ranging data set  ---*/
		for( k = 0; ; k++ ) {
			if( k > 1000 ) dd->err = VLerr_Timeout;
			vl_getRangingDataReady( dd, & ready );
			if( dd->err ) {
				puts( "vl_getRangingDataReady failure" );
				return;
			}
			if( ready ) break;
		}
		vl_getRangingResults( dd, & rangeData );
		if( dd->err ) {
			puts( "VL_GetRangingData failure" );
			return;
		}
		printf( "(%d) Ranging # %d: %d mm\n", k, j, rangeData.rangeMm );
	}
	vl_stopRun( dd );
	if( dd->err ) {
		puts( "vl_stopRun failure" );
		return;
	}
	/*---  always exits loop at first or 2nd test  ---*/
	for( j = 0; j < 500; j++ ) {
		uint8_t		stt;

		vl_getRunStatus( dd, & stt );
		if( ! stt ) break;
	}
	printf( "exiting stop wait at %d iterations\n", j );
	vl_clearInterrupts( dd, 0 );
	if( dd->err ) {
		puts( "vl_clearInterrupts failure" );
		return;
	}
}

static void
i_showTiming(
	VL_Dev_t	*dd
) {
	VL_TimingParams_t	tpar;
	VL_SeqMask_t		seqEn;			/* enabled sequence steps */
	VL_SeqMask_t		j;

	puts( "-------- Timing --------" );
	vl_getTiming( dd, & tpar );
	if( dd->err ) return;

	printf( "Vcsel pclks (pre, final): %d, %d\n",
		tpar.preRangePclks, tpar.finalRangePclks );

	printf( "Sequence Step timeouts (MSRC, pre, final): %d, %d, %d\n",
		tpar.msrcTimeoutUs, tpar.preRangeTimeoutUs, tpar.finalRangeTimeoutUs );

	/*---  show the enabled sequence steps  ---*/
	seqEn = tpar.seqEnables;
	printf( "Enabled Steps: " );
	for( j = Bit_0; j; j <<= 1 ) {
		if( seqEn & j ) printf( vl_sequenceStepString( j ) );
	}
	putchar( '\n' );
	printf( "Total Measurement Time: %d us\n", tpar.totalTimeUs );
}

static void
i_showCal(
	VL_Dev_t	*dd
) {
	VL_CalParams_t	MyCal;

	puts( "------ Calibration -----" );
	vl_getCalibration( dd, & MyCal );
	if( dd->err ) return;

	printf( "Offset: %d um\n", MyCal.offsetUm );
	printf( "Linear Range Scaling: %d (x1000)\n", MyCal.linearRangeScale );
	printf( "VHV settings: %d, Phase cal: %d\n",
		MyCal.vhvSettings, MyCal.phaseCal );
	printf( "Xtalk Comp Rate: %f Mcps/SPAD\n",
		MyCal.xTalkCompRateMcps / 65536.0 );
	printf( "DMax Rate: %f Mcps @ %u mm\n",
		MyCal.dmaxCalSigRateMcps / 65536.0,
		MyCal.dmaxCalRangeMm );

	printf( "NVM: SignalRateAt40cm= %f FP0907 mm, Dist(40cm)= %d um\n",
		(float)dd->DevParams.nvmSigRateAt40cm / 65536.0,
		dd->DevParams.nvmRangeAt40cmUm
	);
}

static void
i_showMeasure(
	VL_Dev_t	*dd
) {
	VL_MeasureParams_t	myPar;
	uint8_t		s;			/* limit selector */

	puts( "------ Measurement -----" );
	vl_getMeasure( dd, & myPar );
	if( dd->err ) return;

	printf( "Auto Repeat Interval: %u ms\n", myPar.repeatPeriodMs );
	printf( "Range Fraction: -%s-\n",
		myPar.rangeFractEnable ? "Enabled" : "Disabled" );

	/*---  show the limit checks  ---*/
	puts( "Limit Checks:" );
	for( s = Bit_0; s; s <<= 1 ) {
		if( s & ~VlLim_All ) continue;
		printf( "%s: -%s- Limit: %f\n",
			vl_limitCheckString( s ),
			s & myPar.limitEnables ? " Enabled" : "Disabled",
			*vc_getLimitPtr( & myPar, s ) / 65536.0
		);
	}
}

FP1616_t *
vc_getLimitPtr(
	VL_MeasureParams_t	*measPar,		/* measure parameters */
	uint8_t		limBit					/* limit flag */
) {
	switch( limBit ) {
	case VlLim_Rit: return( & measPar->limVal_rit );
	case VlLim_SigmaFinalRange: return( & measPar->limVal_sigmaFinalRange );
	case VlLim_SignalRefClip: return( & measPar->limVal_signalRefClip );
	case VlLim_SigRateMsrc:
	case VlLim_SigRatePreRange: return( & measPar->limVal_sigRatePreRange );
	case VlLim_SigRateFinalRange: return( & measPar->limVal_sigRateFinalRange );
	default: 
		puts( "?bad limit selector" );
		exit( 1 );
	}
}

static void
i_registerInteractive(
	VL_Dev_t	*dd
) {
	uint8_t		argCnt;			/* # of addresses entered */
	uint32_t	addrLow, addrHi;
	uint32_t	data;
	uint8_t		ioSpace = 0;	/* i/o or VL device select */
	char		cmdBuf[ 80 ];

	puts( "register mode" );
	puts( "'e aa dd' (enter), 'd aa[,aa]' (dump), 'v', 'i', 'x' (quit)" );
	for( ;; ) {
		printf( ": " );
		if( fgets( cmdBuf, sizeof( cmdBuf ), stdin ) == NULL ) break;
		switch( cmdBuf[ 0 ] ) {
		case 'd':
			argCnt = sscanf( cmdBuf + 1, "%x,%x", & addrLow, & addrHi );
			if( argCnt == 1 ) addrHi = addrLow;
			else if( argCnt != 2 ) {
				puts( "? need low[,high] hex addresses" );
				break;
			}
			if( addrHi < addrLow ) {
				puts( "? low,high please" );
				break;
			}
			/*---  limit the dump to 256 bytes  ---*/
			if( addrHi > addrLow + 255 ) addrHi = addrLow + 255;
			dumpSpace( dd, addrLow, addrHi, ioSpace );
			break;
		case 'e':
/*@@@ make this handle multiple data bytes, autoincrement addr */
			if( sscanf( cmdBuf + 1, "%x %x", & addrLow, & data ) != 2 ) {
				puts( "? need hex address and data" );
				break;
			}
			if( ioSpace ) {
				out8( addrLow, data );
			} else {
				vl_wrByte( dd, (uint8_t) addrLow, data );
			}
			printf( "[%02x] <- %02x\n", addrLow, data );
			break;
		case 'v':
			ioSpace = 0;
			puts( "VL register space" );
			break;
		case 'i':
			ioSpace = !0;
			puts( "i/o space" );
			break;
		case 'x':
			puts( "register mode exit" );
			return;
		case '\r':
		case '\n':
			break;
		default:
			puts( "?" );
		}
	}
}

void
dumpSpace(
	VL_Dev_t	*dd,
	uint32_t	start,
	uint32_t	finish,
	uint8_t		ioMode			/* !0 for VL53 registers */
) {
	uint32_t	adRun;			/* address runner */
	uint8_t		aByte;

	for( adRun = start & 0xfff0; adRun <= finish; adRun++ ) {
		if( ! ( adRun & 0xf ) ) printf( "\n%04x/", adRun );
		else if( ! ( adRun & 0x3 ) ) putchar( ' ' );
		if( adRun >= start ) {
			if( ioMode ) {
				aByte = in8( (uint16_t) adRun );
			} else {
				vl_rdByte( dd, (uint8_t) adRun, & aByte );
			}
			printf( " %02x", aByte );
		} else {
			printf( " .." );
		}
	}
	putchar( '\n' );
}

static void
i_showRevision(
	VL_Dev_t	*dd
) {
	VL_RevInfo_t	MyRev;

	vl_get_revision_info( dd, & MyRev );
	if( dd->err ) return;

	printf("-NVM- Device ID : %s\n", MyRev.productId );
	printf( "Module ID: %02x\n", MyRev.moduleId );
	printf( "Revision: %02x (%s)\n", MyRev.revision,
		vl_revisionString( MyRev.revision ) );
	printf( "Part UID: %08x, %08x\n", MyRev.uidUpper, MyRev.uidLower );
	printf( "-Ident- Model ID & Revision ID: %02x, %02x\n",
		MyRev.modelId, MyRev.revisionId );
	printf( "Revision: %d.%d (expect 1.1)\n",
		1, MyRev.revisionId >> 4 );
}

static void
i_show_api_status(
	VL_Err_t	Status
) {
	printf( "API Error: %d : %s\n", Status, vl_apiErrorString( Status ) );
}

static void
i_calOffset(
	VL_Dev_t	*dd,
	char		*rangeMm		/* optional pointer to actual range */
) {
	int			distMm;

	distMm = rangeMm ? atoi( rangeMm ) : 100;
	printf( "Calibrate Offset @ %dmm...\n", distMm );
	vl_calibrateOffset( dd, distMm << 8 );
}

static void
i_calXtalk(
	VL_Dev_t	*dd,
	char		*rangeMm		/* optional pointer to actual range */
) {
	int			distMm;

	distMm = rangeMm ? atoi( rangeMm ) : 400;
	printf( "Calibrate Xtalk @ %dmm...\n", distMm );
	vl_calibrateXtalk( dd, distMm );
}

static void
i_calReference(
	VL_Dev_t	*dd
) {
	puts( "Calibrate Ref..." );
	vl_calibrateRef( dd );
}

static void
i_calSpads(
	VL_Dev_t	*Dev
) {
	VL_Spads_t	sp;

	memcpy( sp.refSpadMap, VL_Device( nvmRefSpadMap ), VL_SpadMapSiz );
	puts( "vl_manageRefSpads..." );
	sp.nSpads = 0;
	vl_manageRefSpads( Dev, & sp );
	if( Dev->err ) {
		puts( "ManageRefSpad error" );
		return;
	}

	/*---  show the resulting map, and metadata  ---*/
	puts( "vl_getRefSpads..." );
	vl_getRefSpads( Dev, & sp );
	if( Dev->err ) {
		puts( "GetReferenceSpads error" );
		return;
	}
	i_showSpadMap( "NVM map", VL_Device( nvmRefSpadMap ) );
	i_showSpadMap( "New map", sp.refSpadMap );

	printf( "Ref SPAD count: %d, Aperture: %s\n",
		sp.nSpads, sp.tSpads ? "Yes" : "No" );
}

static void
i_showSpadMap(
	char	*title,
	uint8_t		devMap[]
) {
	int		i;

	printf( "%s:  ", title );
	for( i = 0; i < VL_SpadMapSiz; i++ ) printf( "%02x ", devMap[ i ] );
	putchar( '\n' );
}

static void
i_showHelp(
) {
	printf( "%s [opts]\n", "ve" );
	puts( "-h - this help" );
	puts( "-d {hex val} - set debug grant flags" );
	puts( "---  commands  ---" );
	puts( "init  --  initialize the driver and device (do this first)" );
	puts( "reset  --  restore device to power-on condition" );
	puts( "rev  --  show device info" );
	puts( "power  --  show power mode" );
	puts( "power [0|1]  --  set power mode Standby|Idle" );
	puts( "gpio  --  show GPIO int function" );
	puts( "gpio int [0-4]  --  assign GPIO as int function" );
	puts( "gpio drive [0|1]  --  assign GPIO as static drive" );
	puts( "gpio osc  --  assign GPIO as oscillator output" );
	puts( "meas  --  show measurement parameters" );
	puts( "meas set  --  measurement parameter setup dialog" );
	puts( "timing  --  show timing parameters" );
	puts( "timing set  --  timing parameter setup dialog" );
	puts( "cal  --  show calibration parameters" );
	puts( "cal set  --  calibration parameter setup dialog" );
	puts( "cal ref  --  perform reference calibration" );
	puts( "cal offset[=dist]  --  perform offset calibration" );
	puts( "cal xtalk[=dist]  --  perform xtalk calibration" );
	puts( "cal spads  --  regenerate ref SPAD map" );
	puts( "mode [def|acc|far|fast]  --  adjust settings for mode" );
	puts( "range  --  one ranging measurement, show results" );
	puts( "range cont[=num]  --  continuous ranging measurements" );
	puts( "range timed[=num]  --  timed ranging measurements" );
	puts( "reg  --  enter register (and i/o) read/write mode" );
	puts( "debug [grants(hex)]  --  show/set debug grant flags" );
	puts( "q  --  exit test utility" );
}

