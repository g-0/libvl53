/*------------------------------------------------------------------------
| vl53_def.h  --  libvl53 api definitions
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#ifndef _VL53_DEF_H_
#define _VL53_DEF_H_

#define VL_ProdIdLen		18		/* 18-character device ID */

/* fractional value representation using fixed-point integers
| Many formats are used, below is 16.16, usually adds up to 32 bits.
*/
typedef uint32_t	FP1616_t;
typedef int32_t		SFP2408_t;			/* signed */

/* API Error codes */

typedef int8_t VL_Err_t;

#define VLerr_None						0
#define VLerr_NotAvailable				1	/* not implemented/supported */
#define VLerr_InvalidParams				2
#define VLerr_Timeout					3
#define VLerr_NoValidRange				4	/* no valid measures */
#define VLerr_RangeError				5
#define VLerr_UnknownMode				6	/* bad ranging/power mode */
#define VLerr_UnknownGpioPin			7	/* bad pin # */
#define VLerr_UnknownGpioFunc			8
#define VLerr_InterruptNotCleared		9
#define	VLerr_BadSpadNumber				10	/* s/w error */
#define	VLerr_InsuffRefSpads			11
#define VLerr_BadSpadReadback			12	/* SPAD map readback */
#define VLerr_I2cBusy					13	/* controller busy */
#define VLerr_I2cFailedXact				14
#define VLerr_I2cCollision				15
#define VLerr_I2cDeviceError			16

/* GPIO config */

typedef uint8_t VL_GpioMode_t;		/* pin assignment, below */
typedef uint8_t VL_GpioInt_t;		/* interrupt sources, VlReg_GpioInt* */

#define VlGpioFunc_Drive			20
#define VlGpioFunc_Osc				21
#define VlGpioFunc_Int				22

/* Device Power Modes */

typedef uint8_t VL_PowerModes_t;

#define VlPwr_Standby1		0
#define VlPwr_Standby2		1
#define VlPwr_Idle1			2
#define VlPwr_Idle2			3

typedef uint8_t VL_VcselPer_t;

/* The ranging sequence steps
| The sequential steps carried out by the scheduler during a measurement.
| Bits for each sequence step are used to form a mask of enabled steps.
| Additional undocumented steps exist.
*/
typedef uint8_t VL_SeqMask_t;

#define SeqStepTcc			0x01	/* Target Centre Check */
#define SeqStepDss			0x02	/* Dynamic SPAD Selection */
#define SeqStepMsrc			0x04	/* Minimum Signal Rate Check */
#define SeqStepPreRange		0x08	/* Pre-Range */
#define SeqStepFinalRange	0x10	/* Final-Range */

#define SeqStepAll			0x1f

/* Limit Check selectors */

#define VlLim_Rit					0x01
#define VlLim_SigmaFinalRange		0x02
#define VlLim_SignalRefClip			0x04
#define VlLim_SigRateMsrc			0x08
#define VlLim_SigRatePreRange		0x10
#define VlLim_SigRateFinalRange		0x20

#define VlLim_All					0x3f

/*---  Device version info.  Read from the device only when requested. */
typedef struct {
	uint8_t		moduleId;			/* Module ID */
	uint8_t		revision;			/* Revision, if moduleId != 0 */
	char		productId[ VL_ProdIdLen + 1 ];		/* product ID */
	uint32_t	uidUpper;			/* unique part ID upper */
	uint32_t	uidLower;			/* unique part ID lower */
	/* not from NVM */
	uint8_t		modelId;			/* Type, VL53L0X = 1, VL53L1 = 2 */
	uint8_t		revisionId;
} VL_RevInfo_t;

/*---  Range measurement results.  Used for API transfer, not stored  ---*/
typedef struct {
	uint8_t		rangeStat;		/* range status for measurement, 0=valid */
	uint16_t	rangeMm;		/* ranging distance mm */
	uint8_t		rangeFractMm;	/* range fractional mm, FP0008 */

	uint16_t	effSpadRtnCnt;	/* SPAD count for the return signal. FP0808 */
	FP1616_t sigRateRtnMcps;	/* return signal rate (target reflectance) */
	FP1616_t sigRateAmbRtnMcps;		/* return ambient rate (ambient light) */
	FP1616_t sigRatePeakRefMcps;	/* signal ref */

	FP1616_t	sigmaEstimate;	/* from ambient & VCSEL rates and
								| # signal events */
	uint16_t	RangeMaxMm;		/* maximum reliable detection distance
								| with current setup/environment (set in
								| estimated sigma calc) */
} VL_RangeData_t;

/*---  SPAD map.  Used for API transfer, not stored  ---*/
typedef struct {
	uint8_t refSpadMap[ VL_SpadMapSiz ];	/* Reference SPAD map */
	uint8_t nSpads;				/* # ref spads enabled */
	uint8_t tSpads;				/* type, 1=aperture */
} VL_Spads_t;

/*---  timing parameters  ---*/
typedef struct {
	VL_VcselPer_t	preRangePclks;		/* Vcsel pulse period (pll clocks) */
	VL_VcselPer_t	finalRangePclks;	/* Vcsel pulse period (pll clocks) */
	VL_SeqMask_t	seqEnables;			/* enabled sequence steps */
	uint32_t		msrcTimeoutUs;		/* time of Msrc, Tcc and Dss */
	uint32_t		preRangeTimeoutUs;	/* time of prerange */
	uint32_t		finalRangeTimeoutUs;	/* time of final range */
	uint32_t		totalTimeUs;		/* aggregate measurement time */
} VL_TimingParams_t;

/*---  calibration parameters  ---*/
/* Can be get/set collectively, while calibration functions generate
| individual elements
*/
typedef struct {
	uint16_t	linearRangeScale;	/* s/w scaling factor, x1000 */
	FP1616_t	xTalkCompRateMcps;	/* xTalk comp rate per SPAD */
	int32_t		offsetUm;			/* range offset */

	uint8_t		vhvSettings;		/* reference components */
	uint8_t		phaseCal;

	uint16_t	dmaxCalRangeMm;		/* Dmax Cal, Range */
	FP1616_t	dmaxCalSigRateMcps;	/* Dmax Cal, Signal Return Rate */
} VL_CalParams_t;

/*---  measurement parameters  ---*/
/* measurement options and limit checks */
typedef struct {
	uint32_t	repeatPeriodMs;		/* inter-measurement delay */
	uint8_t		rangeFractEnable;	/* fractional ranging result when !0 */

	uint8_t		limitEnables;
	FP1616_t	limVal_rit;					/* s/w, per spad */
	FP1616_t	limVal_sigmaFinalRange;		/* s/w */
	FP1616_t	limVal_signalRefClip;		/* s/w */
	FP1616_t	limVal_sigRatePreRange;		/* h/w, for MSRC also */
	FP1616_t	limVal_sigRateFinalRange;	/* h/w, to device */
} VL_MeasureParams_t;

/*---  device-specific parameters  ---*/
typedef struct {
	uint8_t		nvmRefSpadMap[ VL_SpadMapSiz ];	/* NVM Reference Spad map */
	FP1616_t	nvmSigRateAt40cm;	/* Peak Signal rate at 40 cm */
	int32_t		nvmRangeAt40cmUm;	/* range at 40cm nominal, unused */
	FP1616_t	oscFreqMHz;			/* Frequency, from reg 0x184 (unused) */
} VL_DeviceParams_t;

/*---  top-level driver data container  ---*/
typedef struct {
	VL_Err_t		err;			/* running handler status */
	uint8_t			i2cDevAddr;		/* i2c device slave address */
	uint8_t			runEnables;		/* 0x91 image to enable cycling */
	VL_GpioInt_t	pin0Gpio;		/* GPIO: pin0 int function */

	VL_CalParams_t			CalibrateParams;	/* factors, points, etc */
	VL_TimingParams_t		TimingParams;		/* all timing parameters */
	VL_MeasureParams_t		MeasureParams;		/* measurement options */
	VL_DeviceParams_t		DevParams;			/* mish-mash */
} VL_Dev_t;

/* register blast (settings) metacharacters */
#define MetaStop			0xc0	/* reg blast terminator */
#define MetaDelay			0xc1	/* wait one polling delay */
#define MetaUpdate			0xc2	/* 3 bytes: addr, AND, OR values */

#define VL_Cal( field )			Dev->CalibrateParams.field
#define VL_Meas( field )		Dev->MeasureParams.field
#define VL_Timing( field )		Dev->TimingParams.field
#define VL_Device( field )		Dev->DevParams.field

/*---  fixed-point conversions  ---*/

#define FP1616TOFP0907(a)		( (uint16_t)a >> 9 )
#define FP0907TOFP1616(a)		( (FP1616_t)a << 9 )

#define FP1616TOFP0313(a)		( (uint16_t)a >> 3 )
#define FP0313TOFP1616(a)		( (FP1616_t)a << 3 )

/* histogram stuff left here as clues.  Definitions exist to
| satisfy prototypes, but nothing is allocated */

/* Histogram modes */

typedef uint8_t		VL_HistogramModes_t;

#define VL_HISTMODE_DISABLED		0
#define VL_HISTMODE_REFERENCE_ONLY	1	/* Reference array only */
#define VL_HISTMODE_RETURN_ONLY		2	/* Return array only */
#define VL_HISTMODE_BOTH			3	/* both Arrays */

#define VL_HISTOGRAM_BUFFER_SIZE 24

/* Histogram measurement data */

typedef struct {
	uint32_t histogramData[VL_HISTOGRAM_BUFFER_SIZE];		/* data */
	uint8_t histogramType;		/* type: Return and/or Reference */
	uint8_t firstBin;			/* First Bin value */
	uint8_t bufferSize;			/* Buffer Size - Set by the user.*/
	uint8_t numberOfBins;		/* # of bins filled by the measurement */
	VL_DevErr_t	errorStatus;	/* status of the current measurement */
} VL_HistMeasData_t;

#endif

