/*------------------------------------------------------------------------
| vl53_core.h  --  libvl53 non-api function declarations
|
| New material Copyright (c) 2020 Gord Seiferling.
| Preexisting material Copyright © 2016, STMicroelectronics International N.V.
| Refer to license-lib.txt.
*/

#ifndef _VL53_API_CORE_H_
#define _VL53_API_CORE_H_

/*---  vl53_core.c  ---*/
void vl_get_nvram_info( VL_Dev_t *, VL_Spads_t * );
void vl_get_revision_info( VL_Dev_t *, VL_RevInfo_t * );

void vl_get_api_range_status( VL_Dev_t *, uint8_t, VL_RangeData_t * );
void vl_set_vcsel_pulse_periods( VL_Dev_t * );
void vl_get_vcsel_pulse_periods( VL_Dev_t * );
void vl_get_sequence_step_timeouts( VL_Dev_t * );
void vl_set_sequence_step_timeouts( VL_Dev_t * );

void vl_load_settings( VL_Dev_t *, uint8_t * );
void vl_run_reg_rw( VL_Dev_t *, uint8_t *, uint8_t );
void vl_wait_for_ranging( VL_Dev_t * );
uint32_t vl_calc_timeout_mclks( uint32_t, VL_VcselPer_t );
uint32_t vl_calc_timeout_us( uint32_t, VL_VcselPer_t );
uint32_t vl_calc_macro_period_ps( VL_VcselPer_t );
void vl_rev_bytes( uint8_t *, uint8_t );
uint32_t vl_div( uint32_t, uint32_t );

/*---  vl53_cal.c  ---*/
void vl_calibrate_offset( VL_Dev_t *, SFP2408_t );
void vl_calibrate_xtalk( VL_Dev_t *, uint16_t );

void vl_manage_ref_spads( VL_Dev_t *, VL_Spads_t * );
void vl_get_ref_spads( VL_Dev_t *, VL_Spads_t * );

void vl_calibrate_ref( VL_Dev_t * );
void vl_calibrate_phase( VL_Dev_t * );
void vl_ref_calibration_io( VL_Dev_t *, uint8_t, uint8_t *, uint8_t * );

/*---  vl53_sigma.c  ---*/
void vl_sigma_estimate( VL_Dev_t *, VL_RangeData_t * );

/*---  vl53_strings.c  ---*/
char *vl_revisionString( uint8_t );
char *vl_apiErrorString( VL_Err_t );
char *vl_deviceErrorString( VL_DevErr_t );
char *vl_rangingErrorString( uint8_t );
char *vl_sequenceStepString( uint8_t );
char *vl_limitCheckString( uint8_t );

/*---  I2C access, the application must supply these functions  ---*/
#define VL_PollDelayMs		5

void vl_wrBlock( VL_Dev_t *, uint8_t, uint8_t *, uint8_t );
void vl_wrDWord( VL_Dev_t *, uint8_t, uint32_t );
void vl_wrWord( VL_Dev_t *, uint8_t, uint16_t );
void vl_wrByte( VL_Dev_t *, uint8_t, uint8_t );

void vl_rdBlock( VL_Dev_t *, uint8_t, uint8_t *, uint8_t );
void vl_rdDWord( VL_Dev_t *, uint8_t, uint32_t * );
void vl_rdWord( VL_Dev_t *, uint8_t, uint16_t * );
void vl_rdByte( VL_Dev_t *, uint8_t, uint8_t * );

void vl_updateByte( VL_Dev_t *, uint8_t, uint8_t, uint8_t );

void vl_pollDelay( VL_Dev_t * );

#endif

