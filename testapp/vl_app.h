/*------------------------------------------------------------------------
| vl_app.h  --  vl53l0x test application includes
|
| Copyright (c) 2020 Gord Seiferling.
| Refer to license-app.txt.
| gfs@magma.ca
|
*/

#define Bit_0		0x01
#define Bit_1		0x02
#define Bit_2		0x04
#define Bit_3		0x08
#define Bit_4		0x10
#define Bit_5		0x20
#define Bit_6		0x40
#define Bit_7		0x80

/* vl_cmd.c */
FP1616_t *vc_getLimitPtr( VL_MeasureParams_t *, uint8_t );

/* vl_dial.c */
void vd_measSet( VL_Dev_t * );
void vd_timingSet( VL_Dev_t * );
void vd_calSet( VL_Dev_t * );

