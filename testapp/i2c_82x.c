/*------------------------------------------------------------------------
| i2c_82x.c  --  platform-specific i2c interface functions
|
| Copyright (c) 2020 Gordon Seiferling.
| Refer to license-app.txt.
|
| target: ARM EABI with NXP LPC82x CPU
|
| Provide 8/16/32-bit register access via i2c.
| Every write transaction MUST have the index as 1st byte.
|
| Common arguments:
| Dev       Device Handle
| index     The VL53L0x register index
*/

#include "../vl53/vl53_api.h"
#include "perthmon.h"

/*---  local function protos  ---*/

/* writes the supplied byte buffer to the device.
| For simplicity, all writes >= 4 bytes are sequenced as single-byte
| data transfers, with the index stepped (and explicitly sent) at each
| transaction.
*/

void
vl_wrBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	/* quick 'n' dirty interface to existing i2c functions */
	for( k = 0; k < count; k++ ) {
		vl_wrByte( Dev, index++, *pdata++ );
	}
}

/* write 32-bit word to register, msb first */

void
vl_wrDWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint32_t	data
) {
#if BYTE_ORDER == LITTLE_ENDIAN
	vl_rev_bytes( (uint8_t *) & data, 4 );
#endif
	vl_wrBlock( Dev, index, (uint8_t *) & data, 4 );
}

/* write 16-bit word to register, msb first */

void
vl_wrWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint16_t	data
) {
#if BYTE_ORDER == LITTLE_ENDIAN
	vl_rev_bytes( (uint8_t *) & data, 2 );
#endif
	vl_wrBlock( Dev, index, (uint8_t *) & data, 2 );
}

/* write single byte to register */

void
vl_wrByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		data
) {
	uint8_t		xBuf[ 2 ];
	extern		uint16_t	debugGrant;

	if( Dev->err ) return;
	xBuf[ 0 ] = index;
	xBuf[ 1 ] = data;
	if( i2c_trans( Dev->i2cDevAddr | I2c_Write, xBuf, 2 ) )
		Dev->err = VLerr_I2cFailedXact;

#ifdef I2c_Debug
	if( Dev->err ) {
		printf( "WR: I2C Host error\n" );
	} else if( debugGrant & 0x100 ) {
		printf( "WR: Reg:%02x, Dat:%02x\n", index, data );
	}
#endif
}

/* reads a block of bytes from the device */

void
vl_rdBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	/* quick 'n' dirty interface to existing i2c functions */
	for( k = 0; k < count; k++ ) {
		vl_rdByte( Dev, index++, pdata++ );
	}
}

/* Read 32-bit word from register, msb first */

void
vl_rdDWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint32_t	*data
) {
	vl_rdBlock( Dev, index, (uint8_t *) data, 4 );
#if BYTE_ORDER == LITTLE_ENDIAN
	vl_rev_bytes( (uint8_t *) data, 4 );
#endif
}

/* read 16-bit word from register, msb first */

void
vl_rdWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint16_t	*data
) {
	vl_rdBlock( Dev, index, (uint8_t *) data, 2 );
#if BYTE_ORDER == LITTLE_ENDIAN
	vl_rev_bytes( (uint8_t *) data, 2 );
#endif
}

/* read single byte from register */

void
vl_rdByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*data
) {
	extern		uint16_t	debugGrant;

	if( Dev->err ) return;
	if( i2c_trans( Dev->i2cDevAddr | I2c_Write, & index, 1 ) )
		Dev->err = VLerr_I2cFailedXact;

	if( i2c_trans( Dev->i2cDevAddr | I2c_Read, data, 1 ) )
		Dev->err = VLerr_I2cFailedXact;

#ifdef I2c_Debug
	if( Dev->err ) {
		printf( "RD: I2C Host error\n" );
	} else if( debugGrant & 0x100 ) {
		printf( "RD: Reg:%02x, Dat:%02x\n", index, *data );
	}
#endif
}

/* read-modify-write single byte register
|
| Final_reg = ( Initial_reg & and_data ) | or_data
*/

void
vl_updateByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		AndData,
	uint8_t		OrData
) {
    uint8_t		data;

    vl_rdByte( Dev, index, & data );
    data = ( data & AndData ) | OrData;
    vl_wrByte( Dev, index, data );
}

/*
| delay for device polling
*/

void
vl_pollDelay( VL_Dev_t *Dev
) {
	if( Dev->err ) return;
	cpu_spinUs( VL_PollDelayMs * 1000 );
}

