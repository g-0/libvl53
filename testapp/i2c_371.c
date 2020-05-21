/*------------------------------------------------------------------------
| i2c_371.c  --  platform-specific i2c interface functions
|
| Copyright (c) 2020 Gordon Seiferling.
| Refer to license-app.txt.
|
| target: QNX with Slot1 MB with 82371 Southbridge
|
| Provide 8/16/32-bit register access via i2c.
|
| Common arguments:
| Dev       Device Handle
| index     The VL53L0x register index
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <hw/inout.h>
#include "../vl53/vl53_api.h"

uint16_t	smBaseAddr = 0x5000;	/* base address of SMBus controller */

/*---  local function protos  ---*/
VL_Err_t i_xact( uint8_t, uint8_t, uint8_t );
void i_showSmbRegs();

/* writes the supplied byte buffer to the device
| The older '371 does not have support for 'i2c mode' SMBus protocol.
| This makes longer transfers challenging.  For simplicity,
| all writes >= 4 bytes are sequenced as single-byte data transfers,
| with the index stepped (and explicitly sent) at each transaction.  :-(
*/

void
vl_wrBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	for( k = 0; k < count; k++ ) {
		vl_wrByte( Dev, index++, *pdata++ );
	}
}

/* write 32-bit word to the device, msb first */

void
vl_wrDWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint32_t	data
) {
#if BYTE_ORDER == LITTLE_ENDIAN
	vl_rev_bytes( (uint8_t *) &data, 4 );
#endif
	vl_wrBlock( Dev, index, (uint8_t *) &data, 4 );
}

/* write 16-bit word to the device, msb first */

void
vl_wrWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint16_t	data
) {
	if( Dev->err ) return;
	out8( smBaseAddr + 0x05, data >> 8 );	/* host data 0, msb */
	out8( smBaseAddr + 0x06, data );		/* host data 1, lsb */
	Dev->err = i_xact( Dev->i2cDevAddr, index, 3 );
}

/* write single byte to the device */

void
vl_wrByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		data
) {
	if( Dev->err ) return;
	out8( smBaseAddr + 0x05, data );	/* host data 0 */
	Dev->err = i_xact( Dev->i2cDevAddr, index, 2 );
}

/* reads a block of bytes from the device */

void
vl_rdBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	if( Dev->err ) return;
	/*---  read the first byte as we normally would  ---*/
	Dev->err = i_xact( Dev->i2cDevAddr | 1, index, 2 );
	pdata[ 0 ] = in8( smBaseAddr + 0x05 );		/* host data 0 */

	/*---  read the remainder as 'single-byte' transfers  ---*/
	for( k = 1; k < count; k++ ) {
		Dev->err |= i_xact( Dev->i2cDevAddr | 1, 0xee, 1 );
		pdata[ k ] = in8( smBaseAddr + 0x05 );
	}
}

/* Read 32-bit word from the device, msb first */

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

/* read 16-bit word from the device, msb first */

void
vl_rdWord( VL_Dev_t *Dev,
	uint8_t		index,
	uint16_t	*data
) {
	if( Dev->err ) return;
	Dev->err = i_xact( Dev->i2cDevAddr | 1, index, 3 );
	*data = ( in8( smBaseAddr + 0x05 ) << 8 ) | in8( smBaseAddr + 0x06 );
}

/* read single byte from the device */

void
vl_rdByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*data
) {
	if( Dev->err ) return;
	Dev->err = i_xact( Dev->i2cDevAddr | 1, index, 2 );
	*data = in8( smBaseAddr + 0x05 );		/* host data 0 */
}

/* read-modify-write single byte register
|
| result = (initial & AndData) | OrData
*/

void
vl_updateByte( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		AndData,
	uint8_t		OrData
) {
    uint8_t		data;

    vl_rdByte( Dev, index, &data );
    data = ( data & AndData ) | OrData;
    vl_wrByte( Dev, index, data );
}

/*
| handler for intel FW82371 Southbridge SMB controller.
|
| Controller registers:
| 0 - Host Status d0:busy, d1:int/complete, d2:dev err, d3:collision, d4:failed
|	Reset d1/d2/d3/d4 by writing 1 to the bit
| 1 - Slave Status
| 2 - Host Ctrl, d0:int enab, d1:kill, d4-d2:fcn, d6:start
|	000 - quick r/w (1 byte)
|	001 - byte r/w (2 bytes)
|	010 - byte data r/w	(27 SCLKs for wr, 36 SCLKs for read)
|	011 - word data r/w (4 bytes for write, 5 for read)
|	101 - block data r/w (write count, read until slave NAK)
|	all others reserved
| 3 - Host Command (byte following SLA in 'data' transactions)
| 4 - Host Address (SLA + R/W bit)
| 5 - Data 0 (1st byte, byte/word data).  block tx count. rx count set here.
| 6 - Data 1
| 7 - Block Data (read reg 2 to reset index)
| 8 - Slave Control
| 9 - Shadow Command (command received from external master)
| 
| Any write transaction MUST have the index as 1st byte.
*/

VL_Err_t
i_xact(
	uint8_t		slaRw,		/* slave addr + r/w bit */
	uint8_t		index,		/* command byte (reg index) */
	uint8_t		prot		/* controller SMB protocol */
) {
	int		bCnt;
	uint8_t	smbStat;		/* host transaction status */
	extern uint16_t	debugGrant;

	/*---  test controller busy  ---*/
	if( in8( smBaseAddr + 0x00 ) & 0x01 ) return( VLerr_I2cBusy );

	/*---  clear completion and controller errors  ---*/
	out8( smBaseAddr + 0x00, 0x1e );

	/*---  do a transaction  ---*/
	out8( smBaseAddr + 0x03, index );	/* host cmd, VL53 reg # */
	out8( smBaseAddr + 0x04, slaRw );
	out8( smBaseAddr + 0x02, ( prot << 2 ) | 0x40 );	/* go */

	/*---  crude busy duration  ---*/
	for( bCnt = 0; bCnt < 1000000; bCnt++ ) {
		if( ( in8( smBaseAddr + 0x00 ) & 0x1e ) ) break;
	}
	smbStat = in8( smBaseAddr + 0x00 );

#ifdef I2c_Debug
	if( debugGrant & 0x100 ) {
		printf( "OP:%02x, SLA:%02x%c, Reg:%02x, Dat:%02x, #polls:%d\n",
			prot, slaRw & 0xfe, slaRw & 1 ? 'R' : 'W',
			index,
			in8( smBaseAddr + 5 ),
			bCnt );
	}
	/*---  dump controller regs  ---*/
	if( debugGrant & 0x200 ) i_showSmbRegs();
	if( smbStat & 0x1c ) printf( "SMB Host error: %02x\n", smbStat );
#endif

	if( smbStat & 0x10 ) return( VLerr_I2cFailedXact );
	if( smbStat & 0x08 ) return( VLerr_I2cCollision );
	if( smbStat & 0x04 ) return( VLerr_I2cDeviceError );
	return( VLerr_None );
}

/*
| delay for device polling
*/

void
vl_pollDelay( VL_Dev_t *Dev
) {
	if( Dev->err ) return;
	delay( VL_PollDelayMs );
}

#ifdef I2c_Debug
void
i_showSmbRegs(
) {
	uint8_t		ofs;

	printf( "SMB regs:" );
	for( ofs = 0; ofs < 0x10; ofs++ )
		printf( " %02x", in8( smBaseAddr + ofs ) );
}
#endif

