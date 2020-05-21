/*------------------------------------------------------------------------
| i2c_686.c  --  platform-specific i2c interface functions
|
| Copyright (c) 2020 Gord Seiferling.
| Refer to license-app.txt.
|
| target: QNX with LB700 SBC with VT82C686B Southbridge
|
| Provide 8/16/32-bit register access via i2c.
| Every write transaction MUST have the index as 1st byte.
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

uint16_t	smBaseAddr = 0xefa0;		/* base address of SMBus I/O */

/*---  local function protos  ---*/
VL_Err_t i_xact( uint8_t, uint8_t, uint8_t );
void i_showSmbRegs();

/* writes the supplied byte buffer to the device */

void
vl_wrBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	if( Dev->err ) return;
	/*---  load data into the controller  ---*/
	in8( smBaseAddr + 0x02 );		/* reset the buffer index */
	for( k = 0; k < count; k++ ) {
		out8( smBaseAddr + 0x07, pdata[ k ] );
	}
	out8( smBaseAddr + 0x05, count );
	Dev->err = i_xact( Dev->i2cDevAddr, index, 0xd );
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

/* reads a block of bytes from the device
| Useful for efficiently reading many single-byte registers.
*/

void
vl_rdBlock( VL_Dev_t *Dev,
	uint8_t		index,
	uint8_t		*pdata,
	uint8_t		count
) {
	int		k;

	if( Dev->err ) return;
	out8( smBaseAddr + 0x05, count );
	Dev->err = i_xact( Dev->i2cDevAddr | 1, index, 0xd );
	in8( smBaseAddr + 0x02 );		/* reset the buffer index */
	for( k = 0; k < count; k++ ) {
		pdata[ k ] = in8( smBaseAddr + 0x07 );
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
| operate the SMBus controller in the '686 Southbridge.
| perform as much of a common transaction as possible.
|
| Neither the IRQ enable (PCI config Rxd2 bit 1) nor the controller
| done interrupt enable (offset 0x02 bit 0) need to be set for the
| transaction to complete.
|
| SMBus controller function codes:
| 0 - quick read/write: 9 SCK pulses, sends 8-bit SLA & gets ACK
| 1 - byte read/write: 18 SCK pulses, SLA, one byte read/write
| 2 - byte data read/write: 36 SCK pulses:
|		Start, SLA+W, HostCmd, Repeat Start, SLA+R/W, Host Data
|		1st cycle is cmd write, 2nd cycle is data byte read/write.
| 3 - word data read/write: 45 SCK pulses:
|		Similar to func #2, but 2 data bytes in 2nd transaction.
| 4 - process call: (no repeat start)
|		Start, SLA+W, HostCmd, HostData 0, HostData 1.
|		Start, SLA+R, HostData 0, HostData 1.	(no HostCmd)
| 5 - Block read/write
|		Write: SLA+W, HostCmd, Repeat Start, SLA+W, <count> bytes
|		Read: SLA+W, HostCmd, Repeat Start, SLA+R, <reads until device NACK>
| d - I2C Block
|		Write: SLA+W, HostCmd, <count> bytes
|		Read: SLA+W, HostCmd, Repeat Start, SLA+R, <count> bytes
|
| Repeat Starts have not been verified.
|
| Other findings:
| - LB700 manual incorrectly states MAX1617 is at address 32.  Actually 30.
| - LB700 manual has SMBus Data & Clk pins reversed on J24 (13=Data,14=Clk)
|
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

	/*---  clear controller errors  ---*/
	out8( smBaseAddr + 0x00, 0x1e );

	/*---  do a transaction  ---*/
	out8( smBaseAddr + 0x03, index );	/* host cmd, VL53 reg # */
	out8( smBaseAddr + 0x04, slaRw );
	out8( smBaseAddr + 0x02, ( prot << 2 ) | 0x40 );	/* go */

	/*---  crude busy duration  ---*/
	for( bCnt = 0; bCnt < 1000000; bCnt++ ) {
		if( ! ( in8( smBaseAddr + 0x00 ) & 0x01 ) ) break;
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

