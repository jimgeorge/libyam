/**
\file modbus.c
\brief Modbus/RTU protocol handling.
\author Jim George

This module creates Modbus/RTU packets for transmission, and interprets
received packets. Actual serial I/O is handled in the serial.c module.
*/

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <poll.h>
#include <errno.h>
#include <arpa/inet.h>

#include "modbus.h"
#include "serial.h"

#define PACKED __attribute__((__packed__))

/* Table of CRC values for high-order byte */
static uint8_t crc_table_hi[] = {
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
        0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 
        0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 
        0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static uint8_t crc_table_lo[] = {
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 
        0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 
        0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 
        0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 
        0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 
        0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
        0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 
        0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 
        0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 
        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 
        0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
        0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 
        0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 
        0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 
        0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 
        0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 
        0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 
        0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/**
\brief Compute CRC of the specified buffer, using the Modbus CRC tables
\param *buffer Input buffer
\param buffer_length Length of input buffer
\return The computed CRC
*/
static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length)
{
        uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
        uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
        unsigned int i; /* will index into CRC lookup */

        /* pass through message buffer */
        while (buffer_length--) {
                i = crc_hi ^ *buffer++; /* calculate the CRC  */
                crc_hi = crc_lo ^ crc_table_hi[i];
                crc_lo = crc_table_lo[i];
        }

        return (crc_hi << 8 | crc_lo);
}

/**
\brief Initialize a YAM object with the specified parameters
\param *device_name Name of serial port device to use
\param speed Speed of serial port (in bps)
\param *bus The YAM object representing the Modbus
\return YAM_OK on success, YAM_SERIAL_INIT_FAILED on failure

This function initializes a YAM object. The specified serial device is opened
with the specified bus speed, in 8-bit mode, without handshaking, no parity,
1 stop bit (8-N-1). The YAM object is returned in the *bus parameter, which
must be allocated prior to calling this function. If an error occurs, no
change is made to the bus parameter, and -1 is returned.
*/
int yam_modbus_init(const char *device_name,
             unsigned int speed,
             struct yam_modbus *bus)
{
	int port;
	assert(device_name != NULL);
	assert(bus != NULL);

	/* First initialize the serial port */
	if (-1 == serial_port_init(device_name, speed, &port)) {
		return (bus->last_errorcode = YAM_SERIAL_INIT_FAILED);
	}
	bzero(bus, sizeof(struct yam_modbus));
	bus->serial = port;
	bus->baudrate = speed;
	bus->timeout_ms = YAM_DEFAULT_TIMEOUT;
	strncpy(bus->device_name, device_name, YAM_MAX_DEVICE_NAME);

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Close the interface to the specified modbus object
\param *bus The YAM object representing the Modbus

This function closes the interface specified by the YAM object. The associated
serial port is closed, but the YAM object is left untouched.
*/
void yam_modbus_close(struct yam_modbus *bus)
{
	assert(bus != NULL);
	close(bus->serial);
}

/**
\brief Change the debugging status of the specified YAM object
\param *bus The YAM object representing the Modbus
\param debug_status 0 to disable debug, nonzero to enable

Changes the debugging status of the specified YAM object. When debugging is
enabled, YAM logs all serial port I/O to the console.
*/
void yam_debug(struct yam_modbus *bus, int debug_status)
{
	assert(bus != NULL);
	bus->debug = debug_status;
}

/**
\brief Send generic Modbus/RTU packet to the specified address
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param *pdu Protocol Data Unit (payload) to send to the address
\param pdu_len Length of the PDU

Sends the specified PDU (payload) to the specified address on the bus. The
CRC is computed before sending.
*/
void yam_send_generic_packet(struct yam_modbus *bus, uint8_t addr,
                            uint8_t *pdu, uint8_t pdu_len)
{
	assert(bus != NULL);
	assert(pdu != NULL);
	assert((pdu_len + 3) < YAM_MODBUS_MAX_ADU_LEN);
	
	/* The Application Data Unit (ADU) is the PDU + address and CRC. */
	uint8_t adu[YAM_MODBUS_MAX_ADU_LEN];
	uint8_t adu_len = 0;
	
	adu[adu_len++] = addr;
	memcpy(adu + 1, pdu, pdu_len);	adu_len += pdu_len;
	uint16_t crc = crc16(adu, pdu_len + 1);
	adu[adu_len++] = crc >> 8;
	adu[adu_len++] = crc & 0x00FF;

	if (bus->debug) {
		printf("Generic send packet to %02X: CRC = %04X, "
			"PDU: %d bytes, ADU: %d bytes\n", addr, crc, pdu_len, adu_len);
		int ctr;
		for (ctr = 0; ctr < pdu_len + 3; ctr++) {
			printf("[%.2X]", adu[ctr]);
		}
		printf("\n");
	}
	write(bus->serial, adu, adu_len);
}

/**
\brief Read back a packet from Modbus/RTU and interpret the results
\param *bus The YAM object representing the Modbus
\param *addr Address of the replying Modbus device
\param *pdu Buffer where PDU will be stored
\param *pdu_len Length of the PDU
\return YAM_OK on success, error code on failure

This function reads back a packet of data from the Modbus/RTU, and splits
it up into the ADU and PDU. The CRC is also verified.
*/
int yam_read_generic_packet(struct yam_modbus *bus, uint8_t *addr,
                            uint8_t *pdu, uint8_t *pdu_len)
{
	assert(bus != NULL);
	assert(addr != NULL);
	assert(pdu != NULL);
	assert(pdu_len != NULL);

	enum {ADDR, FUNC, GETBYTECOUNT, READEXCEPTION, DATA, CRC, DONE, ERROR} state = ADDR;
	
	uint8_t adu[YAM_MODBUS_MAX_ADU_LEN];
	uint8_t adu_len = 0;
	int bytes_to_read = 1; /* Prime the reader, to read in the source addr */
	int bytes_read;
	int errcode = YAM_TIMEOUT;

	do {
		/* First wait until there's something to read from port */
		struct pollfd pfd;
		pfd.fd = bus->serial;
		pfd.events = POLLIN;
		pfd.revents = 0;
	
		int ret;
		do {
			ret = poll(&pfd, 1, bus->timeout_ms);
		} while ((ret == -1) && (errno == EINTR));
		if (ret == 0) {
			return YAM_TIMEOUT;
		}
		
		/* Now read the appropriate number of bytes, as determined by the
		state machine */
		bytes_read = read(bus->serial, &adu[adu_len], bytes_to_read);
		if (bus->debug) {
			int ctr;
			for (ctr = 0; ctr < bytes_read; ctr++) {
				printf("<%.2X>", adu[adu_len + ctr]);
			}
		}
		bytes_to_read -= bytes_read;
		adu_len += bytes_read;
		
		/* If we're still waiting for bytes, don't enter the state machine, so
		the next time around, the read will fetch the remaining bytes */
		if (bytes_to_read == 0) {
			switch (state) {
			case ADDR:
				/* Just done reading address, need to read fn code */
				bytes_to_read = 1;
				state = FUNC;
				break;
			case FUNC:
				/* Just done reading code, use code to determine packet size,
				if possible. Some packets return a byte count, if this is the
				case, enter the GETBYTECOUNT state */
				/* If the highest bit is not set, this is not a reply
				character, this is an error condition */
				if (!(adu[adu_len - 1] & 0x80)) {
					switch (adu[adu_len - 1]) {
					case YAM_READ_COILS:
						bytes_to_read = 1;
						state = GETBYTECOUNT;
						break;
					case YAM_READ_DISCRETES:
						bytes_to_read = 1;
						state = GETBYTECOUNT;
						break;
					case YAM_READ_REGISTERS:
						bytes_to_read = 1;
						state = GETBYTECOUNT;
						break;
					case YAM_READ_INPUTS:
						bytes_to_read = 1;
						state = GETBYTECOUNT;
						break;
					case YAM_WRITE_SINGLECOIL:
						bytes_to_read = 4;
						state = DATA;
						break;
					case YAM_WRITE_SINGLEREGISTER:
						bytes_to_read = 4;
						state = DATA;
						break;
					case YAM_READ_EXCEPTIONSTATUS:
						bytes_to_read = 1;
						state = DATA;
						break;
					case YAM_WRITE_COILS:
						bytes_to_read = 4;
						state = DATA;
						break;
					case YAM_WRITE_REGISTERS:
						bytes_to_read = 4;
						state = DATA;
						break;
					case YAM_REPORTSLAVEID:
						bytes_to_read = 1;
						state = GETBYTECOUNT;
						break;
					default:
						errcode = YAM_ILLEGAL_FUNCTION;
						state = ERROR;
						break;
					} /* switch (adu[1] & 0x7F) */
				} /* if (adu[1] & 0x80) */
				else {
					/* Read the exception code */
					state = READEXCEPTION;
					bytes_to_read = 1;
				}
				break;
			case GETBYTECOUNT:
				/* Byte count encoded in the byte just received */
				bytes_to_read = adu[adu_len - 1];
				if (bytes_to_read > YAM_MODBUS_MAX_PDU_LEN) {
					errcode = YAM_INVALIDBYTECOUNT;
					state = ERROR;
				}
				state = DATA;
				break;
			case READEXCEPTION:
				errcode = -1 * adu[adu_len - 1];
				state = ERROR;
				break;
			case DATA:
				bytes_to_read = 2;
				state = CRC;
				break;
			case CRC:
				bytes_to_read = 0;
				state = DONE;
				break;
			case DONE:
			case ERROR:
				break;
			}
		}
	} while ((state != DONE) && (state != ERROR));
	
	if (bus->debug) {
		printf("\nadu_len = %d\n", adu_len);
	}

	/* Check to see if we encountered any errors during receive */
	if (state == ERROR) {
		/* We may be out of sync, flush buffers */
		serial_port_flush(bus->serial);
		return errcode;
	}

	/* CRC computed over buffer (including recv'd CRC) should be zero */
	if(0 != crc16(adu, adu_len)) {
		return YAM_CRC_ERROR;
	}

	*addr = adu[0];
	/* Copy the PDU to buffer, skipping address and CRC bytes */
	if (adu_len < 3) {
		*pdu_len = adu_len;
	}
	else {
		*pdu_len = adu_len - 3;
	}
	memcpy(pdu, adu + 1, *pdu_len);
	return YAM_OK;
}

#define MAX_ERRORS 12
static struct {
	int errnum;
	char error_string[100];
} error_table[MAX_ERRORS] = {
	{YAM_ILLEGAL_FUNCTION, "Illegal Function"},
	{YAM_ILLEGAL_DATA_ADDR, "Illegal Register Address"},
	{YAM_ILLEGAL_DATA_VALUE, "Illegal Data Value"},
	{YAM_SLAVE_FAILURE, "Slave Failure"},
	{YAM_ACKNOWLEDGE, "Acknowledge"},
	{YAM_SLAVE_BUSY, "Slave Busy"},
	{YAM_PARITY_ERROR, "Parity Error"},
	{YAM_CRC_ERROR, "CRC Error"},
	{YAM_TIMEOUT, "Timeout"},
	{YAM_SERIAL_INIT_FAILED, "Serial Initialization Failed"},
	{YAM_INVALIDBYTECOUNT, "Invalid Byte Count"},
};

static char *unknown_err = "Unknown Error";

/**
\brief Return string corresponding to error number
\param *bus The YAM object representing the Modbus
\param errnum Error number
\return String describing the error

Returns a string corresponding to the specified error number. Note that the
returned string is static, and must not be freed.
*/
char *yam_strerror(int errnum)
{
	int ctr;
	for (ctr = 0; ctr < MAX_ERRORS; ctr++) {
		if (error_table[ctr].errnum == errnum) {
			return error_table[ctr].error_string;
		}
	}
	return unknown_err;
}

/**
\brief Print error to stderr
\param *bus The YAM object representing the Modbus
\param *s Identifier string, printed along with error message

Prints out errors associated with the last YAM call made on the specified
YAM object.
*/
void yam_perror(struct yam_modbus *bus, char *s)
{
	assert(bus != NULL);
	
	printf("%s: %s\n", s, yam_strerror(bus->last_errorcode));
}

/**
\brief Read "holding" registers from the specified target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param start_addr Address of the first register to read from the target
\param num_regs Number of registers to read from the target
\param *regs Location to store the registers
\return YAM_OK on success, error code on failure

Read one or more holding registers from the Modbus/RTU target. The results
are placed in *regs. If an error occurs, *regs is unmodified, and the
error code is returned. On success, YAM_OK is returned.
*/
int yam_read_registers(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_regs,
                       uint16_t *regs)
{
	assert(bus != NULL);
	assert(num_regs < YAM_REGS_PER_REQUEST);
	assert(num_regs != 0);
	
	int ret;

	struct read_holding_reg {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_regs;
	} PACKED pdu;
	
	pdu.fncode = YAM_READ_REGISTERS;
	pdu.start_addr = htons(start_addr);
	pdu.num_regs = htons(num_regs);

	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	
	struct read_holding_reg_ret {
		uint8_t fncode;
		uint8_t bytecount;
		uint16_t reg[0];
	} PACKED ret_pdu;

	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu, &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}
	/* Check if the byte count is odd */
	if (ret_pdu.bytecount & 0x01) {
		return (bus->last_errorcode = YAM_INVALIDBYTECOUNT);
	}
	int num_ret_regs = ret_pdu.bytecount / 2;
	if (num_ret_regs != num_regs) {
		return (bus->last_errorcode = YAM_INVALIDBYTECOUNT);
	}
	int ctr;
	for (ctr = 0; ctr < num_ret_regs; ctr++ ) {
		regs[ctr] = ntohs(ret_pdu.reg[ctr]);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Read inputs (read-only registers) from the specified target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param start_addr Address of the first register to read from the target
\param num_regs Number of registers to read from the target
\param *regs Location to store the registers
\return 0 on success, error code on failure

Read one or more inputs from the Modbus/RTU target. The results
are placed in *regs. If an error occurs, *regs is unmodified, and the
error code is returned. On success, YAM_OK is returned.
*/
int yam_read_inputs(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_regs,
                       uint16_t *regs)
{
	assert(bus != NULL);
	assert(num_regs < YAM_REGS_PER_REQUEST);
	assert(num_regs != 0);
	
	int ret;

	struct read_input {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_regs;
	} PACKED pdu;
	
	pdu.fncode = YAM_READ_INPUTS;
	pdu.start_addr = htons(start_addr);
	pdu.num_regs = htons(num_regs);

	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	
	struct read_input_ret {
		uint8_t fncode;
		uint8_t bytecount;
		uint16_t reg[0];
	} PACKED ret_pdu;

	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu, &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}
	if (ret_pdu.bytecount & 0x01) {
		return (bus->last_errorcode = YAM_INVALIDBYTECOUNT);
	}
	int num_ret_regs = ret_pdu.bytecount / 2;
	if (num_ret_regs != num_regs) {
		return (bus->last_errorcode = YAM_INVALIDBYTECOUNT);
	}
	int ctr;
	for (ctr = 0; ctr < num_ret_regs; ctr++ ) {
		regs[ctr] = ntohs(ret_pdu.reg[ctr]);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Write a single coil on the specified target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param coil_addr Address of the coil within the target
\param coil_state Desired state of the coil (0 = off, nonzero = on)
\return 0 on success, error code on failure

Write to a single coil on the Modbus/RTU target.
*/
int yam_write_single_coil(struct yam_modbus *bus, uint8_t addr,
                       uint16_t coil_addr, uint8_t coil_state)
{
	assert(bus != NULL);

	int ret;

	struct {
		uint8_t fncode;
		uint16_t output_addr;
		uint16_t output_value;
	} PACKED pdu, ret_pdu;
	
	pdu.fncode = YAM_WRITE_SINGLECOIL;
	pdu.output_addr = htons(coil_addr);
	pdu.output_value = htons(coil_state ? 0xFF00 : 0x0000);

	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu, &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Write a single holding register on the specified target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param register_addr Address of the holding register within the target
\param register_value Desired value of the register
\return 0 on success, error code on failure

Write to a single holding register on the Modbus/RTU target.
*/
int yam_write_single_register(struct yam_modbus *bus, uint8_t addr,
                       uint16_t register_addr, uint16_t register_value)
{
	assert(bus != NULL);

	int ret;

	struct {
		uint8_t fncode;
		uint16_t output_addr;
		uint16_t output_value;
	} PACKED pdu, ret_pdu;
	
	pdu.fncode = YAM_WRITE_SINGLEREGISTER;
	pdu.output_addr = htons(register_addr);
	pdu.output_value = htons(register_value);

	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu, &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Read Exception Status from the specified target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param *exception_status Location where exception status will be stored
\return 0 on success, error code on failure

Read the exception status word from the specified Modbus/RTU slave.
*/
int yam_read_exception_status(struct yam_modbus *bus, uint8_t addr,
                              uint8_t *exception_status)
{
	assert(bus != NULL);

	int ret;

	struct {
		uint8_t fncode;
	} PACKED pdu;
	
	struct {
		uint8_t fncode;
		uint8_t exceptions;
	} PACKED ret_pdu;
	
	pdu.fncode = YAM_READ_EXCEPTIONSTATUS;

	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu, &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}
	*exception_status = ret_pdu.exceptions;

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Write to multiple coils on the Modbus target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param *coils Buffer containing coils to be written
\param num_coils Number of coils within the buffer
\return 0 on success, error code on failure

Write to multiple coils on the Modbus target. Each byte in the *coils buffer
represents one coil, a zero value turns the coil off, a non-zero value turns
the coil on.
*/
int yam_write_multiple_coils(struct yam_modbus *bus, uint8_t addr,
                             uint16_t start_addr, uint8_t *coils,
                             uint16_t num_coils)
{
	assert(bus != NULL);
	assert(coils != NULL);
	assert(num_coils < YAM_COILS_PER_REQUEST);

	int ret;

	struct {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_coils;
		uint8_t byte_count;
		uint8_t packed_coils[YAM_MODBUS_MAX_PDU_LEN];
	} PACKED pdu;
	
	struct {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_coils;
	} PACKED ret_pdu;
	
	pdu.fncode = YAM_WRITE_COILS;
	pdu.start_addr = htons(start_addr);
	pdu.num_coils = htons(num_coils);
	
	bzero(pdu.packed_coils, YAM_MODBUS_MAX_PDU_LEN);
	int ctr;
	for (ctr = 0; ctr < num_coils; ctr++) {
		if (coils[ctr]) {
			pdu.packed_coils[ctr / 8] |= (1 << (ctr % 8));
		}
	}
	pdu.byte_count = num_coils / 8 + 1;
	/* For this call, we must calculate the number of bytes, since
	sizeof will return even those members of packed_coils that are unused */
	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu),
	                        sizeof(pdu) - YAM_MODBUS_MAX_PDU_LEN +
	                        pdu.byte_count);
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu,
	                              &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Write to multiple registers on the Modbus target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param *regs Buffer containing registers to be written
\param num_registers Number of registers within the buffer
\return 0 on success, error code on failure

Write to multiple coils on the Modbus target. Each byte in the *coils buffer
represents one coil, a zero value turns the coil off, a non-zero value turns
the coil on.
*/
int yam_write_multiple_registers(struct yam_modbus *bus, uint8_t addr,
                                 uint16_t start_addr, uint16_t *regs,
                                 uint16_t num_regs)
{
	assert(bus != NULL);
	assert(regs != NULL);
	assert(num_regs < YAM_REGS_PER_REQUEST);

	int ret;

	struct {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_regs;
		uint8_t byte_count;
		uint16_t regs[YAM_REGS_PER_REQUEST];
	} PACKED pdu;
	
	struct {
		uint8_t fncode;
		uint16_t start_addr;
		uint16_t num_regs;
	} PACKED ret_pdu;
	
	pdu.fncode = YAM_WRITE_REGISTERS;
	pdu.start_addr = htons(start_addr);
	pdu.num_regs = htons(num_regs);
	int ctr;
	for (ctr = 0; ctr < num_regs; ctr++) {
		pdu.regs[ctr] = htons(regs[ctr]);
	}
	pdu.byte_count = num_regs * 2;

	/* For this call, we must calculate the number of bytes, since
	sizeof will return even those members of regs that are unused */
	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu),
	                        sizeof(pdu) - YAM_REGS_PER_REQUEST *
	                        sizeof(uint16_t) + pdu.byte_count);
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu,
	                              &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}

	return (bus->last_errorcode = YAM_OK);
}

/**
\brief Write to multiple registers on the Modbus target
\param *bus The YAM object representing the Modbus
\param addr Address of the target Modbus device
\param *id Location where Slave ID will be written
\param *run_status Location where Run Status will be written
\param *additional_data Buffer to store additional data from slave
\param *buflen Location where additional data length is stored.
\return 0 on success, error code on failure

Obtains the ID, run status and additional information from the specified slave
address. The run_status and additional_data parameters are optional, and can
be set to NULL if they are not needed.

The additional data buffer should be at least 256 bytes long.
*/
int yam_report_slave_id(struct yam_modbus *bus, uint8_t addr, uint8_t *id,
                        uint8_t *run_status, char *additional_data, int *buflen)
{
	assert(bus != NULL);
	assert(id != NULL);

	int ret;

	struct {
		uint8_t fncode;
	} PACKED pdu;
	
	struct {
		uint8_t fncode;
		uint8_t byte_count;
		uint8_t slave_id;
		uint8_t run_status;
		uint8_t addl_data[1];
	} PACKED ret_pdu;
	
	pdu.fncode = YAM_REPORTSLAVEID;
	yam_send_generic_packet(bus, addr, (uint8_t *)(&pdu), sizeof(pdu));
	uint8_t ret_addr, ret_pdu_len;
	ret = yam_read_generic_packet(bus, &ret_addr, (uint8_t *)&ret_pdu,
	                              &ret_pdu_len);
	if (0 > ret) {
		return (bus->last_errorcode = ret);
	}
	*id = ret_pdu.slave_id;
	if (run_status) {
		*run_status = ret_pdu.run_status;
	}
	if (additional_data) {
		memcpy(additional_data, ret_pdu.addl_data, ret_pdu.byte_count - 2);
		*buflen = ret_pdu.byte_count - 2;
	}

	return (bus->last_errorcode = YAM_OK);
}

