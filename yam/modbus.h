/**
\file modbus.h
\brief Include file for YAM - Yet Another Modbus/RTU library
\author Jim George
*/

#ifndef _YAM_MODBUS_H_
#define _YAM_MODBUS_H_

#include <stdint.h>

#define YAM_MAX_DEVICE_NAME 64

/**
\brief The YAM object

This structure represents the YAM object. One may be created using the
yam_modbus_init function. All calls to the YAM library require a YAM object,
to know which modbus serial device to use. This permits multiple serial
devices to be open simultaneously.
*/
struct yam_modbus {
	int serial; /**< Serial port file descriptor */
	int baudrate; /**< Baud rate */
	int debug; /**< Nonzero to enable debug stuff to stdout */
	int timeout_ms; /**< Timeout, in milliseconds, when reading */
	int last_errorcode; /**< Last error code seen by this bus */
	char device_name[YAM_MAX_DEVICE_NAME]; /**< Name of the serial device */
};

/* MODBUS Function codes */
#define YAM_READ_COILS 0x01
#define YAM_READ_DISCRETES 0x02
#define YAM_READ_REGISTERS 0x03
#define YAM_READ_INPUTS 0x04
#define YAM_WRITE_SINGLECOIL 0x05
#define YAM_WRITE_SINGLEREGISTER 0x06
#define YAM_READ_EXCEPTIONSTATUS 0x07
/*
#define YAM_READ_DIAGNOSTICS 0x08
#define YAM_READ_COMMEVTCOUNTER 0x0B
#define YAM_READ_COMMEVTLOG 0x0C
*/
#define YAM_WRITE_COILS 0x0F
#define YAM_WRITE_REGISTERS 0x10
#define YAM_REPORTSLAVEID 0x11

/* MODBUS Exception codes */
/** Return code - slave does not respond to specified function code */
#define YAM_ILLEGAL_FUNCTION -1
/** Return code - slave reports invalid address */
#define YAM_ILLEGAL_DATA_ADDR -2
/** Return code - slave reports that value written to register is invalid */
#define YAM_ILLEGAL_DATA_VALUE -3
/** Return code - slave reports a general failure */
#define YAM_SLAVE_FAILURE -4
/** Return code - slave acknowledges the request but did not process it */
#define YAM_ACKNOWLEDGE -5
/** Return code - slave is busy and cannot process the request */
#define YAM_SLAVE_BUSY -6
/** Return code - command sent to slave had a parity or CRC error */
#define YAM_PARITY_ERROR -8

/* Exeception codes returned by YAM */
/** Return code - everything's OK */
#define YAM_OK 0
/** Return code - response from slave had a bad CRC */
#define YAM_CRC_ERROR -256
/** Return code - slave timed out */
#define YAM_TIMEOUT -257
/** Return code - invalid number of bytes returned by slave */
#define YAM_INVALIDBYTECOUNT -258
/** Return code - serial port init failed */
#define YAM_SERIAL_INIT_FAILED -259
/** Return code - too many registers/coils (exceeds ADU size) */
#define YAM_TOO_MANY_REGISTERS -260

/** Maximum ADU length, in bytes */
#define YAM_MODBUS_MAX_ADU_LEN 256
/** Maximum PDU length, in bytes */
#define YAM_MODBUS_MAX_PDU_LEN 253
/** Maximum number of registers per request */
#define YAM_REGS_PER_REQUEST 123
/** Maximum number of coils per request */
#define YAM_COILS_PER_REQUEST 1968
/** Default timeout of a request, in milliseconds */
#define YAM_DEFAULT_TIMEOUT 1000

int yam_modbus_init(const char *device_name,
             unsigned int speed,
             struct yam_modbus *bus);
void yam_modbus_close(struct yam_modbus *bus);
void yam_debug(struct yam_modbus *bus, int debug_status);
void yam_set_timeout(struct yam_modbus *bus, int timeout_ms);
int yam_get_serial_device(struct yam_modbus *bus);

int yam_read_coils(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_coils,
                       uint8_t *coils);
int yam_read_discretes(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_discretes,
                       uint8_t *discretes);
int yam_read_registers(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_regs,
                       uint16_t *regs);
int yam_read_inputs(struct yam_modbus *bus, uint8_t addr,
                       uint16_t start_addr, uint16_t num_regs,
                       uint16_t *regs);
int yam_write_single_coil(struct yam_modbus *bus, uint8_t addr,
                       uint16_t coil_addr, uint8_t coil_state);
int yam_write_single_register(struct yam_modbus *bus, uint8_t addr,
                       uint16_t register_addr, uint16_t register_value);
int yam_read_exception_status(struct yam_modbus *bus, uint8_t addr,
                              uint8_t *exception_status);
int yam_write_multiple_coils(struct yam_modbus *bus, uint8_t addr,
                             uint16_t start_addr, uint8_t *coils,
                             uint16_t num_coils);
int yam_write_multiple_registers(struct yam_modbus *bus, uint8_t addr,
                                 uint16_t start_addr, uint16_t *regs,
                                 uint16_t num_regs);
int yam_report_slave_id(struct yam_modbus *bus, uint8_t addr, uint8_t *id,
                        uint8_t *run_status, char *additional_data, int *buflen);

void yam_perror(struct yam_modbus *bus, char *s);
char *yam_strerror(int errnum);

#endif /* _YAM_MODBUS_H_ */

