/**
\file test-rtu.c
\brief Tests the YAM library functionality
\author Jim George
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <yam/modbus.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <getopt.h>

enum {
	OPT_DEBUG,
	OPT_TIMEOUT,
	OPT_DEVICE,
	OPT_SLAVEADDR,
	OPT_RUNSTATUS,
	OPT_READCOILS,
	OPT_READDISCRETES,
	OPT_READINPUT,
	OPT_READREGISTER,
	OPT_WRITECOIL,
	OPT_WRITEREGISTER,
	OPT_WRITECOILS,
	OPT_WRITEREGISTERS,
};

char *usage_string =
"Test libyam\n"
"Usage: test-rtu [options] [commands]\n"
"Options:\n"
"--debug: Enable debug mode\n"
"--timeout=val: Set timeout (in milliseconds, default = 1 sec)\n"
"--device=dev[,baudrate[,bits[,par[,stop]]]: Set serial device\n"
"             (default: /dev/ttyUSB0, 57600 bps, 8b, Even parity, 1 stop bit)\n"
"--address=addr: Set slave address\n"
"\n"
"Modbus commands:\n"
"--runstatus: Get the running status\n"
"--readcoils=start[,num]: Read specified coils\n"
"--readdiscretes=start[,num]: Read specified discrete inputs\n"
"--readinput=start[,num]: Read specified inputs\n"
"--readregister=start[,num]: Read specified register\n"
"--writecoil=num,val: Write value to specified coil\n"
"--writeregister=num,val: Write value to specified register\n"
"--writecoils=num,val[,num]: Write value to specified coils\n"
"--writeregisters=num,val[,num]: Write value to specified registers\n"
"\n";

int main(int argc, char *argv[])
{
	struct yam_modbus bs;
	struct yam_modbus *bus = NULL;
	uint16_t regs[100];
	uint8_t coils[100];
	int slave_addr = 0x40;
	int opt_idx, opt_errors = 0, opt;
	int baudrate = 57600;
	int serial_flags = 0;
	char serdev[YAM_MAX_DEVICE_NAME] = "/dev/ttyUSB0";

	bus = &bs;

	static struct option opt_lst[] = {
		{"debug", no_argument, 0, OPT_DEBUG},
		{"timeout", required_argument, 0, OPT_TIMEOUT},
		{"device", required_argument, 0, OPT_DEVICE},
		{"address", required_argument, 0, OPT_SLAVEADDR},
		{"runstatus", no_argument, 0, OPT_RUNSTATUS},
		{"readcoils", required_argument, 0, OPT_READCOILS},
		{"readdiscretes", required_argument, 0, OPT_READDISCRETES},
		{"readinput", required_argument, 0, OPT_READINPUT},
		{"readregister", required_argument, 0, OPT_READREGISTER},
		{"writecoil", required_argument, 0, OPT_WRITECOIL},
		{"writeregister", required_argument, 0, OPT_WRITEREGISTER},
		{"writecoils", required_argument, 0, OPT_WRITECOILS},
		{"writeregisters", required_argument, 0, OPT_WRITEREGISTERS},

		{NULL, 0, 0, 0}
	};

	if (argc == 1) {
		puts(usage_string);
		return -1;
	}

	bzero(bus, sizeof(struct yam_modbus));
	yam_modbus_init(serdev, baudrate, YAM_SERIAL_FLAGS_DEFAULT, bus);

	int ret = 0;
	int numregs = 1, start = 40210;
	char *save;
	int value;

	while (-1 != (opt = getopt_long(argc, argv, "dv", opt_lst, &opt_idx))) {
		switch(opt) {
		case OPT_DEBUG:
			if (bus->serial == 0) goto bus_not_initialized;
			yam_debug(bus, 1);
			break;
		case OPT_TIMEOUT:
			if (bus->serial == 0) goto bus_not_initialized;
			yam_set_timeout(bus, strtoul(optarg, NULL, 10));
			break;
		case OPT_DEVICE:
			if (bus->serial == 0) goto bus_not_initialized;
			{
			yam_modbus_close(bus);
			char *delims=", ";
			strncpy(serdev, strtok(optarg, delims), YAM_MAX_DEVICE_NAME);
			char *baudstr = strtok(NULL, delims);
			if (baudstr != NULL) baudrate = strtoul(baudstr, NULL, 10);
			else baudrate = 57600;
			char *bitsstr = strtok(NULL, delims);
			if (bitsstr != NULL) {
				if (!strcasecmp(bitsstr, "8")) {
					serial_flags |= YAM_SERIAL_FLAGS_8BIT;
				}
				else if (!strcasecmp(bitsstr, "7")) {
					serial_flags |= YAM_SERIAL_FLAGS_7BIT;
				}
				else if (!strcasecmp(bitsstr, "6")) {
					serial_flags |= YAM_SERIAL_FLAGS_6BIT;
				}
				else {
					printf("Could not parse number of bits\n");
					return -1;
				}
			}
			else {
				serial_flags |= YAM_SERIAL_FLAGS_8BIT;
			}
			char *parstr = strtok(NULL, delims);
			if (parstr != NULL) {
				if (!strcasecmp(parstr, "N")) {
					serial_flags |= YAM_SERIAL_FLAGS_NO_PARITY;
				}
				else if (!strcasecmp(parstr, "E")) {
					serial_flags |= YAM_SERIAL_FLAGS_EVEN_PARITY;
				}
				else if (!strcasecmp(parstr, "O")) {
					serial_flags |= YAM_SERIAL_FLAGS_ODD_PARITY;
				}
				else {
					printf("Could not parse parity type\n");
					return -1;
				}
			}
			else {
				serial_flags |= YAM_SERIAL_FLAGS_NO_PARITY;
			}
			char *stopstr = strtok(NULL, delims);
			if (stopstr != NULL) {
				if (!strcasecmp(stopstr, "1")) {
					serial_flags |= YAM_SERIAL_FLAGS_ONE_STOP;
				}
				else if (!strcasecmp(stopstr, "2")) {
					serial_flags |= YAM_SERIAL_FLAGS_TWO_STOP;
				}
				else {
					printf("Could not parse number of stop bits\n");
					return -1;
				}
			}
			else {
				serial_flags |= YAM_SERIAL_FLAGS_ONE_STOP;
			}
			if (0 > yam_modbus_init(serdev, baudrate, serial_flags , bus)) {
				printf("Error initializing bus with device %s at %d bps\n",
					serdev, baudrate);
				return -1;
			}
			}
			break;
		case OPT_SLAVEADDR:
			slave_addr = strtoul(optarg, NULL, 16);
			break;
		case OPT_RUNSTATUS:
			if (bus->serial == 0) goto bus_not_initialized;
			{
			uint8_t id, run_status, exception_status;
			char additional_data[256];
			int adl;
			ret = yam_report_slave_id(bus, slave_addr, &id, &run_status, additional_data, &adl);
			if (0 > ret) {
				yam_perror(bus, "Error reading Slave ID");
			}
			else {
				printf("ID: %02X, Run status: %02X, %d additional bytes\n", id, run_status, adl);
			}
			ret = yam_read_exception_status(bus, slave_addr, &exception_status);
			if (0 > ret) {
				yam_perror(bus, "Error reading exception status");
			}
			else {
				printf("Exception status: %02X (%d)\n", exception_status, exception_status);
			}
			}
			break;
		case OPT_READCOILS:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) numregs = strtoul(save + 1, NULL, 10);
			else numregs = 1;
			ret = yam_read_coils(bus, slave_addr, start, numregs, coils);
			if (0 > ret) {
				yam_perror(bus, "Error reading coils");
			}
			else {
				int ctr;
				for (ctr = 0; ctr < numregs; ctr++) {
					printf("Coil %d = %s\n", ctr + start, coils[ctr] ? "ON" : "OFF");
				}
			}
			break;
		case OPT_READDISCRETES:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) numregs = strtoul(save + 1, NULL, 10);
			else numregs = 1;
			ret = yam_read_discretes(bus, slave_addr, start, numregs, coils);
			if (0 > ret) {
				yam_perror(bus, "Error reading discrete inputs");
			}
			else {
				int ctr;
				for (ctr = 0; ctr < numregs; ctr++) {
					printf("Discrete input %d = %s\n", ctr + start, coils[ctr] ? "ON" : "OFF");
				}
			}
			break;
		case OPT_READINPUT:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) numregs = strtoul(save + 1, NULL, 10);
			else numregs = 1;
			ret = yam_read_inputs(bus, slave_addr, start, numregs, regs);
			if (0 > ret) {
				yam_perror(bus, "Error reading registers");
			}
			else {
				int ctr;
				for (ctr = 0; ctr < numregs; ctr++) {
					printf("Input %d = %d (0x%04X)\n", ctr + start, regs[ctr], regs[ctr]);
				}
			}
			break;
		case OPT_READREGISTER:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) numregs = strtoul(save + 1, NULL, 10);
			else numregs = 1;
			ret = yam_read_registers(bus, slave_addr, start, numregs, regs);
			if (0 > ret) {
				yam_perror(bus, "Error reading registers");
			}
			else {
				int ctr;
				for (ctr = 0; ctr < numregs; ctr++) {
					printf("Input %d = %d (0x%04X)\n", ctr + start, regs[ctr], regs[ctr]);
				}
			}
			break;
		case OPT_WRITECOIL:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) value = strtoul(save + 1, NULL, 10);
			else value = 0;
			ret = yam_write_single_coil(bus, slave_addr, start, value);
			if (0 > ret) {
				yam_perror(bus, "Error writing coil");
			}
			break;
		case OPT_WRITEREGISTER:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) value = strtoul(save + 1, NULL, 16);
			else value = 0;
			ret = yam_write_single_register(bus, slave_addr, start, value);
			if (0 > ret) {
				yam_perror(bus, "Error writing register");
			}
			break;
		case OPT_WRITECOILS:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) value = strtoul(save + 1, &save, 10);
			else value = 0;
			if (0 != *save) numregs = strtoul(save + 1, NULL, 10);
			else numregs = 1;
			for (ret = 0; ret < numregs; ret++) {
				coils[ret] = value;
			}
			ret = yam_write_multiple_coils(bus, slave_addr, start, numregs, coils);
			if (0 > ret) {
				yam_perror(bus, "Error writing coils");
			}
			break;
		case OPT_WRITEREGISTERS:
			if (bus->serial == 0) goto bus_not_initialized;
			start = strtoul(optarg, &save, 10);
			if (0 != *save) value = strtoul(save + 1, &save, 10);
			else value = 0;
			if (0 != *save) numregs = strtoul(save + 1, NULL, 16);
			else numregs = 1;
			for (ret = 0; ret < numregs; ret++) {
				regs[ret] = value;
			}
			ret = yam_write_multiple_registers(bus, slave_addr, start, numregs, regs);
			if (0 > ret) {
				yam_perror(bus, "Error writing register");
			}
			break;
		default:
			opt_errors++;
			break;
		}
	};
	if (opt_errors) {
		puts(usage_string);
	}
	yam_modbus_close(bus);

	return 0;

bus_not_initialized:
	printf("Default device (%s) not available\n", serdev);
	puts ("Serial bus not initialized");
	return -1;
}

