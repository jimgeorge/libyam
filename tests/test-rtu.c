/**
\file test-rtu.c
\brief Tests the YAM library functionality
\author Jim George
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <yam/modbus.h>
#include <assert.h>
#include <stdint.h>
#include <time.h>

int main(int argc, char *argv[])
{
	struct yam_modbus bs;
	struct yam_modbus *bus;
	uint16_t regs[100];
	int slave_addr = 0x40;
	
	bus = &bs;
	printf("Testing libyam\n");

	if (0 > yam_modbus_init("/dev/ttyUSB0", 57600, bus)) {
		printf("Error initializing bus\n");
		return -1;
	}

	//yam_debug(bus, 1);
	int ret = 0;
	int numregs = 8, start = 1000;
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
	//usleep(100000);

	numregs = 4;
	ret = yam_read_registers(bus, slave_addr, start, numregs, regs);
	if (0 > ret) {
		yam_perror(bus, "Error reading registers");
	}
	else {
		int ctr;
		for (ctr = 0; ctr < numregs; ctr++) {
			printf("Register %d = %d (0x%04X)\n", ctr + start, regs[ctr], regs[ctr]);
		}
	}
	//usleep(100000);

	uint8_t id, run_status;
	char additional_data[256];
	int adl;
	ret = yam_report_slave_id(bus, slave_addr, &id, &run_status, additional_data, &adl);
	if (0 > ret) {
		yam_perror(bus, "Error reading Slave ID");
	}
	else {
		printf("ID: %02X, Run status: %02X, %d additional bytes\n", id, run_status, adl);
	}

	yam_modbus_close(bus);

	printf("Done!\n");
	return 0;
}

