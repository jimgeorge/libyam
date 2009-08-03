/**
\file serial.h
\brief Include file YAM serial port handling.
\author Jim George
*/

#ifndef _YAM_SERIAL_H_

int serial_port_init(const char *device_name,
	unsigned int speed,
	int *port);
void serial_port_flush(int fd);

#define _YAM_SERIAL_H_

#endif /* _YAM_SERIAL_H_ */

