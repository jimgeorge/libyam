dnl Autoconf script for libyam

AC_INIT(libyam, 0.3, jgeorge@engr.colostate.edu)
AC_CONFIG_SRCDIR([yam/modbus.c])
LT_INIT
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE

dnl Silent mode make (use make V=1 to override)
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_PROG_MAKE_SET
AC_PROG_LIBTOOL
AC_PROG_CXX
AC_PROG_RANLIB

AC_C_CONST

AC_HEADER_STDC
AC_CHECK_HEADERS([termios.h	unistd.h fcntl.h arpa/inet.h sys/ioctl.h])

AC_CHECK_FUNCS([ntohs htons poll bzero strtoul])

AC_TYPE_UINT16_T
AC_TYPE_UINT8_T
AC_TYPE_SIZE_T

AC_CONFIG_FILES([
	Makefile
	yam/Makefile
	tests/Makefile
	Doxyfile
	yam.pc
])

AC_OUTPUT
