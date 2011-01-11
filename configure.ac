dnl Autoconf script for libyam

AC_INIT(libyam, 0.2, jgeorge@engr.colostate.edu)
AC_CONFIG_SRCDIR([yam/modbus.c])
dnl Remove me after debugging is done
LT_INIT
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE

dnl Silent mode make (use make V=1 to override)
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_PROG_MAKE_SET
AC_PROG_LIBTOOL

AC_HEADER_STDC
AC_CHECK_HEADERS([termios.h unistd.h])

AC_C_CONST
AC_CHECK_FUNCS([ntohs htons poll])

AC_OUTPUT([
	Makefile
	yam/Makefile
	tests/Makefile
	Doxyfile
	yam.pc
])

