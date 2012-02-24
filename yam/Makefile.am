AM_CPPFLAGS = -Wall
ACLOCAL_AMFLAGS = -I m4

lib_LTLIBRARIES = libyam.la
libyam_la_SOURCES = serial.c modbus.c modbus.h
libyam_la_LDFLAGS = -version-info 2:0:0

# Include files to install
libyamincludedir = $(includedir)/yam
libyaminclude_HEADERS = modbus.h

# Include files that are part of the source, but not installed
noinst_HEADERS = serial.h

CLEANFILES = *~
