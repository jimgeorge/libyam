Summary: Yet another Modbus/RTU library. Currently supports slave mode RTU using termios
Name: libyam
Version: 0.1.0
Release: 1
License: LGPL V3+
Packager: Some random Internet user
URL: http://www.chill.colostate.edu
Group: Applications/System 
Provides: libyam=0.1.0
Requires: ,/bin/sh

Source0: libyam-0.1.0.tar.bz2

BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)
BuildRequires: autoconf, automake

%description
Slave mode Modbus/RTU library, using termios.

%prep
%setup -q

autoreconf

%build
%configure 

make


%install
rm -rf $RPM_BUILD_ROOT
mkdir -p -m755 $RPM_BUILD_ROOT/
make install DESTDIR=$RPM_BUILD_ROOT
mkdir -p -m755 $RPM_BUILD_ROOT/usr/share/libyam/
ls -lRh $RPM_BUILD_ROOT/


%clean
rm -rf $RPM_BUILD_ROOT


%files
%defattr(-,root,root)
%attr(0755,root,root) %dir %{_libdir}
%attr(0755,root,root) %dir %{_libdir}/pkgconfig
%attr(0755,root,root) %dir %{_includedir}
%attr(0755,root,root) %dir %{_includedir}/yam/
%dir %{_libdir}/libyam.so.2
%dir %{_libdir}/libyam.so
%attr(0755,root,root) %{_libdir}/libyam.so.2.0.3
%attr(0755,root,root) %{_libdir}/libyam.la
%attr(0644,root,root) %{_libdir}/pkgconfig/yam.pc
%attr(0644,root,root) %{_includedir}/yam/modbus.h
%doc AUTHORS ChangeLog INSTALL NEWS COPYING* README


%changelog

