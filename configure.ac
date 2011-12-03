#
# This is the configure script to build the arduino libraries.  I stole
# chunks of this code from v1.6.8 of the avr-libc confgure.ac file and have
# no idea what I'm supposed to put up here in the header.  
#
# Shawn Wilson <shawn@ch2a.ca)
# Dec 2011
#


AC_INIT(libarduino, 0022-0)
AC_PREREQ(2.59)

# Get the host and build variables.
AC_CANONICAL_BUILD
AC_CANONICAL_HOST

# See if the user specified '--build=`../config.guess` --host=avr' when
# running ../configure.

AC_MSG_CHECKING([if configuring for cross compile])
if test "x${build_alias}" = "x${host_alias}"; then
   if test "x${build_alias}" = "x" ; then
      build_alias='`./config.guess`'
   fi
   AC_MSG_RESULT(no)
   AC_MSG_WARN()
   AC_MSG_WARN(libarduino must be built using an avr cross-compiler.)
   AC_MSG_WARN(Try configuring with:)
   AC_MSG_WARN("$0 --build=$build_alias --host=avr")
   AC_MSG_WARN()
   AC_MSG_ERROR(aborting configure)
else
   AC_MSG_RESULT([yes])
fi

AC_MSG_CHECKING([if target host is avr])
case "$host" in
   avr* )
      AC_MSG_RESULT([yes])
      ;;
   * )
      AC_MSG_RESULT(no)
      AC_MSG_WARN()
      AC_MSG_WARN(libarduino must be built using an avr cross-compiler.)
      AC_MSG_WARN(Try configuring with:)
      AC_MSG_WARN("$0 --build=$build --host=avr")
      AC_MSG_WARN()
      AC_MSG_ERROR(aborting configure)
      ;;
esac

dnl The default check whether the C compiler can create an executable
dnl is inappropriate for us as it requires an existing library.
AC_NO_EXECUTABLES

# Unless the user specified some CXXFLAGS or CFLAGS, set the flags 
# to blank.
AC_SUBST(CXXFLAGS, [${CXXFLAGS:-''}])
AC_SUBST(CFLAGS, [${CFLAGS:-''}])

# Minimum version of automake is 1.8.
AM_INIT_AUTOMAKE([1.8])


# Here are the checks for programs.

AC_PROG_CC
AC_PROG_CXX
AC_CHECK_TOOL(AS, as, as)
AM_PROG_AS
AC_PROG_RANLIB
AC_CHECK_TOOL(AR, ar, ar)

# Make sure that we found the right avr cross-compiler.

case "${CC}" in
   *avr-gcc*) ;;
   *) AC_MSG_ERROR(Wrong C compiler found; You need an avr-gcc installed!) ;;
esac
case "${AS}" in
   *avr-as*) ;;
   *) AC_MSG_ERROR(Wrong assembler found; You need avr-as installed!) ;;
esac
case "${AR}" in
   *avr-ar*) ;;
   *) AC_MSG_ERROR(Wrong archiver found; You need avr-ar installed!) ;;
esac
case "${RANLIB}" in
   *avr-ranlib*) ;;
   *) AC_MSG_ERROR(Wrong ranlib found; You need avr-ranlib installed!) ;;
esac

AC_PROG_INSTALL


# Set the default microcontroller to Arduino Uno
AC_ARG_VAR(MMCU, "Default processor type.")
AC_SUBST(MMCU, [${MMCU:-'atmega328p'}])
AC_ARG_VAR(MHZ, "Default processor speed in MHz.")
AC_SUBST(MHZ, [${MHZ:-'16'}])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
