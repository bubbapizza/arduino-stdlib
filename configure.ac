# Copyright (c) 2004,  Theodore A. Roth
# Copyright (c) 2005,2006,2007,2009  Anatoly Sokolov
# Copyright (c) 2005,2008  Joerg Wunsch
# Copyright (c) 2011  Shawn Wilson
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of the copyright holders nor the names of
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Process this file with autoconf to produce a configure script.
#

#
# This is the autoconf file for building all the standard arduino
# libraries.  The libraries and proper header files get installed during
# the normal install process.
#

AC_INIT(arduino-stdlib, 1.0-0)
AC_PREREQ(2.59)

# Set the default --host parameter to 'avr'.
if test "$host" == "" ; then
   host=${host:-'avr'}
   host_alias=${host_alias:-'avr'}
   ac_tool_prefix=${ac_tool_prefix:-'avr-'}
   cross_compiling=yes
fi


# Get the host and build variables.
AC_CANONICAL_BUILD
AC_CANONICAL_HOST

#
# See if the user specified '--build=`../config.guess` --host=avr' when
# running ../configure.
#
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


#
# This is where all the program variables get set.  Most of them get 
# prefixed with 'avr-' during this process.
#
AC_PROG_CC
AC_PROG_CXX
AC_CHECK_TOOL(AS, as, as)
AM_PROG_AS
AC_PROG_RANLIB
AC_CHECK_TOOL(AR, ar, ar)

# Make sure that we found the right avr cross-compiler.
case "${CC}" in
   *avr-gcc*) ;;
   *) AC_MSG_ERROR(Wrong C compiler found; You need avr-gcc installed!) ;;
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


#
# Allow the user to specify the AVR CPU type an speed in megahertz.
# By default, we set the AVR CPU type to be an Arduino Uno.
#
AC_ARG_VAR(MMCU, "AVR processor type. (Default is atmega328p)")
AC_SUBST(MMCU, [${MMCU:-'atmega328p'}])
AC_ARG_VAR(MHZ, "AVR processor speed in MHz. (Default is 16)")
AC_SUBST(MHZ, [${MHZ:-'16'}])

# This is a new variable needed for pins_arduino.h variants.
AC_ARG_VAR(VARIANT, "Arduino variant for pins_arduino.h. (Default is 'standard')")
AC_SUBST(VARIANT, [${VARIANT:-'standard'}])

AC_CONFIG_FILES([Makefile Arduino.h wiring_digital.c wiring_pulse.c \
                 wiring_analog.c Tone.cpp])
AC_OUTPUT
