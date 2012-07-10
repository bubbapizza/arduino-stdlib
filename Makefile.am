#
#        Copyright (C) 2011 Shawn Wilson
#        shawn@ch2a.ca
#        
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

#
# This config generates a makefile for building the arduino library
# and the other 13 libraries that come with the normal arduino IDE.
#


AM_CXXFLAGS = -Wall -Os -mmcu=$(MMCU) -DF_CPU=$(MHZ)000000 
AM_CFLAGS = -Wall -Os -mmcu=$(MMCU) -DF_CPU=$(MHZ)000000 
ARFLAGS = rc

# The libraries will get installed in a directory named according to 
# the CPU type they were compiled for.
libcpudir = $(libdir)/$(MMCU)-$(MHZ)

# The include files will get installed in a directory named according
# to the pin-variant type they were compiled for.
includepindir = $(includedir)/arduino-$(VARIANT)


####### ARDUINO ########

ARDUINOLIB = libarduino.a

# These headers are the headers that get installed so apps can link
# to libarduino.h and some of the other libraries.  For example, Client.h
# and Server.h are needed by libEthernet.
ARDUINOHEADERS = Arduino.h binary.h WCharacter.h HardwareSerial.h \
   Stream.h USBAPI.h Print.h WString.h Printable.h  Client.h Server.h \
   IPAddress.h Udp.h new.h

STANDARDPINHEADERS = variants/standard/pins_arduino.h 
LEONARDOPINHEADERS = variants/leonardo/pins_arduino.h 
EIGHTANALOGINPUTSPINHEADERS = variants/eightanaloginputs/pins_arduino.h 
MEGAPINHEADERS = variants/mega/pins_arduino.h

# These headers need to be included in the distribution so we can compile
# the libarduino.a sources.
ARDUINOEXTRAHEADERS = wiring_private.h  Platform.h Printable.h Stream.h \
   USBCore.h USBDesc.h

libarduino_a_SOURCES = HardwareSerial.cpp Tone.cpp WString.cpp \
   Print.cpp  WMath.cpp wiring_analog.c wiring_digital.c \
   wiring_shift.c wiring.c wiring_pulse.c WInterrupts.c main.cpp \
   CDC.cpp IPAddress.cpp USBCore.cpp Stream.cpp HID.cpp new.cpp  


####### EEPROM ########

EEPROMLIB = libEEPROM.a
EEPROMHEADERS = EEPROM.h
EEPROMEXTRAHEADERS = 
libEEPROM_a_SOURCES = EEPROM.cpp


####### ETHERNET ########

ETHERNETLIB = libEthernet.a 
ETHERNETHEADERS = Dns.h util.h EthernetClient.h EthernetUdp.h \
   EthernetServer.h Ethernet.h Dhcp.h
ETHERNETEXTRAHEADERS = socket.h w5100.h
libEthernet_a_SOURCES = Dhcp.cpp EthernetClient.cpp EthernetServer.cpp \
   Dns.cpp Ethernet.cpp EthernetUdp.cpp utility/socket.cpp \
   utility/w5100.cpp


####### FIRMATA ########

FIRMATALIB = libFirmata.a
FIRMATAHEADERS = Firmata.h Boards.h
FIRMATAEXTRAHEADERS = 
libFirmata_a_SOURCES = Firmata.cpp


####### LIQUIDCRYSTAL ########

LIQUIDCRYSTALLIB = libLiquidCrystal.a
LIQUIDCRYSTALHEADERS = LiquidCrystal.h
LIQUIDCRYSTALEXTRAHEADERS = 
libLiquidCrystal_a_SOURCES = LiquidCrystal.cpp


####### SD ########

SDLIB = libSD.a
SDHEADERS = SD.h 
SDUTILHEADERS = utility/SdFat.h utility/SdFatUtil.h utility/Sd2Card.h \
   utility/FatStructs.h utility/Sd2PinMap.h utility/SdInfo.h
SDEXTRAHEADERS = 
libSD_a_SOURCES = SD.cpp File.cpp utility/Sd2Card.cpp utility/SdFile.cpp \
   utility/SdVolume.cpp


####### SERVO ########

SERVOLIB = libServo.a
SERVOHEADERS = Servo.h
SERVOEXTRAHEADERS = 
libServo_a_SOURCES = Servo.cpp


####### SOFTWARESERIAL ########

SOFTWARESERIALLIB = libSoftwareSerial.a
SOFTWARESERIALHEADERS = SoftwareSerial.h
SOFTWARESERIALEXTRAHEADERS = 
libSoftwareSerial_a_SOURCES = SoftwareSerial.cpp


####### SPI ########

SPILIB = libSPI.a
SPIHEADERS = SPI.h
SPIEXTRAHEADERS = 
libSPI_a_SOURCES = SPI.cpp


####### STEPPER ########

STEPPERLIB = libStepper.a
STEPPERHEADERS = Stepper.h
STEPPEREXTRAHEADERS = 
libStepper_a_SOURCES = Stepper.cpp


####### WIRE ########

WIRELIB = libWire.a
WIREHEADERS = Wire.h
WIREEXTRAHEADERS = twi.h
libWire_a_SOURCES = Wire.cpp utility/twi.c



####### INSTALLATION ########

includepin_HEADERS = $(ARDUINOHEADERS) $(STEPPERHEADERS) \
   $(EEPROMHEADERS) $(ETHERNETHEADERS) \
   $(SPIHEADERS) $(FIRMATAHEADERS) $(LIQUIDCRYSTALHEADERS) \
   $(MATRIXHEADERS) $(SPRITEHEADERS) $(SDHEADERS) $(SERVOHEADERS) \
   $(SOFTWARESERIALHEADERS) $(WIREHEADERS)
libcpu_LIBRARIES = $(ARDUINOLIB) $(STEPPERLIB) \
   $(EEPROMLIB) $(ETHERNETLIB) $(SPILIB) $(FIRMATALIB) \
   $(LIQUIDCRYSTALLIB) $(MATRIXLIB) $(SPRITELIB) $(SDLIB) $(SERVOLIB) \
   $(SOFTWARESERIALLIB) $(WIRELIB)

# This is special for SD because it needs a 'utility' subdirectory in the
# include directory.
utildir=$(includepindir)/utility
util_HEADERS = $(SDUTILHEADERS) 

#
# All these directories and files are needed for different arduino
# pin configurations.
# 
standarddir=$(includepindir)/variants/standard
standard_HEADERS=$(STANDARDPINHEADERS)
megadir=$(includepindir)/variants/mega
mega_HEADERS=$(MEGAPINHEADERS)
leonardodir=$(includepindir)/variants/leonardo
leonardo_HEADERS=$(LEONARDOPINHEADERS)
eightanaloginputsdir=$(includepindir)/variants/eightanaloginputs
eightanaloginputs_HEADERS=$(EIGHTANALOGINPUTSPINHEADERS)


# These are headers that we need for compiling but don't need to be
# installed.
EXTRA_DIST = $(ARDUINOEXTRAHEADERS) $(ARDUINOTESTSUITEEXTRAHEADERS) \
   $(ETHERNETEXTRAHEADERS) $(WIREEXTRAHEADERS)


# When uninstalling, blow away the include/utility directory and the 
# lib/mmcu-mhz directory.
uninstall-hook:
	rm -Rf $(utildir) $(libcpudir) $(includepindir)
