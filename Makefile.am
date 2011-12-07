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


####### ARDUINO ########

ARDUINOLIB = libarduino.a
ARDUINOHEADERS = WProgram.h wiring.h WCharacter.h WString.h \
   HardwareSerial.h binary.h Print.h Stream.h 
ARDUINOEXTRAHEADERS = wiring_private.h WConstants.h \
   pins_arduino.h
libarduino_a_SOURCES = HardwareSerial.cpp Tone.cpp WString.cpp \
   Print.cpp  WMath.cpp pins_arduino.c wiring_analog.c wiring_digital.c \
   wiring_shift.c wiring.c wiring_pulse.c WInterrupts.c


####### ARDUINOTESTSUITE ########

# Technically, I don't think avr_cpunames.h needs to be included
# in the installed headers because no other headers use it but 
# if you write an app using the Arduino IDE, it includes it.

ARDUINOTESTSUITELIB = libArduinoTestSuite.a
ARDUINOTESTSUITEHEADERS = ArduinoTestSuite.h avr_cpunames.h
ARDUINOTESTSUITEEXTRAHEADERS = 
libArduinoTestSuite_a_SOURCES = ArduinoTestSuite.cpp


####### EEPROM ########

EEPROMLIB = libEEPROM.a
EEPROMHEADERS = EEPROM.h
EEPROMEXTRAHEADERS = 
libEEPROM_a_SOURCES = EEPROM.cpp


####### ETHERNET ########

ETHERNETLIB = libEthernet.a 
ETHERNETHEADERS = Ethernet.h Client.h Server.h Udp.h 
ETHERNETEXTRAHEADERS = utility/w5100.h utility/socket.h
libEthernet_a_SOURCES = Ethernet.cpp Client.cpp Server.cpp Udp.cpp \
   utility/socket.cpp utility/w5100.cpp SPI.cpp


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


####### MATRIX ########

MATRIXLIB = libMatrix.a
MATRIXHEADERS = Matrix.h
MATRIXEXTRAHEADERS = 
libMatrix_a_SOURCES = Matrix.cpp


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


####### SPRITE ########

SPRITELIB = libSprite.a
SPRITEHEADERS = Sprite.h
SPRITEEXTRAHEADERS = 
libSprite_a_SOURCES = Sprite.cpp


####### STEPPER ########

STEPPERLIB = libStepper.a
STEPPERHEADERS = Stepper.h
STEPPEREXTRAHEADERS = 
libStepper_a_SOURCES = Stepper.cpp


####### WIRE ########

WIRELIB = libWire.a
WIREHEADERS = Wire.h
WIREEXTRAHEADERS = utility/twi.h
libWire_a_SOURCES = Wire.cpp utility/twi.c



####### INSTALLATION ########

include_HEADERS = $(ARDUINOHEADERS) $(STEPPERHEADERS) \
   $(ARDUINOTESTSUITEHEADERS) $(EEPROMHEADERS) $(ETHERNETHEADERS) \
   $(SPIHEADERS) $(FIRMATAHEADERS) $(LIQUIDCRYSTALHEADERS) \
   $(MATRIXHEADERS) $(SPRITEHEADERS) $(SDHEADERS) $(SERVOHEADERS) \
   $(SOFTWARESERIALHEADERS) $(WIREHEADERS)
libcpu_LIBRARIES = $(ARDUINOLIB) $(STEPPERLIB) $(ARDUINOTESTSUITELIB) \
   $(EEPROMLIB) $(ETHERNETLIB) $(SPILIB) $(FIRMATALIB) \
   $(LIQUIDCRYSTALLIB) $(MATRIXLIB) $(SPRITELIB) $(SDLIB) $(SERVOLIB) \
   $(SOFTWARESERIALLIB) $(WIRELIB)

# This is special for SD because it needs a 'utility' subdirectory in the
# include directory.
utildir=$(includedir)/utility
util_HEADERS = $(SDUTILHEADERS) 

# These are headers that we need for compiling but don't need to be
# installed.
EXTRA_DIST = $(ARDUINOEXTRAHEADERS) $(ARDUINOTESTSUITEEXTRAHEADERS) \
   $(ETHERNETEXTRAHEADERS) $(WIREEXTRAHEADERS)


# When uninstalling, blow away the include/utility directory and the 
# lib/mmcu-mhz directory.
uninstall-hook:
	rm -Rf $(utildir) $(libcpudir)
