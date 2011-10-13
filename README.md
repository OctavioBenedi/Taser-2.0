#Taser 2.0

Taser is a simple way to communicate to serial ports like telnet connections.

Taser is oriented to be easy of use, portable and lightweight.

##Dependencies
Threads library should be installed on your system.

##Installation
Copy taser to your path or use sudo make install

##Compile
A Makefile for x86 is provided so just run make and done.

##Usage

taser -h
Usage: ./taser [OPTIONS] Port
Options:
        --help: display this help and exit
        --verbose: set verbose mode
        --discover: Automatically try to discover system serial ports
        --cr: replace \n with \r\n
        --speed [value]: set value as serial port speed
                Allowed speed values are: 1200 2400 4800 9600 19200 38400 57600 115200
Author: Octavio Benedi Sanchez



Ex: Verbose and CR mode on

taser --verbose --cr /dev/ttyUSB0

##Tips
If you don't know for sure serial line port you can provide several ports on command, first availiable port will be used.

taser --verbose --cr /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2

If your kernel updates /dev/serial with serial USB devices you can use auto discovery option

taser --discover

