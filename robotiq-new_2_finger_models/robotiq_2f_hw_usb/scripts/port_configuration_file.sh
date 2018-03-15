#!/bin/bash

# This script is taken from the Robotiq urcap files available at 
# http://support.robotiq.com/pages/viewpage.action?pageId=5963876&_ga=1.226707289.552542445.1469802183

# configuration of virtual port com FT X series
FTDI_CONF_FILE='/etc/modprobe.d/ftdi_sio.conf'
FTDI_CONF_OPTION='options ftdi_sio product=0x6015 vendor=0x0403'
FTDI_CONF_ALIAS='alias usb:v0403p6015d*dc*dsc*dp*ic*isc*ip* ftdi_sio'
if [ -f $FTDI_CONF_FILE ]; then
        MSG="FTDI conf file already exists."
        echo $MSG
else
        MSG="FTDI conf file does not exist. Creating it."
        echo $MSG

        echo "# This file allows the ftdi_sio driver to use FT X series." > $FTDI_CONF_FILE
        echo $FTDI_CONF_OPTION >> $FTDI_CONF_FILE
        echo $FTDI_CONF_ALIAS >> $FTDI_CONF_FILE

        #reload the module for the changes to have effect
        modprobe -r ftdi_sio
        modprobe ftdi_sio
fi