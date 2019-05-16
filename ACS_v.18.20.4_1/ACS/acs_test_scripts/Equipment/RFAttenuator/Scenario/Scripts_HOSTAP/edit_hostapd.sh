#!/bin/bash

# This script is used to :
# - Stop the current running process of hostapd according to the wlan interface number given by the user
# - Change the SSID and the MAC address (with macchanger)
# - Restart the new hostapd process with the new changes

# Run this script using root permissions
# Syntax to use : edit_hostapd.sh <wlanX> <new_SSID> <new_MAC>


# Check if the syntax is OK

if [ $# -ne 4 ]
then
	echo "Syntax : edit_hostapd.sh <wlanX> <new_SSID> <new_MAC> <new_freq>"
	exit
fi


# Global variables used in this script

CONFIG_PATH=$( cd "$( dirname "$(readlink -f "$0")" )" && pwd )
CONFIG_PATH=${CONFIG_PATH}/hostapd
echo "${CONFIG_PATH}"
INT_WLAN=$1 
NEW_SSID=$2
NEW_MAC=$3
NEW_FREQ=$4

NEW_CH=0

freq_ch1=2412000
freq_ch2=2417000
freq_ch3=2422000
freq_ch4=2427000
freq_ch5=2432000
freq_ch6=2437000
freq_ch7=2442000
freq_ch8=2447000
freq_ch9=2452000
freq_ch10=2457000
freq_ch11=2462000
freq_ch12=2467000
freq_ch13=2472000


if [ $NEW_FREQ -eq $freq_ch1 ]; then
	NEW_CH=1	
	echo "channel 1"
elif [ $NEW_FREQ -eq $freq_ch2 ]; then
	NEW_CH=2	
	echo "channel 2"
elif [ $NEW_FREQ -eq $freq_ch3 ]; then
	NEW_CH=3	
	echo "channel 3"
elif [ $NEW_FREQ -eq $freq_ch4 ]; then
	NEW_CH=4	
	echo "channel 4"
elif [ $NEW_FREQ -eq $freq_ch5 ]; then
	NEW_CH=5	
	echo "channel 5"
elif [ $NEW_FREQ -eq $freq_ch6 ]; then
	NEW_CH=6	
	echo "channel 6"
elif [ $NEW_FREQ -eq $freq_ch7 ]; then
	NEW_CH=7	
	echo "channel 7"
elif [ $NEW_FREQ -eq $freq_ch8 ]; then
	NEW_CH=8	
	echo "channel 8"
elif [ $NEW_FREQ -eq $freq_ch9 ]; then
	NEW_CH=9	
	echo "channel 9"
elif [ $NEW_FREQ -eq $freq_ch10 ]; then
	NEW_CH=10	
	echo "channel 10"
elif [ $NEW_FREQ -eq $freq_ch11 ]; then
	NEW_CH=11	
	echo "channel 11"
elif [ $NEW_FREQ -eq $freq_ch12 ]; then
	NEW_CH=12	
	echo "channel 12"
elif [ $NEW_FREQ -eq $freq_ch13 ]; then
	NEW_CH=13	
	echo "channel 13"
else
	echo "frequency not found"
fi


# Get the PID of the current hostapd process corresponding to the wlan interface number given

pid=`ps -edf | grep hostapd_$INT_WLAN | awk '{print $2}'` # $2 = PID field in process list
kill -s 2 $pid

sleep 1

# Shutdown interface, change MAC address and SSID, then set interface to up

ifconfig $INT_WLAN down
macchanger --mac $NEW_MAC $INT_WLAN
sed -ie "s/^ssid=.*/ssid=$NEW_SSID/" ${CONFIG_PATH}/hostapd_$INT_WLAN.conf # Replace the first matching SSID line in the file (the broadcasted SSID)
sed -ie "s/^channel=.*/channel=$NEW_CH/" ${CONFIG_PATH}/hostapd_$INT_WLAN.conf
ifconfig $INT_WLAN up


# Start the new hostapd process

hostapd -B ${CONFIG_PATH}/hostapd_$INT_WLAN.conf


echo "Changes successfully applied !"
