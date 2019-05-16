#!/usr/bin/env python

######################################################################
#
# @filename:    _hotspot_test.py
# @description: Test hotspot related tests
#
# @run example:
#
#            python wifi_turned_off_by_hotspot.py -s 0BA8F2A0
#                                               --script-args
#                                                       hotSpot_band=2.4
#                                                       hotSpot_security=WPA2
#                                                       hotSpot_pass=test1234
#

#
# @author:      kalpana.mandapati@intel.com
#
#######################################################################

##### imports #####
import sys
import time
#from testlib.scripts.wireless.wifi import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.base.base_utils import get_args
from testlib.scripts.android.ui import ui_steps
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps


##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params

hotSpot_band = args["hotSpot_band"]
#hotSpot_security = args["hotSpot_security"]
if "hotSpot_security" in args.keys():
    hotSpot_security=args["hotSpot_security"]
else:
    hotSpot_security = ""
if (hotSpot_security == "") or (hotSpot_security == "None"):
    hotSpot_pass = ""
else:
    hotSpot_pass = args["hotSpot_pass"]

hotSpot_name = args["hotSpot_name"]
serial2= args["serial2"]
no_of_clients =1
if args["no_of_clients"]:
  no_of_clients=args["no_of_clients"]
else:
  no_of_clients = 1
no_clients_back=no_of_clients

# define the hotspot name
#hotSpot_name = "HS_" + serial

# restart as root
adb_steps.root_connect_device(serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# make sure there are no saved networks
wifi_steps.open_wifi_settings(serial= serial)()
wifi_steps.clear_saved_networks(serial = serial)()
wifi_steps.set_wifi(serial = serial, state="OFF")
i=2
while int(no_of_clients) > 0:
    #On reference device
    # turn display on, if turned off on reference device
    ser="serial"+str(i)
    ser=args[ser]
    ui_steps.wake_up_device(serial = ser)()

    # ensure the device is unlocked
    ui_steps.unlock_device(serial = ser, pin=wifi_defaults.wifi['pin'])()

    wifi_steps.open_wifi_settings(serial= ser)()
    # make sure there are no saved networks
    wifi_steps.clear_saved_networks(serial = ser)()
    no_of_clients = int(no_of_clients)-1
    i=int(i)+1


if "ref_hotspot" in args.keys():
    ref_dev = serial2
    dut = serial
    serial = ref_dev

# create the hotspot
wifi_steps.configure_hotSpot(serial = serial, hotSpot_name=hotSpot_name,
                                     hotSpot_security=hotSpot_security,
                                     hotSpot_band=hotSpot_band,
                                     hotSpot_pass=hotSpot_pass)()

# turn on the HotSpot
wifi_steps.set_hotSpot(serial=serial, hotSpot_state="ON")()

# verify the status of the wifi
wifi_steps.check_connection_info(serial = serial,
                                         state = "DISCONNECTED/DISCONNECTED")()
time.sleep(2)
if "iterations" in args.keys():
    iterations=args["iterations"]
else:
    iterations=1
iterations = int(iterations)
i=1
i=int(i)
if "ref_hotspot" in args.keys():
   ser= dut
   # connect to the HotSpot from reference device
   """wifi_steps.add_network(serial = ser,
                               ssid = hotSpot_name,
                               security = hotSpot_security,
                               password = hotSpot_pass)()"""
   if  (hotSpot_security == "") or (hotSpot_security == "None"):
       wifi_steps.connect_wifi_from_UI(ap_name=hotSpot_name, scroll=True, open_wifi_settings=True,
                                       security=False, serial=ser)()
   else:
       wifi_steps.connect_wifi_from_UI(ap_name = hotSpot_name,scroll= True,open_wifi_settings=True,password = hotSpot_pass,serial=ser)()
   # wait until the reference device is connected
   wifi_steps.wait_until_connected(serial = ser)()
   # verify that reference device is connected
   wifi_steps.ping_gateway(serial = ser)()
   while iterations > 1:
          # if iterations !=1:
          print "iterations"
          wifi_steps.set_wifi(serial = ser, state="OFF",use_adb=False)()
          time.sleep(2)
          ui_steps.wait_for_view_common(view_to_find={"textContains": "To see available networks, turn"},
                                      timeout=1000, serial=ser)()
          wifi_steps.set_wifi(serial = ser, state="ON",use_adb=False)()
          wifi_steps.wait_until_connected(serial = ser)()
          # verify that reference device is connected
          wifi_steps.ping_gateway(serial = ser)()
          iterations=iterations-1
else:
	i=2
	no_of_clients=int(no_clients_back)
	while int(no_of_clients) > 0:
         	#On reference device
    	        # turn display on, if turned off on reference device
    	        ser="serial"+str(i)
    	        ser=args[ser]

    	        # connect to the HotSpot from reference device
    	        wifi_steps.add_network(serial = ser,
                               ssid = hotSpot_name,
                               security = hotSpot_security,
                               password = hotSpot_pass)()

    	        # wait until the reference device is connected
   	        wifi_steps.wait_until_connected(serial = ser)()

    	        # verify that reference device is connected
    	        wifi_steps.ping_gateway(serial = ser)()
    	        no_of_clients = int(no_of_clients) - 1
    	        i = int(i) + 1

# Cleaning-turn off the HotSpot
wifi_steps.set_hotSpot(serial=serial, hotSpot_state="OFF")()
wifi_steps.open_wifi_settings(serial= serial)()
wifi_steps.set_wifi(serial = serial, state="ON")

