#!/usr/bin/env python

######################################################################
#
# @filename:    connect_disconnect_from_UI.py
# @description: Tests that the DUT can connect to a network and
#               disconnect from it (iteratively) using the WiFi Settings UI.
#
# @run example:
#
#            python connect_disconnect_from_UI.py -s 0BA8F2A0
#                            --script-args
#                                mode=n
#                                ap_name=ddwrt
#                                passphrase=test1234
#             Optional:   scenario=wifioff_scanning_check
#			  ap2_name = secondap
#
# @author:      kalpana.mandapati@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.ap import ap_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
mode = args["mode"]
ddwrt_ap1_name =  args["ap_name"]
if "ap1_ip" in args.keys():
    ddwrt_ap1_ip=args["ap1_ip"]
else:
    ddwrt_ap1_ip = "192.168.1.1"
if "ap1_security" in args.keys():
    if args["ap1_security"] == "none" :
        ddwrt_ap1_pass = ""
    else:
        ddwrt_ap1_pass =args["password"]
if "ap_band" in args.keys():
    ddwrt_ap1_band = args["ap_band"]
else:
    ddwrt_ap1_band = "0"
if "security" in args.keys():
    security = args["security"]
else:
    security = "wpa2"
if "ap1_channel_bw" in args.keys():
    ddwrt_ap1_channel_bw = args["ap1_channel_bw"]
else:
    ddwrt_ap1_channel_bw = "20"
if "ap2_channel_bw" in args.keys():
    ddwrt_ap2_channel_bw = args["ap2_channel_bw"]
else:
    ddwrt_ap2_channel_bw = "20"
encryption = "aes"
if "roaming" in args.keys():
   ddwrt_ap2_name = ddwrt_ap1_name
   args["ap2_name"] = ddwrt_ap2_name
   args["ap2_passphrase"]= ddwrt_ap1_pass
   args["ap2_encryption"] = encryption
   args["ap2_security"] = args["ap1_security"]
   if "ap2_mode" in args.keys():
      mode = args["ap2_mode"]
if "scenario" in args.keys():
    scenario=args["scenario"]
else:
    scenario = ""
if "ap2_name" in args.keys():
      ap2_mode = args["ap2_mode"]
      ddwrt_ap2_name =  args["ap2_name"]
      ddwrt_ap2_ip =  args["ap2_ip"]
      ddwrt_ap2_pass = args["ap2_passphrase"]
      ap2_encryption = args["ap2_encryption"]
      if "ap2_band" in args.keys():
        ddwrt_ap2_band = args["ap2_band"]
      else:
        ddwrt_ap2_band = "0"
      if "ap2_security" in args.keys():
        ddwrt_ap2_security = args["ap2_security"]
      else:
        ddwrt_ap2_security = args["security"]

i = 1
if "iterations" in args.keys():
    i = args["iterations"]

if "adb" in args.keys():
    adb=args["adb"]
else:
    adb="True"
##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# configure ap1
ap_steps.setup(mode, security,
               encryption = encryption,
               wifi_password = ddwrt_ap1_pass,
               new_ssid = ddwrt_ap1_name,
               interface5ghz = ddwrt_ap1_band,
               channel_bw = ddwrt_ap1_channel_bw,
               serial = serial)()

#configure second AP
if "ap2_name" in args.keys():
       ap_steps.setup(mode, security,
                encryption = encryption,
                ssh_host = ddwrt_ap2_ip,
                wifi_password = ddwrt_ap2_pass,
                new_ssid = ddwrt_ap2_name,
                interface5ghz = ddwrt_ap2_band,
                channel_bw=ddwrt_ap2_channel_bw,
                serial = serial)()

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

# open wifi settings menu
wifi_generic_steps.set_wifi(serial = serial)()

# make sure there are no saved networks
wifi_steps.clear_saved_networks(serial = serial)()
ui_steps.press_home(serial=serial)()
wifi_steps.open_wifi_settings(serial=serial)()
wifi_steps.set_wifi(serial = serial,use_adb=False)()
i= int(i)
if "roaming" in args.keys():
    """This is to execute roaming scenario tests"""
    ap_steps.set_ap_wireless(state="off",ssh_host=ddwrt_ap2_ip, interface5g=ddwrt_ap2_band)()
    wifi_steps.connect_with_password(ap_name = ddwrt_ap1_name,scroll= True,open_settings=False,password = ddwrt_ap1_pass,serial=serial)()
    time.sleep(3)
    """ to execute CWS_WLAN Roaming tests , use the key "roaming_ping" to script args"""
    if "roaming_ping" in args.keys():
        print "entering roaming ping"
        wifi_steps.ping_ip(serial=serial, ip=ddwrt_ap1_ip, trycount=10, timeout=30)()
        wifi_steps.ping_ip(serial=serial, ip=ddwrt_ap2_ip,trycount=10,timeout=30,negative=True)()
    else:
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()
    ap_steps.set_ap_wireless(state="off", interface5g=ddwrt_ap1_band, ssh_host=ddwrt_ap1_ip)()
    ap_steps.set_ap_wireless(state="on", interface5g=ddwrt_ap2_band, ssh_host=ddwrt_ap2_ip)()
    wifi_steps.wait_until_connected(serial=serial)()
    time.sleep(3)
    if "roaming_ping" in args.keys():
        wifi_steps.ping_ip(serial=serial, ip=ddwrt_ap2_ip, trycount=10, timeout=30)()
        wifi_steps.ping_ip(serial=serial, ip=ddwrt_ap1_ip, trycount=10, timeout=30, negative=True)()
    else:
        wifi_steps.ping_gateway(serial = serial, trycount=10, timeout=30)()
    #ap_steps.set_ap_wireless(state="off", interface5g=ddwrt_ap2_band, ssh_host=ddwrt_ap2_ip)()


elif scenario == "forget_ap_test" :
   """this is to execute the tests : """
   while i > 0:
        """ Scenarios related to - General Behaviour tests- Forget Wifi AP"""
	#connect wifi to AP1
	wifi_steps.connect_wifi_from_UI(ap_name = ddwrt_ap1_name,scroll= True,open_wifi_settings=False,password = ddwrt_ap1_pass,serial=serial)()
        time.sleep(2)
        #ping to gateway
	wifi_steps.ping_gateway(serial = serial, trycount=10, timeout=30)()

        #wait for ap2 ssid from the list
	ui_steps.wait_for_view_with_scroll(view_to_find={"textContains": ddwrt_ap2_name}, timeout=1000, serial=serial)()
	#connect wifi to AP2
	wifi_steps.connect_wifi_from_UI(ap_name=ddwrt_ap2_name,scroll = True,
                                          password=ddwrt_ap2_pass,open_wifi_settings=False,
                                          serial=serial)()
        #ping to gateway
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()

	#Once DUt is connected to AP2, AP1 status on DUT should be saved
	ui_steps.wait_for_view_common(serial=serial, timeout=30000,view_to_find={"textContains": ddwrt_ap1_name},
                                   optional=False, second_view_to_find={"textMatches":"Saved"})()
	#Tap on first ap , this should connect to first ap with out providing the security key
	ui_steps.click_button_common(serial=serial, view_to_find={"textMatches":ddwrt_ap1_name})()
        time.sleep(3)
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()

	ui_steps.wait_for_view_with_scroll(view_to_find={"textContains": "Forget"},timeout=1000 ,serial = serial)()
	ui_steps.click_button_common(serial=serial, view_to_find={"textMatches": "Forget"})()
	#clean up to go to next iteration
        wifi_steps.forget_wifi_network(serial=serial,ap_name=ddwrt_ap1_name)()
	i = i-1

elif scenario == "check_password" :
        """ To test - general behaviour test - Show wifi password"""
        #connect device to AP , while entering password, password should be displayed .
        wifi_steps.connect_with_password(ap_name = ddwrt_ap1_name,scroll= True,open_settings=False,password = ddwrt_ap1_pass,show_password=True,serial=serial)()
        time.sleep(2)
        #ping from device to gateway
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()

        #teadown step- disconnect device from AP
        wifi_steps.forget_wifi_network(serial=serial,ap_name=ddwrt_ap1_name)()
elif scenario == "reconnect_wifi_on_off":
        """ To test - Wifi reconnect test cases"""
        #connect device to AP without password
        wifi_steps.connect_with_password(ap_name = ddwrt_ap1_name,scroll= True,security=False,serial=serial)()
        time.sleep(2)

        #ping from device to gateway
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()

        #Turn off Wifi and check the wifi is off or not
        ui_steps.click_button(serial=serial,
                              view_to_find={"resourceId": "com.android.settings:id/switch_text"},
                              view_to_check={"textContains": "Off"})()
        ui_steps.wait_for_view_common(view_to_find={"text": ddwrt_ap1_name},
                                      timeout=1000, serial=serial, optional=True, return_value=True)()
        ui_steps.wait_for_view_common(view_to_find={"textContains": "To see available networks, turn"},
                                      timeout=1000, serial=serial)()

        #switch on Wifi
        ui_steps.click_button(serial=serial,
                              view_to_find={"resourceId": "com.android.settings:id/switch_text"},
                              view_to_check={"textContains": "On"})()
        time.sleep(3)
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()

        #clear the connection
        wifi_steps.forget_wifi_network(serial=serial,ap_name=ddwrt_ap1_name)()
elif  scenario == "wifioff_scanning_check"  :
        """ to test - Genearl behaviour test - scanning list should not be displayed when Wifi is off"""
        #check configured AP ap1_name is listed in wifi UI or not
        #make sure wifi is turned on
        wifi_status=ui_utils.get_view_text(serial= serial,view_to_find={"resourceId": "com.android.settings:id/switch_text"})
        if wifi_status is "Off":
            ui_steps.click_button(serial= serial,
                                  view_to_find={"resourceId": "com.android.settings:id/switch_text"} ,
                                  view_to_check = {"text":"On"})()

        #check configured ap name is available on DUT
	ui_steps.wait_for_view(view_to_find={"textMatches":ddwrt_ap1_name},timeout=1000 ,serial = serial)()

        #Turn off the Wifi and check for the message "to see available networks, turn on "
        ui_steps.click_button(serial=serial,
                              view_to_find={"resourceId": "com.android.settings:id/switch_text"},
                              view_to_check={"textContains": "Off"})()
        ui_steps.wait_for_view_common(serial=serial,view_to_find={"text":ddwrt_ap1_name},
                                         timeout=1000 ,optional=True,scroll=True)()
        ui_steps.wait_for_view(view_to_find={"textContains" : "To see available networks"},
                               timeout=3000 ,serial = serial)()
elif scenario == "disable_enable_ap":
        """ To test - wifi behaviour when AP is disable/enable Ap"""
        #connect device to AP
        wifi_steps.connect_wifi_from_UI(ap_name = ddwrt_ap1_name,scroll= True,open_wifi_settings=False,security=True,password = ddwrt_ap1_pass,serial=serial)()
        time.sleep(2)
        #verify the ping
        wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()
        #disable/enable the 3rd party AP
        while i > 0:
            #disable the wifi interface on AP
            ap_steps.set_ap_wireless(state="off",interface5g="1")()
            time.sleep(10)
            ui_steps.wait_for_view_common(view_to_find={"textContains": ddwrt_ap1_name},
                                      timeout=1000, serial=serial,scroll=True,optional=True)()
            #enable the wifi interface on AP
            ap_steps.set_ap_wireless(state="on", interface5g="1")()
            time.sleep(3)
            ui_steps.wait_for_view_common(view_to_find={"textContains": ddwrt_ap1_name},
                              timeout=1000, scroll=True, serial=serial)()
            #verify the ping
            time.sleep(5)
            wifi_steps.ping_gateway(serial=serial, trycount=10, timeout=30)()
            i= i-1
else:
    """ start the stress test"""
    x=time.time()
    wifi_steps.connect_disconnect_UI_stress(ap_name = ddwrt_ap1_name,
                               password = ddwrt_ap1_pass,
                               iterations = i,
                               scroll=True,
                               open_settings=False,
                               ping = True,
                               forget_ap= True,
                               serial = serial)()
    y=time.time()
    d=int(y)-int(x)
    print d
# go to home screen
ui_steps.press_home(serial = serial)()

##### test end #####
