#!/usr/bin/env python

######################################################################
#
# @filename:    band_selection.py
# @description: Tests that the DUT will correctly apply the frequency
#               band setting
#               frequency bands: 2 / 5 / auto
#
# @run example:
#
#            python band_selection.py -s 0BA8F2A0
#                                               --script-args
#                                                       test_type=set_freq_2
#                                                       wait_time=5
#
#           test_types: set_freq_2 / set_freq_5 / set_freq_auto /
#                       setting_toggle_wifi / setting_reboot_DUT /
#                       reconnect_2.4GHz / reconnect_5GHz
#
# @author:      stefanx.todirica@intel.com
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
from testlib.base.base_utils import get_args
from testlib.scripts.ap import ap_steps
from random import choice
from string import ascii_letters, digits
from testlib.utils.defaults import wifi_defaults

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
if script_args:
    for entry in script_args:
        key, val = entry.split("=")
        args[key] = val

# mandatory params
test_type = args["test_type"]

# test_name = args["test_type"]

# optional params
wait_time = 5
if "wait_time" in args.keys():
    wait_time = args["wait_time"]

##### test start #####
# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(serial = serial, pin=wifi_defaults.wifi['pin'])()

# go to home screen
ui_steps.press_home(serial = serial)()

wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = 'auto', wait_time = wait_time, verify_dumpsys = False)()

# make sure there are no saved networks
wifi_generic_steps.clear_saved_networks(serial = serial)()

# generate unique ssids (to avoid the bug with ssid not disappearing from list - reproduced on nexus)
new_ssid = 'tempname'+''.join(choice(ascii_letters + digits) for i in range(3))

# change ap name (also for bypassing bug with SSID not disappearing from list)
ap_steps.setup(mode = 'n', security = 'wpa2',
               encryption = 'aes',
               wifi_password = 'qwerasdf',
               new_ssid = new_ssid,
               interface5ghz = '0',
               serial = serial)()

# change ap name (also for bypassing bug with SSID not disappearing from list)
ap_steps.setup(mode = 'n', security = 'wpa2',
               encryption = 'aes',
               wifi_password = 'qwerasdf',
               new_ssid = new_ssid+'5',
               interface5ghz = '1',
               serial = serial)()

if test_type == 'set_freq_2':
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '2', wait_time = wait_time)()

if test_type == 'set_freq_5':
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '5', wait_time = wait_time)()

if test_type == 'set_freq_auto':
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = 'auto', wait_time = wait_time)()

if test_type == 'setting_toggle_wifi':
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '5', wait_time = wait_time, verify_dumpsys = False)()
    wifi_steps.set_wifi_state(serial = serial, state = 'OFF')()
    wifi_steps.set_wifi_state(serial = serial, state = 'ON')()
    wifi_steps.open_wifi_settings(serial = serial)()
    # Click "More"
    ui_steps.click_button(serial = serial,
        view_to_find = {"descriptionContains": "More"},
        view_to_check = {"textContains": "Add network"})()
    # Click "Advanced"
    ui_steps.click_button(serial = serial,
        view_to_find = {"textContains": "Advanced"},
        view_to_check = {"textContains": "5 GHz only"})()
    wifi_steps.check_scanned_ssid_frequencies(serial = serial, frequency_band = '5')

if test_type == 'setting_reboot_DUT':
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '5', wait_time = wait_time, verify_dumpsys = False)()
    # reboot DUT
    adb_steps.reboot(serial = serial, ip_enabled=False)()
    wifi_steps.open_wifi_settings(serial = serial)()
    # Click "More"
    ui_steps.click_button(serial = serial,
        view_to_find = {"descriptionContains": "More"},
        view_to_check = {"textContains": "Add network"})()
    # Click "Advanced"
    ui_steps.click_button(serial = serial,
        view_to_find = {"textContains": "Advanced"},
        view_to_check = {"textContains": "5 GHz only"})()
    wifi_steps.check_scanned_ssid_frequencies(serial = serial, frequency_band = '5')

if test_type == 'reconnect_2.4GHz':
    wifi_generic_steps.scan_and_check_ap(new_ssid+'5', trycount=10, should_exist=True, serial = serial)()

    # add the 2.4GHz Wi-Fi network
    wifi_generic_steps.connect_with_password(ap_name = new_ssid,
                               password = 'qwerasdf',
                               serial = serial)()

    wifi_generic_steps.wait_until_connected(serial = serial)()

    # add the 5GHz Wi-Fi network
    wifi_generic_steps.connect_with_password(ap_name = new_ssid+'5',
                               password = 'qwerasdf',
                               serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the 5GHz network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = new_ssid+'5',
                                            state='CONNECTED/CONNECTED')()

    # avoid the bug which always shows last AP to which the DUT was connected <= verify_dumpsys = False
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '2', wait_time = wait_time, verify_dumpsys = False)()

    # wait until the device connects to a wifi network
    time.sleep(5)
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the 2.4GHz network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = new_ssid,
                                            state='CONNECTED/CONNECTED')()

if test_type == 'reconnect_5GHz':
    wifi_generic_steps.scan_and_check_ap(new_ssid+'5', trycount=30, should_exist=True, serial = serial)()

    # add the 5GHz Wi-Fi network
    wifi_generic_steps.connect_with_password(ap_name = new_ssid+'5',
                               password = 'qwerasdf',
                               serial = serial)()

    wifi_generic_steps.wait_until_connected(serial = serial)()

    # add the 2.4GHz Wi-Fi network
    wifi_generic_steps.connect_with_password(ap_name = new_ssid,
                               password = 'qwerasdf',
                               serial = serial)()

    # wait until the device connects to a wifi network
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the 5GHz network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = new_ssid,
                                            state='CONNECTED/CONNECTED')()

    # avoid the bug which always shows last AP to which the DUT was connected <= verify_dumpsys = False
    wifi_generic_steps.set_wifi_frequency_band(serial = serial, frequency_band = '5', wait_time = wait_time, verify_dumpsys = False)()

    # wait until the device connects to a wifi network
    time.sleep(5)
    wifi_generic_steps.wait_until_connected(serial = serial)()

    # check we are connected to the 2.4GHz network.
    wifi_generic_steps.check_connection_info(serial = serial,
                                            SSID = new_ssid+'5',
                                            state='CONNECTED/CONNECTED')()

##### test end #####
