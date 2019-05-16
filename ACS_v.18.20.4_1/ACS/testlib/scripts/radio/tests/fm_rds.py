#!/usr/bin/env python

######################################################################
#
# @filename:    fm_rds.py
# @description: FM Stack shall support:
#                Radio Data System (RDS/RBDS),
#                Program Identification Code (PI),
#                Program Service Name (PS),
#                Program Type Code (PTY),
#                Programme Type Name (PTYN),
#                Radio Text Message (RT),
#                Traffic Announcement (TA)
#
# @run example:
#
#            python fm_rds.py -s 0BA8F2A0
#                                                  --script-args
#                                                  frequency=9100
#                                                   pty_check=True
#                                                   ta_check=True
#                                                   pi_check=True
#                                                   reset_rds=True
# @author:      dragosx.nicolaescu@intel.com
#
#######################################################################

##### imports #####

import sys
import time
import random
import re
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.utils.defaults import wifi_defaults
from testlib.scripts.android.logcat import logcat_steps
from testlib.scripts.radio import radio_steps
from testlib.utils.defaults import radio_defaults
from testlib.scripts.connections.local import local_steps
from testlib.external import radio

##### initialization #####

globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params

radio_name = random.choice(radio_defaults.radio_name_values)
ta = "on"
pi = random.randint(20001, 99999)
pty = random.choice(radio_defaults.pty_rds.keys())

if "frequency" in args.keys():
    frequency = args["frequency"]
else:
    # Select the frequency param with the lowest power from the scan results
    radio_path = radio.__file__
    if radio_path.endswith("c"):
        radio_path = radio_path[:-1]
    # Start the scan
    a = local_steps.command(command = radio_path + " -s")()
    # Select the frequency
    b = re.findall("Measuring (\d+:\d+)", a[0])
    c = sorted(zip([int(i.split(":")[1]) for i in b], [int(i.split(":")[0]) for i in b]))
    frequency = random.choice([c[0][1], c[1][1], c[2][1]])

# optional params

if "pty_check" in args.keys():
    pty_check = args["pty_check"]
else:
    pty_check = False

if "ta_check" in args.keys():
    ta_check = "yes"
else:
    ta_check = False

if "pi_check" in args.keys():
    pi_check = args["pi_check"]
else:
    pi_check = False

if "reset_rds" in args.keys():
    reset_rds = args["reset_rds"]
else:
    reset_rds = False

##### start Ada Radio ####

radio.start_radio(" -n " + str(radio_name) + " -f " + str(frequency) + " --TA " + ta + " --PI " + str(pi) + " --PTY " + str(pty))

##### test start #####

# turn display on, if turned off
ui_steps.wake_up_device(serial = serial)()

# ensure the device is unlocked
ui_steps.unlock_device(pin=wifi_defaults.wifi['pin'],serial = serial)()

# clear logcat
logcat_steps.clear_logcat(serial=serial)()

# start the FM Radio app
radio_steps.start_fmradio(serial = serial, frequency = frequency, rds = "ON", ta = "ON", af = "ON", wait_time = 10)()

# check the Radio Data System (RDS) data is supported
logcat_steps.grep_for(serial=serial, grep_for_text = "radio_hw: PS: " + radio_name, text_presence = True)()

# check the Program Type Code (PTY) is supported
if pty_check:
    logcat_steps.grep_for(serial=serial, grep_for_text = "radio_hw: PTY: " + str(pty), text_presence = True)()

# check the Traffic Announcement (TA) is supported
if ta_check:
    logcat_steps.grep_for(serial=serial, grep_for_text = "radio_hw: TA: " + ta_check, text_presence = True)()

# check the Program Identification Code (PI) is supported
if pi_check:
    logcat_steps.grep_for(serial=serial, grep_for_text = "radio_hw: PI: " + str(hex(pi)).lstrip("0x1"), text_presence = True)()

# turn RDS/RDBS on/off while tuned to a station broadcasting RDS information
if reset_rds:
    radio_steps.toggle_rds(serial=serial, iterations = 1)()
    logcat_steps.grep_for(serial=serial, grep_for_text = "radio_hw: PS: " + radio_name, text_presence = True)()

# go to home screen
ui_steps.press_home(serial = serial)()

##### test end #####
