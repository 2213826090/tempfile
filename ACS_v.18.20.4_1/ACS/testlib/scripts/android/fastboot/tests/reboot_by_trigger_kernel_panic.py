#!/usr/bin/env python

######################################################################
#
# @filename:    reboot_by_trigger_kernel_panic.py
# @description: Reboot by ADB command and check ADB log.
#
# @run example:
#
#            python reboot_by_trigger_kernel_panic.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
# @author:      haojiex.xie@intel.com
#
#######################################################################

##### imports #####
import os
import re
import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]

##### test start #####
try:
    relay_steps.reboot_main_os(serial = serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = True)()

    adb_steps.root_connect_device(serial = serial)()
    time.sleep(5)

    local_steps.command("adb -s {} shell \"echo c > /proc/sysrq-trigger\"".format(serial))()
    time.sleep(60)

    relay_steps.reboot_main_os(serial = serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = True)()

    adb_steps.root_connect_device(serial = serial)()
    time.sleep(5)

    command = "adb shell cat /proc/cmdline"
    result = False
    r = os.popen(command)
    info = r.readlines()
    for line in info:
        line = line.strip("\r\n")
        line = line.split()
        for s in line:
            if "androidboot.bootreason" in s:
                s = s.split("=")
                if isinstance(s[1], str) and len(s[1].split()) == 1:
                    rule = re.compile(r"[^a-zA-Z]")
                    s[1] = rule.sub("", s[1])
                    if re.match("^[a-z]+$", s[1]):
                        result = True
    if not result:
        raise Exception("The test result did not achieve the desired results")

except:
    raise

finally:
    relay_steps.reboot_main_os(serial = serial,
                               relay_type = relay_type,
                               relay_port = relay_port,
                               power_port = power_port,
                               wait_ui = True)()
##### test end #####