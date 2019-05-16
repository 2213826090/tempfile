#!/usr/bin/env python

######################################################################
#
# @filename:    forced_shutdown.py
# @description: Shut down the device using power button.
#
# @run example:
#
#            python forced_shutdown.py --serial=A6EC8F70 --script-args
#                                               relay_type=RLY08B
#                                               relay_port=/dev/serial/by-id/usb-Devantech_Ltd._USB-RLY08_00015234-if00
#                                               power_port=4
#
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import os
import sys
from testlib.base.base_utils import get_args
from testlib.scripts.relay import relay_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.connections.local import local_steps
from testlib.base.ParallelSteps import ParallelSteps
import time

##### initialization #####
globals().update(vars(get_args(sys.argv)))

args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

relay_type = args["relay_type"]
relay_port = args["relay_port"]
power_port = args["power_port"]
USB_VC_cut_port = args["USB_VC_cut"]

##### test start #####
# ensure the DUT is in main OS
relay_steps.reboot_main_os(serial=serial,
                             relay_type = relay_type,
                             relay_port = relay_port,
                             power_port = power_port,
                             force_reboot = False)()

command = "adb shell cat /system/build.prop | grep \"ro.product.model\""
r = os.popen(command)
info = r.readlines()
for line in info:
  line = line.strip("\r\n")
  line = line.split("=")
  if line[1] == "cht_mrd" or line[1] == "Generic androidia_64":
    # power off DUT
    relay_steps.power_off_device(serial=serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port,
                                 except_charging=True)()
  else:
    # power off DUT
    relay_steps.power_off_device(serial=serial,
                                 relay_type = relay_type,
                                 relay_port = relay_port,
                                 power_port = power_port,
                                 except_charging=True)()
    # ensure COS is loaded
    local_steps.wait_for_cos(serial=serial)()

# disconnect adb
relay_steps.connect_disconnect_usb(serial=serial,
                           relay_type = relay_type,
                           relay_port = relay_port,
                           connect=False,
                           USB_VC_cut_port=USB_VC_cut_port)()

# power on device
psteps = ParallelSteps(use_control_process=False)
step_id_power_on = psteps.add_step(relay_steps.power_on_device,
                                  serial=serial,
                                  timeout = 120,
                                  relay_type = relay_type,
                                  relay_port = relay_port,
                                  power_port = power_port)
# wait for a while without USB connection
time.sleep(60)

# reconnect adb
relay_steps.connect_disconnect_usb(serial=serial,
                           relay_type = relay_type,
                           relay_port = relay_port,
                           connect=True,
                           USB_VC_cut_port=USB_VC_cut_port)()
# finalize the power on step
psteps.interpret_step(step_id_power_on)

adb_steps.wait_for_ui_processes(serial = serial)()
##### test end #####