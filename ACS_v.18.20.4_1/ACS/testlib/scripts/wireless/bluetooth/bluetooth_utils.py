#!/usr/bin/env python

#######################################################################
#
# @filename:    bluetooth_steps.py
# @description: Bluetooth test steps
# @author:      nicolas.paccou@intel.com
#
#######################################################################
import time

from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.utils.ui.uiandroid import UIDevice as ui_device


def check_airplane_mode_on(serial = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    cmd = "settings get global airplane_mode_on"
    return adb_connection.parse_cmd_output(cmd, timeout = 5).strip()


def get_switch_state(view_to_find, serial = None):
    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()
    return uidevice(**view_to_find).info['text']


def check_bluetooth_state_on(serial = None):
    if serial:
        adb_connection = connection_adb(serial = serial)
    else:
        adb_connection = connection_adb()
    # below sleep time will help get proper result when there is a sudden
    # toggle in bt power state
    time.sleep(1)
    cmd = "service call bluetooth_manager 5"
    out = adb_connection.parse_cmd_output(cmd=cmd)
    if "00000000 00000001   '........'" in out.strip():
        return True
    elif "00000000 00000000   '........'" in out.strip():
        return False
    else:
        return None
