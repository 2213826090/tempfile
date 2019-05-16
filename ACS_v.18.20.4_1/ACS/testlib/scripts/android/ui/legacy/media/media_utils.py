#!/usr/bin/env python

#######################################################################
#
# @filename:    bluetooth_steps.py
# @description: Bluetooth test steps
# @author:      nicolas.paccou@intel.com
#
#######################################################################

from testlib.utils.connections.adb import Adb as connection_adb


def is_media(path, grep_for, serial = None):
    if serial:
        adb_connection = connection_adb(serial)
    else:
        adb_connection = connection_adb()
    a = adb_connection.parse_cmd_output(cmd = 'ls ' + path, grep_for = grep_for)
    return a != ''
