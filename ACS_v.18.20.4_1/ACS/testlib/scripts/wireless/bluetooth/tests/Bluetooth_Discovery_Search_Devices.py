#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_Discovery_Search_Devices.py
# @description: Search other bluetooth devices
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old

# Connect to device
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

PAIRING_DEVICE_NAME = 'PAIRING DEVICE'
PAIRING_DEVICE = script_args[0]

########### Preconditions ###############
#########################################

adb_steps.connect_device(
    serial = serial,
    local_port = adb_server_port
)()

adb_steps.connect_device(
    serial = PAIRING_DEVICE,
    local_port = adb_server_port,
)()

bluetooth_steps_old.bt_change_device_name(serial = PAIRING_DEVICE,
                                        port = adb_server_port,
                                        name = PAIRING_DEVICE_NAME
                                     )()
bluetooth_steps_old.bt_make_discoverable(serial = PAIRING_DEVICE,
                                        port = adb_server_port,
                                        name = PAIRING_DEVICE_NAME
                                    )()


############### Test ####################
#########################################
bluetooth_steps_old.bt_search_devices(serial = serial,
                                  port = adb_server_port,
                                  dev_to_find = PAIRING_DEVICE_NAME
                                 )()


########### Postconditions ##############
#########################################
adb_steps.disconnect_device(
    serial = serial,
    local_port = adb_server_port
)()

adb_steps.disconnect_device(
    serial = PAIRING_DEVICE,
    local_port = adb_server_port
)()
