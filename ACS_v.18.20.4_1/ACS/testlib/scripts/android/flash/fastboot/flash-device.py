#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: HomeScreen / Add icon to homescreen
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.utils.relay import Relay

# Connect to device
from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils

globals().update(vars(get_args(sys.argv)))
#globals().update({"version": adb_utils.get_android_version()})
PATH_TO_BUILD="/home/ccarabas/work/cts-dispatcher/repair/latest"


################################################################################
# Device has to be in fastboot
#   - if already in fastboot, do nothing
#   - if it has adb, reboot in fastboot via adb
#   - for any other state:
#       * force poweroff via relay
#       * boot in fastboot via relay (poweron & volume down)
################################################################################
if local_utils.has_fastboot_serial(serial):
    pass
elif local_utils.has_adb_serial(serial):
    adb_steps.reboot(serial = serial, command="fastboot", ip_enabled=False)()
else:
    ############################################################################
    # device file should be made dynamic to support multiple relays
    ############################################################################
    my_relay = Relay(port = "/dev/ttyACM0")
    my_relay.power_off()
    my_relay.enter_fastboot()
    my_relay.close()

local_steps.wait_for_fastboot(serial = serial,
                              timeout = 20)()

local_steps.change_dir(serial = serial,
                       new_folder=PATH_TO_BUILD)()
local_steps.command(serial = serial,
                    command="./flash-all.sh")()

local_steps.wait_for_adb(serial = serial,
                              timeout = 720)()
