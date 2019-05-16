#!/usr/bin/env python

#######################################################################
#
# @filename:    test_05_sp_verify_repairOS.py
# @description: Checks sp_update is available on DUT in repairOS
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################

from testlib.scripts.connections.ssh import ssh_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.adb import adb_steps

import sys
import time

adb_steps.connect_device(serial = sys.argv[1] + ":5555",
                         port = sys.argv[2])()

adb_steps.reboot(command = "recovery",
                 reboot_timeout = 200,
                 blocking = True)()

ssh_steps.command(command = "ls /usr/bin/sp_verify",
                  stdout_grep = "/usr/bin/sp_verify",
                  stdout_not_grep = "No such file or directory",
                  host = sys.argv[1],
                  user = "root",
                  password = "",
                  sftp_enabled = False)()

ssh_steps.command(command = "reboot",
                  timeout = 200,
                  host = sys.argv[1],
                  user = "root",
                  password = "",
                  sftp_enabled = False)()

time.sleep(10)

local_steps.wait_for_ping(timeout = 100,
                          ip = sys.argv[1])()

