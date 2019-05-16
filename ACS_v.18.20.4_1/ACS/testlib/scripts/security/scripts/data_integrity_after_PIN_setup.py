#!/usr/bin/env python

# #############################################################################
#
# @filename:    data_integrity_after_pin_setup.py
#
# @description: Checks the integrity of the data after reboot with pin set up
#
# @author:      costin.carabas@intel.com
#
##############################################################################


from testlib.scripts.android.adb import adb_steps
from testlib.scripts.file import file_steps
from testlib.scripts.security.scripts import prerequisites
from testlib.base.base_utils import get_args
import sys

globals().update(vars(get_args(sys.argv)))

# Run Prerequisites
prerequisites.run_prereq(serial = serial,
                         pin = "1234",
                         set_screen_lock = True,
                         blocking = True)()

# CREATE 1GB file
file_steps.create_random_file(file_name = "testfile",
                              size = 1024**3,
                              blocking = True)()

# SEND 1GB file to DUT
adb_steps.push_file(serial = serial,
                    local = "testfile",
                    remote = "/data/local/tmp",
                    timeout = 5000,
                    blocking = True)()

# Reboot DUT
adb_steps.reboot(serial = serial,
                 pin = "1234",
                 ip_enabled = False)()

# Check for sent file
adb_steps.command(serial = serial,
                  command = "ls -l /data/local/tmp/testfile",
                  stdout_grep = str(1024**3))()

# Remake initial state
adb_steps.command(serial = serial,
                  command = "rm /data/local/tmp/testfile",
                  timeout = 50000)()

file_steps.remove_file(file_name = "testfile")()
