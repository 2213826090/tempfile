#!/usr/bin/env python

######################################################################
#
# @filename:    kill_ftp_server.py
# @description: Kills the PID used by the FTP server
#
# @run example:
#
#            python kill_ftp_server.py.py -s 0BA8F2A0
#                                                  --script-args
#                                                  port_number=20211
#                                                  file_name=generated.bin
#
# @author:      alexandru.i.nemes@intel.com
#
#######################################################################

##### imports #####
import sys
import time
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.ap import ap_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base.base_utils import get_args
from testlib.scripts.connections.local import local_steps

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
port_number = args["port_number"]
file_name = args["file_name"]
# kill the PID of the FTP Server in order to release the socket
local_steps.kill_socket(port_number, process_name = "python")()

#deleting test files from working directory directory
files = [ file_name, file_name + "_from_device" ]
for filex in files:
    local_steps.delete_file(filex, serial = serial)()

##### test end #####
