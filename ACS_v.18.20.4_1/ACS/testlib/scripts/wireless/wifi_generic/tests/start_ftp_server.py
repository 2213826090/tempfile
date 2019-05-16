#!/usr/bin/env python

######################################################################
#
# @filename:    start_ftp_server.py
# @description: Tests that the socket needed for FTP server is available
#               and, if so, starts FTP server on that particular socket.
#
# @run example:
#
#            python start_ftp_server.py.py -s 0BA8F2A0
#                                                  --script-args
#                                                  port_number=20211
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
from testlib.scripts.wireless.wifi import wifi_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.wireless.wifi.wifi_utils import FTPService

##### initialization #####
globals().update(vars(get_args(sys.argv)))
args = {}
for entry in script_args:
    key, val = entry.split("=")
    args[key] = val

# mandatory params
port_number = args["port_number"]


# optional params
# the below params are not mandatory for all configurations,
# i.e.: for open wifi

##### test start #####
adb_steps.connect_device(serial = serial,
                         port = adb_server_port)()

# opting to run server on all interfaces
IP = "0.0.0.0"

# make sure that the needed socket is available
local_steps.check_socket_available(IP, port_number)()

# if the socket is available, instantiate and start the server
server = FTPService(ip_address = IP, port_number = port_number)
server.start()

##### test end #####
