#!/usr/bin/env python

##############################################################################
#
# @filename:    test.122_kmod_list.py
#
# @description: Testing kmod -n command
#
# @author:      alexandrux.n.branciog@intel.com
#
##############################################################################

import sys
from testlib.scripts.hal_autodetect import hal_steps as steps
from testlib.base.base_utils import get_args

# usage:
#       python test.<test_name>.py --serial 192.168.1.1:5555
args = get_args(sys.argv)
globals().update(vars(args))

kmodules = [
    {"ecs_e7":'bmc150_accel', "t100":'brcmfmac', "xps12":'iwlwifi'},
    {"ecs_e7":'i2c_hid', "t100":'wmi', "xps12":'btusb'},
]

for kmodule in kmodules:
    steps.kmod_list(
        print_error = " Incorrect output",
        serial = serial,
        kmodule = kmodule,
        media_path = media_path,
    )()
