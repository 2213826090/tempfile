#!/usr/bin/env python

##############################################################################
#
# @filename:    test.116_halctl_get_module.py
#
# @description: Testing halctl -g command
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

modules = ["power", "lights", "bluetooth"]

for module in modules:
    steps.halctl_get_module(
        print_error = "Error - invalid output for halctl -g {0}".format(\
                                                                module),
        serial = serial,
        media_path = media_path,
        module = module
    )()
