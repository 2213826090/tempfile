#!/usr/bin/env python

##############################################################################
#
# @filename:    test.112_halctl_list.py
#
# @description: Testing few halctl listing commands
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

steps.halctl_list(
    print_error = "Error - invalid output for halctl -l",
    serial = serial,
    media_path = media_path,
)()

steps.halctl_filter_gralloc(
    print_error = "Error - invalid output for halctl -i gralloc",
    media_path = media_path,
)()
