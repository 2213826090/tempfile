#!/usr/bin/env python

##############################################################################
#
# @filename:    test.311_fallback.py
#
# @description: Testing fallback bindings
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

steps.halctl_filter_fallback(
    print_error = "Error - invalid output for halctl -i type=fallback",
    serial = serial,
    media_path = media_path,
)()
