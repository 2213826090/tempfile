#!/usr/bin/env python

##############################################################################
#
# @filename:    test.121_kmod.py
#
# @description: Testing if kmod stub is available and
#               correctly displays the usage
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

steps.kmod(
    print_error = "Error - invalid output for kmod",
    serial = serial,
    media_path = media_path,
)()
