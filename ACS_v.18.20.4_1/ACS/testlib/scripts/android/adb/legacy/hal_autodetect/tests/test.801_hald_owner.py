#!/usr/bin/env python

##############################################################################
#
# @filename:    test.801_hald_owner.py
#
# @description: Testing if hald runs as hal
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

steps.check_hald_owner(
    print_error = "Error - hald not running as hal",
    serial = serial,
    media_path = media_path,
)()
