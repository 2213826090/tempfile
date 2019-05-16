#!/usr/bin/env python

##############################################################################
#
# @filename:    test.104_boot_hal_impact_time.py
#
# @description: Testing if hald boot impact time is less than 3% of
#               the total boot time
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

steps.boot_time_impact(
    print_error = "Error - boot time impact too high",
    serial = serial,
    media_path = media_path,
)()

