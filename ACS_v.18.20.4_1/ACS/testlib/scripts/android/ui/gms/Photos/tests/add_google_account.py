#!/usr/bin/env python

##############################################################################
#
# @filename:    add_google_account.py
#
# @description: add a google account in order to run the Photos tests
#
# @author:      gabriel.porumb@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.Photos import photos_steps
from testlib.base.base_utils import get_args


args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

ui_steps.press_home(serial = serial)()

photos_steps.add_account(serial = serial, account = account,
                        password = password, force = True)()

