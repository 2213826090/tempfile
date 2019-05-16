#!/usr/bin/env python

##############################################################################
#
# @filename:    sign_in.py
#
# @description: Sign in to youtube
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.youtube import youtube_steps

from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

ui_steps.press_home(serial = serial)()

youtube_steps.open_youtube(serial = serial)()

# Sign in adding a google account
youtube_steps.sign_in(serial = serial, account = account,
                        password = password, force = True)()

# Sign in using existing account
# First sign out
youtube_steps.sign_out(serial = serial)()

youtube_steps.sign_in(serial = serial, account = account,
                        password = password, existing = True, force = False)()

# Close app
ui_steps.close_app_from_recent(serial = serial,
        view_to_find={"text": "YouTube"})()
