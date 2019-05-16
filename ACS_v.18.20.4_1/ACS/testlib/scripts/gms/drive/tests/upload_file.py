#!/usr/bin/env python

##############################################################################
#
# @filename:    upload_file.py
#
# @description: Upload a file to gdrive
#
# @author:      alexandru.n.branciog@intel.com
#
##############################################################################

import sys
import time
import datetime
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.drive import drive_steps

from testlib.base.base_utils import get_args

args = get_args(sys.argv)
globals().update(vars(args))
globals().update(eval(script_args[0]))

ui_steps.press_home(serial = serial)()

drive_steps.open_drive(serial = serial, account = account,
                        password = password, force = True)()

# upload a file
drive_steps.upload_file(serial = serial, file_name = file_name)()

# Delete the file
drive_steps.remove_file(serial = serial, file_name = file_name)()


# Close app
ui_steps.close_app_from_recent(serial = serial,
                               view_to_find={"text": "Drive"})()
