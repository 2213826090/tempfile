#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Books / Open book
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.gms.books import books_steps

from testlib.base.base_utils import get_args
from testlib.scripts.android.adb import adb_utils
globals().update(vars(get_args(sys.argv)))
adb_steps.connect_device(serial = serial)()
globals().update({"version": adb_utils.get_android_version()})

books_steps.open_my_library(serial = serial)()

books_steps.open_all_from_library()()
books_steps.open_purchases_from_library()()
books_steps.open_samples_from_library()()

ui_steps.press_back(serial = serial)()
ui_steps.press_home(serial = serial)()
