#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Books / Open book from first page
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

books_steps.open_first_page_book(serial = serial)()

ui_steps.press_back(serial = serial)()
ui_steps.press_home(serial = serial)()
