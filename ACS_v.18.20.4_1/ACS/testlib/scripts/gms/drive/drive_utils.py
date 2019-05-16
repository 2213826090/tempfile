#!/usr/bin/env python

from testlib.utils.connections.adb import Adb
from testlib.utils.ui import uiandroid
from testlib.scripts.android.adb import adb_utils

def search_by(name, serial = None, wait_time = 20000):
    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    uidevice(description="Search").click()
    uidevice(resourceId = "com.google.android.apps.docs:id/search_text").set_text(name)
    uidevice.press(66)
    uidevice(resourceIdMatches = ".*doc_list_syncing_spinner").wait.gone(timeout=wait_time)
    # clear search history suggestions and remove keyboard by focusing
    # next item on the right
    uidevice.press(22)

def exit_search(serial = None):
    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    if uidevice(description = "Close search").exists:
        uidevice(description = "Close search").click()

