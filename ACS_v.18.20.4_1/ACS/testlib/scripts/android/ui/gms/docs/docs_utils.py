#!/usr/bin/env python

from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.scripts.android.adb import adb_utils


def search_by(name, serial = None, wait_time = 20000):
    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()
    uidevice(description="Search").click()
    uidevice(resourceId = "com.google.android.apps.docs.editors.docs:id/search_text").set_text(name)
    uidevice.press(66)
    uidevice(resourceIdMatches = ".*doc_list_syncing_spinner").wait.gone(timeout=wait_time)

def exit_search(serial = None):
    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()
    if uidevice(description = "Close search").exists:
        uidevice(description = "Close search").click()

