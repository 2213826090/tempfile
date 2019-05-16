#!/usr/bin/env python

from testlib.utils.connections.adb import Adb as connection_adb
from testlib.utils.ui.uiandroid import UIDevice as ui_device


def check_for_download_file(value_str, serial = None):
    if serial:
        adb_connection = connection_adb(serial)
    else:
        adb_connection = connection_adb()
    cmd = "ls /storage/sdcard0/Download"
    return value_str in adb_connection.parse_cmd_output(cmd, timeout = 5).strip()


def get_open_tabs_no(serial = None):
    if serial:
        uidevice = ui_device(serial)
    else:
        uidevice = ui_device()

    return uidevice(resourceId = "com.android.chrome:id/tab_title").count

