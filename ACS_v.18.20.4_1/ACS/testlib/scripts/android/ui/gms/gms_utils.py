#!/usr/bin/env python

from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.base import base_utils
from testlib.utils.connections.adb import Adb as connection_adb
from testlib.scripts.android.adb import adb_utils


def check_google_account(serial = None):
    """
    description:
        check if a Google account is configured on DUT.
        Return True if the Google account is configured.

    usage:
        gms_utils.check_google_account()

    tags: google account, account, google, check google
    """
    if serial:
        uidevice = ui_device(serial = serial)
        adb_connection = connection_adb(serial = serial)
    else:
        uidevice = ui_device()
        adb_connection = connection_adb()

    if ( "Starting: Intent { act=android.settings.SYNC_SETTINGS }" in\
        adb_connection.parse_cmd_output(\
        cmd = "am start -a android.settings.SYNC_SETTINGS",\
        grep_for = "Starting: Intent")):
            if uidevice(text = "Google").exists:
                uidevice.press.recent()
                uidevice.wait.update()
                if uidevice(text = "Accounts").wait.exists(timeout = 5000):
                    uidevice(text = "Accounts").swipe.right()
                return True
            else:
                uidevice.press.recent()
                uidevice.wait.update()
                if uidevice(text = "Accounts").wait.exists(timeout = 5000):
                    uidevice(text = "Accounts").swipe.right()
                return False
    else:
        print "The settings.SYNC_SETTINGS activity doesn't start"
        return False


def get_google_account_number(serial = None):
    """
    description:
        this fubction return the google account number configured on DUT
        Return 0 if there is no Google account configured.

    usage:
        gms_utils.get_google_account_number()

    tags: google account, account number, google, check google, account
    """

    db = "/data/system/users/0/accounts.db"
    table = "accounts"
    return adb_utils.sqlite_count_query(serial = serial,
                                        db = db, table = table)
