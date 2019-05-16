#!/usr/bin/env python

#######################################################################
#
# @filename:    security_utils.py
# @description: Security tests helper functions
# @author:      costin.carabas@intel.com
#
#######################################################################

from testlib.utils.ui import uiandroid
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.base import base_utils
from testlib.utils.connections.adb import Adb
import time


def is_dut_encrypted(serial = None):

    """ description:
            check if dut is encrypted
        usage:
            ui_utils.is_dut_encrypted()
        tags:
        ui, settings, security, encrypt, android
    """
    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    ui_steps.open_security_settings(serial = serial)()
    if uidevice(scrollable = True).exists:
        result = uidevice(scrollable = True).scroll.to(text = "Encrypted")
    else:
        result = uidevice(text = "Encrypted").exists
    uidevice.press.home()
    return result


def is_storage_hardware_backed(serial = None):

    """ description:
            check if developer options is enabled
        usage:
            ui_utils.is_developer_options_enabled()
        tags:
            ui, settings, developer, android
    """
    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    ui_steps.open_security_settings(serial = serial)()
    if uidevice(scrollable = True).exists:
        result = uidevice(scrollable = True).scroll.to(text = "Hardware-backed")
    else:
        result = uidevice(text = "Hardware-backed").exists
    uidevice.press.home()
    return result
