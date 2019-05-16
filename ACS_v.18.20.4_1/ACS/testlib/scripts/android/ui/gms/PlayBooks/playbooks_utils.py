#!/usr/bin/env python

##############################################################################
#
# @filename:    playstore_steps.py
# @description: UI Playstore test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.utils.ui.uiandroid import UIDevice as ui_device


def books_home(serial = None):

    if serial:
        uidevice = ui_device(serial = serial)
    else:
        uidevice = ui_device()
    if not uidevice(text = "Read Now").exists:
        uidevice(description = "Show navigation drawer").click()
        uidevice(text = "Read Now").click()
    elif uidevice(text = "Shop").exists:
        uidevice(text = "Read Now").click()
