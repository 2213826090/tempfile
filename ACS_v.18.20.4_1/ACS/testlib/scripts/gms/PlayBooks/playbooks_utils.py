#!/usr/bin/env python

##############################################################################
#
# @filename:    playstore_steps.py
# @description: UI Playstore test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.utils.ui import uiandroid


def books_home(serial = None):

    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    if not uidevice(text = "Read Now").exists:
        uidevice(description = "Show navigation drawer").click()
        uidevice(text = "Read Now").click()
    elif uidevice(text = "Shop").exists:
        uidevice(text = "Read Now").click()
