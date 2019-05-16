#!/usr/bin/env python

##############################################################################
#
# @filename:    playstore_utils.py
# @description: UI Playstore test utils
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################


from testlib.utils.ui import uiandroid


def playstore_home(serial = None):
    """
    description:
           Check if PlayStore it's' in home page. If not goes to home page.

        usage:
            playstore_utils.playstore_home(serial = serial)

        tags:
            ui, android, playstore, playstore home
    """

    if serial:
        uidevice = uiandroid.UIDevice(serial = serial)
    else:
        uidevice = uiandroid.UIDevice()
    for f in range(1,6):
        while not uidevice(descriptionContains =\
                "Show navigation drawer").sibling(text = "Play Store").exists:
                    uidevice.press.back()
