#!/usr/bin/env python

########################################################################
#
# @filename:
# @description: GUI tests helper functions
# @author:      silviux.l.andrei@intel.com
#
#######################################################################

from testlib.utils.ui.uiandroid import UIDevice as ui_device

def check_apps_are_unique():
    list_sorted = False

    uidevice = ui_device()

    installed_apps = []
    for item in uidevice(className = "android.widget.TextView"):
        installed_apps.append(str(item.info['text'])) #item is the app
    installed_apps = installed_apps[3:]
    #find which app name appears >1
    list_of_duplicates =\
        set([x for x in installed_apps if installed_apps.count(x) > 1])
    print list_of_duplicates

    return len(list_of_duplicates) == 0

