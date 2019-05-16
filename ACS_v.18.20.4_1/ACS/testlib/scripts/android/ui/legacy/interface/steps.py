#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: ET/Star Peak/Script Library/L2/Interface
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui.ui_step import step as ui_step
import math


class check_apps_are_sorted(ui_step):
    """
        description:
            Checks if the apps from all apps are displayed in
            alphabetical order.
            L-dessert views are presented order on columns, not
            rows.

        usage:
            check_apps_are_sorted(version = "L",
                                  apps_col_no = 5)()
    """
    def __init__(self, version = "L", apps_cols_no = 5, **kwargs):
        self.version = version
        self.apps_cols_no = apps_cols_no
        ui_step.__init__(self, **kwargs)

    def do(self):
        installed_apps = []
        for item in self.uidevice(className =\
                                           "android.widget.TextView"):
            installed_apps.append(str(item.info['text'])) #item is an app
        if self.version == 'K':
            installed_apps = installed_apps[3:]

        self.installed_apps = []

        if self.version == 'L':
            apps_no = len(installed_apps)
            apps_rows_no = int(math.ceil(apps_no/self.apps_cols_no))
            i = 0
            for i in range(apps_rows_no):
                for j in range(self.apps_cols_no):
                    self.installed_apps.append(installed_apps[j * apps_rows_no + i])
        else:
            self.installed_apps = installed_apps


    def check_condition(self):

        second_list= sorted(self.installed_apps, key = str.lower)
        i = 0
        for entry in second_list:
            if entry != self.installed_apps[i]:
                print entry + " == " + self.installed_apps[i]

            i+=1
        return sorted(self.installed_apps, key=str.lower) == \
                      self.installed_apps

