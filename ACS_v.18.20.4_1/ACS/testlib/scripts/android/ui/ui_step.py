#!/usr/bin/env python

########################################################################
#
# @filename:    ui_step.py
# @description: UI test step
# @author:      ion-horia.petrisor@intel.com
#
########################################################################

from testlib.scripts.android.android_step import step as android_step
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.scripts.android.ui import ui_utils
import os

class step(android_step):
    """
        helper class for all gui test steps
    """
    uidevice = None

    def __init__(self, **kwargs):
        android_step.__init__(self, **kwargs)
        self.uidevice = ui_device(**kwargs)

        if os.environ.has_key("TESTLIB_UI_WATCHERS_GROUP"):
            ui_utils.register_watchers(serial = self.serial,
                                       ui_watcher_groups =\
                                       os.environ["TESTLIB_UI_WATCHERS_GROUP"])
        elif kwargs.has_key("ui_watcher_group"):
            ui_utils.register_watchers(serial = self.serial,
                                       ui_watcher_groups =\
                                       kwargs["ui_watcher_group"])
        else:
            pass
            #ui_utils.remove_watchers(serial = self.serial)


        if os.environ.has_key("TESTLIB_UI_HANDLERS_GROUP"):
            ui_utils.register_handlers(serial = self.serial,
                                       ui_handler_groups =\
                                       os.environ["TESTLIB_UI_HANDLERS_GROUP"])
        elif kwargs.has_key("ui_handler_group"):
            ui_utils.register_handlers(serial = self.serial,
                                       ui_handler_groups =\
                                       kwargs["ui_handler_group"])


    def take_picture(self, file_name):
        try:
            self.uidevice.screenshot(file_name)
        except:
            pass

    def get_ui_dump(self, file_name):
        # save UI dump
        try:
            uidevice = ui_device(serial = self.serial)
            uidevice.dump(out_file = file_name,
                          compressed = False,
                          serial = self.serial)
        except:
            pass
