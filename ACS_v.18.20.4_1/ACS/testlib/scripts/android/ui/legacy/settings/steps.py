#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: UI test steps
# @author:      silviux.l.andrei@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.utils.ui import uiandroid
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.base.base_step import step as base_step


class click_radio_button_with_agree(ui_step):
    """returns pass if the radio button is checked."""
    def __init__(self, view_to_find, index = 0, **kwargs):
        ui_steps.ui_step.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.instance = index
        self.set_passm(str(view_to_find))
        self.set_errorm("",str(view_to_find))

    def do(self):
        ui_steps.click_button(print_error = "Failed to check radio",
                      blocking = True,
                      view_to_find = self.view_to_find)()
        if self.uidevice(text = "Agree").exists:
            ui_steps.click_button(print_error = "Failed to close popup",
                      blocking = True,
                      view_to_find = {"text":"Agree"})()

    def check_condition(self):
        return ui_utils.is_radio_button_enabled(self.instance)


class click_checkbox_button_with_ok(ui_step):
    """returns pass if the checkbox button is checked."""
    def __init__(self, view_to_find, **kwargs):
        ui_steps.ui_step.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.set_passm(str(view_to_find))
        self.set_errorm("",str(view_to_find))

    def do(self):
        self.initial_checkbox_state = ui_utils.is_checkbox_checked(
            self.view_to_find)

        ui_steps.click_button(print_error = "Failed to check checkbox",
                      blocking = True,
                      view_to_find = self.view_to_find)()
        if self.uidevice(text = "OK").exists:
            ui_steps.click_button(print_error = "Failed to close popup",
                      blocking = True,
                      view_to_find = {"text":"OK"})()

    def check_condition(self):
        return ui_utils.is_checkbox_checked(self.view_to_find) !=\
               self.initial_checkbox_state


class click_switch_button_with_ok(ui_step):
    """returns pass if the switch button is on."""
    def __init__(self, view_to_find, view_to_check, state = "ON", **kwargs):
        ui_steps.ui_step.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.view_to_check = view_to_check
        self.state = state
        self.set_passm(str(view_to_find))
        self.set_errorm("",str(view_to_find))

    def do(self):
        ui_steps.click_button(print_error = "Failed to switch on",
                      blocking = True,
                      view_to_find = self.view_to_find)()
        if self.uidevice(text = "OK").exists:
            ui_steps.click_button(print_error = "Failed to close popup",
                      blocking = True,
                      view_to_find = {"text":"OK"})()

    def check_condition(self):
        switch = self.uidevice(**self.view_to_check)
        return switch.info["text"] == self.state


class slide_brightness(ui_step, adb_step):
    """change brightness using the slider from Display"""
    def __init__(self, view_to_find, position, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.view_to_find = view_to_find
        self.position = position

    def do(self):
        ui_utils.move_slider(self.view_to_find,
                             position = self.position)

    def check_condition(self):
        real_brightness = int(self.adb_connection.parse_file(file_name =
        "/sys/class/backlight/intel_backlight/brightness"))
        percentange_to_real_brightness = 255 * self.position / 100.0

        self.uidevice(**self.view_to_find).wait.gone(timeout = 5000)
        return (real_brightness - 25 < percentange_to_real_brightness) and \
            (real_brightness + 25 > percentange_to_real_brightness)

class move_volume_slider(base_step):

    def __init__(self, view_to_find, position, db, table, columns, values,
            where_columns, where_values, **kwargs):
        self.view_to_find = view_to_find
        self.position = position
        self.db = db
        self.table = table
        self.columns = columns
        self.values = values
        self.where_columns = where_columns
        self.where_values = where_values
        base_step.__init__(self, **kwargs)
        self.set_passm(str(view_to_find) + " on " + str(self.position))

    def do(self):
        ui_utils.move_slider(view_to_find = self.view_to_find,
                             position = self.position)

    def check_condition(self):
        adb_steps.sqlite_select_query(db = self.db,
                                      table = self.table,
                                      columns = self.columns,
                                      where_columns = self.where_columns,
                                      where_values = self.where_values,
                                      values = self.values)()
        return True


class add_google_account(ui_step):

    def __init__(self, username, password, **kwargs):
        self.uname = username
        self.passwd = paswword
        ui_step.__init__(self, **kwargs)
        self.set_passm("Adding " + self.uname + " account with pass: " +\
                       self.passwd)
        self.set_errorm("", "Adding " + self.uname + " account with pass: " +\
                            self.passwd)
