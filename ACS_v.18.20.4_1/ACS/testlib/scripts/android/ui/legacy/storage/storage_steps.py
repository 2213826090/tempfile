#######################################################################
#
# @filename:    storage_steps.py
# @description: storage steps
# @author:      andreeax.a.vlad@intel.com
#
#######################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
import time

class entry_in_path(adb_step):
    #TODO description
    def __init__(self, path, inode, **kwargs):
        self.path = path
        self.inode = inode
        adb_step.__init__(self, **kwargs)
        self.set_passm(self.inode + " present in " + self.path)
        self.set_errorm("", self.inode + " present in " + self.path)

    def do(self):
        self.grep_out =\
            self.adb_connection.parse_cmd_output(cmd = "ls " + self.path,
                                                 grep_for = self.inode)

    def check_condition(self):
        return self.inode in self.grep_out


class file_manager_usb(ui_step):
    #TODO description
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
    def do(self):
        ui_steps.press_home()()
        ui_steps.press_all_apps()()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the app",
            blocking = True,
            view_to_find = {"text" : "File Manager"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Home"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Up"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Up"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "usbdisk2"})()
    def check_condition(self):
        return self.uidevice(textContains = "/storage/usbdisk2").exists

class copy_from_usb(ui_step):
    #TODO description
    def __init__(self, value, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.value = value
    def do(self):
        file_manager_usb(serial = self.serial)()
        self.uidevice(textContains = self.value).swipe.right(steps = 100)
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Copy"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Home"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Download"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Paste"})()
    def check_condition(self):
        return self.uidevice(textContains = self.value).exists

class erase_copied_file_from_usb(ui_step):
    #TODO description
    def __init__(self, value, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.value = value
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.press_all_apps(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the app",
            blocking = True,
            view_to_find = {"text" : "File Manager"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Home"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Download"})()
        self.uidevice(textContains = self.value).swipe.right(steps = 100)
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Delete"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "OK"})()
    def check_condition(self):
        return not self.uidevice(textContains = self.value).exists

class erase_copied_file(ui_step):
    #TODO description
    def __init__(self, value, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.value = value
    def do(self):
        file_manager_usb(serial = self.serial)()
        self.uidevice(textContains = self.value).swipe.right(steps = 100)
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Delete"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "OK"})()
    def check_condition(self):
        return not self.uidevice(textContains = self.value).exists


class write_to_usb(ui_step):
    #TODO description
    def __init__(self, value, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.value = value
    def do(self):
        ui_steps.press_all_apps(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the app",
            blocking = True,
            view_to_find = {"text" : "File Manager"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Home"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Download"})()
        self.uidevice(textContains = self.value).swipe.right(steps = 100)
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Copy"})()
        ui_steps.press_home(serial = self.serial)()
        file_manager_usb(serial = self.serial)()
        if not self.uidevice(textContains = self.value).exists:
            ui_steps.click_button(serial = self.serial,
                print_error = "Error - Could not find the button",
                blocking = True,
                view_to_find = {"text" : "Paste"})()
        else:
            print "The file already exists!"
    def check_condition(self):
        return self.uidevice(textContains = self.value).exists


class unmount_usb(ui_step):
    #TODO description
    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.press_all_apps(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the app",
            blocking = True,
            view_to_find = {"text" : "Settings"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Storage"})()
        ui_utils.is_text_visible(serial = self.serial,
                                 text_to_find = "Unmount USB storage")
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Unmount USB storage"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "OK"})()
        ui_steps.press_home(serial = self.serial)()
        file_manager_usb(serial = self.serial)()
    def check_condition(self):
        return self.uidevice(text = "( Directory is empty )").exists


class mount_usb(ui_step):
    #TODO description
    def __init__(self,value, **kwargs):
        ui_step.__init__(self,  **kwargs)
        self.value = value
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.press_all_apps(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the app",
            blocking = True,
            view_to_find = {"text" : "Settings"})()
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Storage"})()
        ui_utils.is_text_visible(serial = self.serial,
                                 text_to_find = "Mount USB storage")
        ui_steps.click_button(serial = self.serial,
            print_error = "Error - Could not find the button",
            blocking = True,
            view_to_find = {"text" : "Mount USB storage"})()
        ui_steps.press_home(serial = self.serial)()
        file_manager_usb(serial = self.serial)()
    def check_condition(self):
        return self.uidevice(textContains = self.value).exists
